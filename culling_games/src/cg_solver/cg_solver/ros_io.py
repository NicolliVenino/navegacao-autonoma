"""Nó cliente ROS 2: a única parte do projeto que conversa com o simulador.

O algoritmo (planejamento.py / exploracao.py) só enxerga quatro operações:
get_map(), move(direcao), sense() e reset(). Todas são síncronas: a chamada
bloqueia até a resposta chegar, e os movimentos saem estritamente em sequência.

Por que o loop do algoritmo não fica dentro de um callback: chamar um serviço
de forma síncrona dentro de um callback trava o executor (deadlock). Por isso
o algoritmo roda no fluxo principal e o nó só "gira" (spin) enquanto espera
uma resposta ou uma leitura de sensor; o callback do sensor roda nesses giros.
"""
import time

import rclpy
from cg_interfaces.msg import RobotSensors
from cg_interfaces.srv import GetMap, MoveCmd, Reset
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .planejamento import caminho_para_direcoes, grid_de_lista_achatada

# Campo da mensagem RobotSensors -> deslocamento (linha, coluna)
OFFSETS_SENSOR = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
    'up_left': (-1, -1),
    'up_right': (-1, 1),
    'down_left': (1, -1),
    'down_right': (1, 1),
}


class RosIO(Node):
    def __init__(self, delay=0.0, espera_sensor=0.03, espera_reset=0.5,
                 timeout_servico=10.0):
        super().__init__('cg_solver')
        self.delay = delay                    # pausa após cada movimento (para o vídeo)
        self.espera_sensor = espera_sensor    # descarta leituras dos primeiros ~30 ms
        self.espera_reset = espera_reset      # o pygame só aplica o reset no próximo frame
        self.timeout_servico = timeout_servico

        self.move_cli = self.create_client(MoveCmd, '/move_command')
        self.map_cli = self.create_client(GetMap, '/get_map')
        self.reset_cli = self.create_client(Reset, '/reset')
        for cliente in (self.move_cli, self.map_cli, self.reset_cli):
            while not cliente.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'Esperando o serviço {cliente.srv_name} (o simulador está rodando?)')

        self.last_sensor = None
        self.contador_sensor = 0
        self.last_move_time = 0.0
        self.movimentos = 0
        # O simulador publica com QoS "sensor_data" (best effort). Um assinante com a
        # QoS padrão (reliable) fica incompatível e não recebe nada, sem dar erro.
        self.create_subscription(RobotSensors, '/culling_games/robot_sensors',
                                 self._on_sensor, qos_profile_sensor_data)

    # ------------------------------------------------------------------ internos
    def _on_sensor(self, msg):
        self.last_sensor = msg
        self.contador_sensor += 1

    def _call(self, cliente, req):
        futuro = cliente.call_async(req)
        rclpy.spin_until_future_complete(self, futuro, timeout_sec=self.timeout_servico)
        if not futuro.done() or futuro.result() is None:
            raise RuntimeError(f'Sem resposta do serviço {cliente.srv_name}')
        return futuro.result()

    # ---------------------------------------------------------------- operações
    def get_map(self):
        """Mapa completo do simulador como matriz de strings ('b', 'f', 'r', 't')."""
        resp = self._call(self.map_cli, GetMap.Request())
        linhas, colunas = (int(v) for v in resp.occupancy_grid_shape)
        return grid_de_lista_achatada(resp.occupancy_grid_flattened, linhas, colunas)

    def move(self, direction):
        """Anda uma célula. Retorna (sucesso, robot_pos, target_pos)."""
        req = MoveCmd.Request()
        req.direction = direction
        resp = self._call(self.move_cli, req)
        self.last_move_time = time.monotonic()
        self.movimentos += 1
        if self.delay:
            time.sleep(self.delay)
        return (bool(resp.success),
                tuple(int(v) for v in resp.robot_pos),
                tuple(int(v) for v in resp.target_pos))

    def sense(self, timeout=5.0):
        """Leitura dos 8 vizinhos: {(d_linha, d_coluna): 'b' | 'f' | 't'}.

        O tópico é best effort a 100 Hz, então logo depois de um movimento pode
        chegar uma leitura feita ANTES dele (com os vizinhos da célula anterior).
        Para evitar isso: espera ~30 ms após o último movimento girando o nó
        (o que esvazia a fila de mensagens antigas) e só então aceita leituras,
        exigindo duas mensagens novas.
        """
        limite = self.last_move_time + self.espera_sensor
        while time.monotonic() < limite:
            rclpy.spin_once(self, timeout_sec=0.005)

        base = self.contador_sensor
        prazo = time.monotonic() + timeout
        while self.contador_sensor < base + 2:
            if time.monotonic() > prazo:
                raise RuntimeError(
                    'Nenhuma leitura em /culling_games/robot_sensors. '
                    'O simulador está rodando?')
            rclpy.spin_once(self, timeout_sec=0.05)

        msg = self.last_sensor
        return {delta: getattr(msg, campo) for campo, delta in OFFSETS_SENSOR.items()}

    def reset(self, map_name=''):
        """Reinicia o labirinto (map_name vazio = o mapa atual). Retorna o nome do mapa."""
        req = Reset.Request()
        req.is_random = False
        req.map_name = map_name
        resp = self._call(self.reset_cli, req)
        if not resp.success:
            raise RuntimeError(f'O simulador recusou o reset para o mapa {map_name!r}')
        time.sleep(self.espera_reset)
        self.last_move_time = time.monotonic()  # leituras anteriores ao reset são velhas
        return resp.loaded_map_name

    # ------------------------------------------------------------ utilitário
    def executar_caminho(self, caminho):
        """Executa um caminho [inicio, ..., alvo] conferindo cada passo.

        Malha aberta (o plano já está pronto), mas cada resposta é verificada:
        a robot_pos tem que ser exatamente a próxima célula do plano.
        Retorna a posição final e a posição do alvo informada pelo simulador.
        """
        pos, alvo = tuple(caminho[0]), None
        for direcao, esperado in zip(caminho_para_direcoes(caminho), caminho[1:]):
            sucesso, pos, alvo = self.move(direcao)
            if not sucesso or pos != tuple(esperado):
                raise RuntimeError(
                    f'Movimento {direcao!r} falhou: esperado {tuple(esperado)}, '
                    f'robô em {pos}. O mapa e o plano divergem.')
        return pos, alvo
