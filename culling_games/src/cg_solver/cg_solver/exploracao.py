"""Exploração do labirinto sem mapa: DFS com backtracking usando só sensores.

Python puro (não importa nada do ROS). O Explorador recebe de fora:
  - sentir(): devolve {(d_linha, d_coluna): 'b'|'f'|'t'} com as 8 células ao redor;
  - mover(direcao): devolve (sucesso, posicao_real, ...) após tentar andar uma célula;
  - inicio: posição inicial absoluta do robô (a única informação além dos sensores).

Na execução real quem fornece isso é o ros_io.RosIO; nos testes, um simulador falso.

Regras importantes:
  - O alvo é anotado no mapa quando o sensor o vê, mas o robô NUNCA entra nele:
    no simulador, pisar no alvo encerra o jogo e todo movimento seguinte é recusado.
    Isso não afeta a rota ótima: o que só se alcança atravessando o alvo fica
    "depois" do alvo e nunca faz parte do menor caminho até ele.
  - A exploração só termina quando toda a área alcançável (sem atravessar o alvo)
    foi visitada. Assim o grafo mapeado é igual ao real e o A* sobre ele dá a
    mesma rota ótima da Parte 1.
"""
from dataclasses import dataclass, field

from .planejamento import ALVO, DELTA_PARA_DIRECAO, DIRECOES, LIVRE, PAREDE, ROBO

VALORES_VALIDOS = {PAREDE, LIVRE, ALVO}


class ErroOdometria(RuntimeError):
    """O robô não está onde o explorador acha que ele está."""


class ErroSensor(RuntimeError):
    """As leituras do sensor continuam contradizendo o mapa já conhecido."""


@dataclass
class ResultadoExploracao:
    inicio: tuple
    mapa: dict = field(default_factory=dict)   # (linha, coluna) -> 'b' | 'f' | 't'
    alvo: tuple = None
    movimentos: int = 0
    releituras: int = 0

    @property
    def celulas_livres(self):
        return sum(1 for valor in self.mapa.values() if valor != PAREDE)


class Explorador:
    def __init__(self, sentir, mover, inicio, max_leituras=6, log=None):
        self._sentir_fn = sentir
        self._mover_fn = mover
        self.inicio = tuple(inicio)
        self.max_leituras = max_leituras
        self.log = log or (lambda msg: None)

    def explorar(self):
        resultado = ResultadoExploracao(inicio=self.inicio)
        mapa = resultado.mapa
        mapa[self.inicio] = LIVRE

        pos = self.inicio
        visitados = {self.inicio}
        sentidas = set()          # células onde o sensor já foi lido
        pilha = [self.inicio]     # caminho do início até a célula atual (para voltar)

        while True:
            if pos not in sentidas:
                self._sentir(pos, resultado)
                sentidas.add(pos)

            candidatos = [vizinho for vizinho in self._vizinhos4(pos)
                          if mapa.get(vizinho) == LIVRE and vizinho not in visitados]

            if candidatos:                 # avança para uma célula nova
                destino = candidatos[0]
                visitados.add(destino)
                pilha.append(destino)
            elif len(pilha) > 1:           # beco sem saída: volta um passo
                pilha.pop()
                destino = pilha[-1]
            else:                          # voltou ao início sem nada novo: fim
                break

            self._mover(pos, destino, resultado)
            pos = destino

        return resultado

    @staticmethod
    def _vizinhos4(pos):
        for d_linha, d_coluna in DIRECOES.values():
            yield (pos[0] + d_linha, pos[1] + d_coluna)

    def _sentir(self, pos, resultado):
        """Lê o sensor e anota os 8 vizinhos no mapa.

        Uma leitura só é aceita se (1) duas leituras seguidas forem iguais e
        (2) não contradisser nada já conhecido. Isso descarta uma mensagem
        atrasada da célula anterior, inclusive num corredor reto, onde ela pode
        não contradizer nada e passaria despercebida só com a checagem (2).
        """
        anterior = None
        for _ in range(self.max_leituras):
            leitura = self._normalizar(self._sentir_fn())
            conflitos = self._conflitos(pos, leitura, resultado.mapa)
            if leitura != anterior and anterior is not None:
                resultado.releituras += 1
                self.log(f'Leituras do sensor diferentes em {pos}; lendo de novo')
            if leitura == anterior and not conflitos:
                for (d_linha, d_coluna), valor in leitura.items():
                    celula = (pos[0] + d_linha, pos[1] + d_coluna)
                    resultado.mapa[celula] = valor
                    if valor == ALVO:
                        if resultado.alvo is None:
                            self.log(f'Alvo avistado em {celula}')
                        resultado.alvo = celula
                return
            if conflitos:
                self.log(f'Leitura do sensor contradiz o mapa em {pos}: {conflitos}')
            anterior = leitura
        raise ErroSensor(f'Sensor contradiz o mapa em {pos} após {self.max_leituras} leituras')

    @staticmethod
    def _normalizar(leitura):
        normalizada = {}
        for delta, valor in leitura.items():
            valor = LIVRE if valor == ROBO else valor
            if valor not in VALORES_VALIDOS:
                raise ErroSensor(f'Valor de sensor desconhecido: {valor!r}')
            normalizada[tuple(delta)] = valor
        return normalizada

    @staticmethod
    def _conflitos(pos, leitura, mapa):
        conflitos = []
        for (d_linha, d_coluna), valor in leitura.items():
            celula = (pos[0] + d_linha, pos[1] + d_coluna)
            if celula in mapa and mapa[celula] != valor:
                conflitos.append((celula, mapa[celula], valor))
        return conflitos

    def _mover(self, pos, destino, resultado):
        direcao = DELTA_PARA_DIRECAO[(destino[0] - pos[0], destino[1] - pos[1])]
        resposta = self._mover_fn(direcao)
        sucesso, pos_real = resposta[0], tuple(int(v) for v in resposta[1])
        resultado.movimentos += 1
        if not sucesso or pos_real != destino:
            raise ErroOdometria(
                f'Movimento {direcao!r} de {pos}: esperado {destino}, '
                f'simulador diz {pos_real} (sucesso={sucesso}). '
                'A posição inicial informada está certa?')


def mapa_para_grid(mapa, inicio=None, alvo=None):
    """Transforma o mapa explorado em matriz. Célula desconhecida vira parede."""
    linhas = max(linha for linha, _ in mapa) + 1
    colunas = max(coluna for _, coluna in mapa) + 1
    grid = [[PAREDE] * colunas for _ in range(linhas)]
    for (linha, coluna), valor in mapa.items():
        if linha >= 0 and coluna >= 0:
            grid[linha][coluna] = valor
    if inicio is not None:
        grid[inicio[0]][inicio[1]] = ROBO
    if alvo is not None:
        grid[alvo[0]][alvo[1]] = ALVO
    return grid


def comparar_com_real(mapa, grid_real):
    """Compara o mapa explorado com o mapa real (o gabarito do /get_map).

    Retorna (divergencias, nao_exploradas):
      - divergencias: células conhecidas cujo valor difere do real;
      - nao_exploradas: células livres do mapa real que a exploração não viu
        (só deveriam existir atrás do alvo).
    """
    def normalizar(valor):
        return LIVRE if valor == ROBO else valor

    divergencias = []
    for (linha, coluna), valor in sorted(mapa.items()):
        real = normalizar(grid_real[linha][coluna])
        if real != valor:
            divergencias.append(((linha, coluna), valor, real))

    nao_exploradas = [(i, j)
                      for i, linha in enumerate(grid_real)
                      for j, valor in enumerate(linha)
                      if valor != PAREDE and (i, j) not in mapa]
    return divergencias, nao_exploradas
