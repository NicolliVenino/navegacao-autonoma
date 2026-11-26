"""Simulador falso para testar o algoritmo sem ROS.

Reproduz as regras de culling_games/src/cg/cg/{Robot,Game}.py:
  - move: anda se a célula de destino não for parede ('b');
  - pisar no alvo vence o jogo e, depois disso, todo movimento é recusado;
  - sensores: conteúdo das 8 células ao redor do robô.
"""
import copy
import csv
import importlib.util
import random
from pathlib import Path

RAIZ_WORKSPACE = Path(__file__).resolve().parents[3]   # .../culling_games
PASTA_MAPAS = RAIZ_WORKSPACE / 'src' / 'cg' / 'maps'

OFFSETS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
DIRECOES = {'up': (-1, 0), 'down': (1, 0), 'left': (0, -1), 'right': (0, 1)}


def carregar_mapa(nome):
    with open(PASTA_MAPAS / nome) as arquivo:
        return [linha for linha in csv.reader(arquivo)]


def nomes_dos_mapas():
    return sorted(p.name for p in PASTA_MAPAS.glob('*.csv'))


def gerar_labirinto(semente, ciclos=0, tamanho=29):
    """Labirinto do generate_maze.py do simulador, com `ciclos` paredes extras abertas."""
    spec = importlib.util.spec_from_file_location('generate_maze',
                                                  RAIZ_WORKSPACE / 'generate_maze.py')
    modulo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modulo)
    random.seed(semente)
    grid = modulo.generate_maze(tamanho, tamanho)
    rng = random.Random(semente)
    candidatas = [(i, j) for i in range(1, len(grid) - 1) for j in range(1, len(grid) - 1)
                  if grid[i][j] == 'b' and (
                      (grid[i - 1][j] != 'b' and grid[i + 1][j] != 'b')
                      or (grid[i][j - 1] != 'b' and grid[i][j + 1] != 'b'))]
    for i, j in rng.sample(candidatas, min(ciclos, len(candidatas))):
        grid[i][j] = 'f'
    return grid


class FakeSim:
    def __init__(self, grid, sensor_atrasado=False):
        self.grid = copy.deepcopy(grid)
        self.pos = next((i, j) for i, linha in enumerate(self.grid)
                        for j, c in enumerate(linha) if c == 'r')
        self.alvo = next((i, j) for i, linha in enumerate(self.grid)
                         for j, c in enumerate(linha) if c == 't')
        self.venceu = False
        self.movimentos = 0
        # Simula a mensagem atrasada do tópico: a 1ª leitura após um movimento
        # ainda mostra os vizinhos da célula anterior.
        self.sensor_atrasado = sensor_atrasado
        self._leitura_velha = None

    def _ler(self, pos):
        i, j = pos
        return {(di, dj): self.grid[i + di][j + dj] for di, dj in OFFSETS}

    def sense(self):
        if self._leitura_velha is not None:
            leitura, self._leitura_velha = self._leitura_velha, None
            return leitura
        return self._ler(self.pos)

    def move(self, direcao):
        if self.venceu:
            return False, self.pos, self.pos
        anterior = self.pos
        di, dj = DIRECOES[direcao]
        destino = (self.pos[0] + di, self.pos[1] + dj)
        sucesso = self.grid[destino[0]][destino[1]] != 'b'
        if sucesso:
            self.grid[self.pos[0]][self.pos[1]] = 'f'
            self.grid[destino[0]][destino[1]] = 'r'
            self.pos = destino
            self.movimentos += 1
            if self.sensor_atrasado:
                self._leitura_velha = self._ler(anterior)
        if self.pos == self.alvo:
            self.venceu = True
        return sucesso, self.pos, self.alvo
