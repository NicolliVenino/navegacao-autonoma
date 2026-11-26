"""Planejamento de caminho: A* em um grid 4-conectado com custo 1 por passo.

Este módulo é Python puro (não importa nada do ROS), então pode ser testado
sem o simulador. Coordenadas são sempre (linha, coluna), como no simulador:
'up' diminui a linha e 'left' diminui a coluna.
"""
import heapq
from itertools import count

PAREDE = 'b'
LIVRE = 'f'
ROBO = 'r'
ALVO = 't'

# Direção do simulador -> deslocamento (linha, coluna)
DIRECOES = {
    'up': (-1, 0),
    'down': (1, 0),
    'left': (0, -1),
    'right': (0, 1),
}
DELTA_PARA_DIRECAO = {delta: nome for nome, delta in DIRECOES.items()}


def manhattan(a, b):
    """Distância de Manhattan: nunca superestima em grid 4-conectado (admissível)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def livre(grid, celula):
    """True se a célula está dentro do grid e não é parede."""
    linha, coluna = celula
    return (0 <= linha < len(grid)
            and 0 <= coluna < len(grid[linha])
            and grid[linha][coluna] != PAREDE)


def vizinhos(grid, celula):
    """Vizinhos livres de uma célula (cima, baixo, esquerda, direita)."""
    for d_linha, d_coluna in DIRECOES.values():
        vizinho = (celula[0] + d_linha, celula[1] + d_coluna)
        if livre(grid, vizinho):
            yield vizinho


def astar(grid, inicio, alvo):
    """Menor caminho de `inicio` até `alvo` com A* (heurística de Manhattan).

    Retorna (caminho, expandidas):
      - caminho: lista de células [inicio, ..., alvo], ou None se não há caminho;
      - expandidas: quantas células foram tiradas de `abertos` e analisadas.
    """
    inicio, alvo = tuple(inicio), tuple(alvo)
    desempate = count()  # se dois f empatam, quem entrou primeiro sai primeiro

    g = {inicio: 0}                 # andei 0 passos até o início
    veio_de = {}                    # para lembrar o caminho
    fechados = set()                # células que já olhei
    # abertos: fila de prioridade ordenada por (f, h, ordem de chegada)
    abertos = [(manhattan(inicio, alvo), manhattan(inicio, alvo), next(desempate), inicio)]
    expandidas = 0

    while abertos:
        _, _, _, atual = heapq.heappop(abertos)
        if atual in fechados:
            continue  # entrada antiga: essa célula já foi melhorada e analisada
        expandidas += 1

        if atual == alvo:
            return reconstruir_caminho(veio_de, atual), expandidas

        fechados.add(atual)

        for vizinho in vizinhos(grid, atual):
            if vizinho in fechados:
                continue
            novo_g = g[atual] + 1
            if vizinho not in g or novo_g < g[vizinho]:
                g[vizinho] = novo_g             # achei um jeito melhor de chegar nele
                veio_de[vizinho] = atual        # cheguei nele vindo do atual
                h = manhattan(vizinho, alvo)
                heapq.heappush(abertos, (novo_g + h, h, next(desempate), vizinho))

    return None, expandidas


def reconstruir_caminho(veio_de, fim):
    """Volta pelo `veio_de` até o início e devolve o caminho na ordem certa."""
    caminho = [fim]
    while caminho[-1] in veio_de:
        caminho.append(veio_de[caminho[-1]])
    caminho.reverse()
    return caminho


def caminho_para_direcoes(caminho):
    """[(1,1), (2,1), (2,2)] -> ['down', 'right']."""
    return [DELTA_PARA_DIRECAO[(b[0] - a[0], b[1] - a[1])]
            for a, b in zip(caminho, caminho[1:])]


def encontrar(grid, valor):
    """Primeira célula do grid com o valor dado (ex.: 'r' ou 't'), ou None."""
    for i, linha in enumerate(grid):
        for j, celula in enumerate(linha):
            if celula == valor:
                return (i, j)
    return None


def grid_de_lista_achatada(achatado, linhas, colunas):
    """Remonta a matriz que o /get_map devolve 'achatada' numa lista só."""
    achatado = list(achatado)
    return [achatado[i * colunas:(i + 1) * colunas] for i in range(linhas)]


def grid_para_texto(grid, caminho=None):
    """Desenho ASCII do grid: '#' parede, ' ' livre, R robô, T alvo, '.' caminho."""
    no_caminho = set(caminho or [])
    simbolos = {PAREDE: '#', LIVRE: ' ', ROBO: 'R', ALVO: 'T'}
    linhas = []
    for i, linha in enumerate(grid):
        texto = ''
        for j, celula in enumerate(linha):
            simbolo = simbolos.get(celula, '?')
            if (i, j) in no_caminho and celula == LIVRE:
                simbolo = '.'
            texto += simbolo * 2  # duplica para a célula ficar "quadrada" no terminal
        linhas.append(texto)
    return '\n'.join(linhas)
