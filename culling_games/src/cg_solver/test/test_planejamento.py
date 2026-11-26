from collections import deque

import pytest
from cg_solver.planejamento import (astar, caminho_para_direcoes, encontrar,
                                    grid_de_lista_achatada, livre, manhattan)
from fake_sim import carregar_mapa, gerar_labirinto, nomes_dos_mapas

# Menor caminho de cada mapa do simulador (calculado por BFS)
ESPERADO = {'1.csv': 102, '2.csv': 102, '3.csv': 192, '4.csv': 114, '5.csv': 204,
            '6.csv': 110, '7.csv': 96, '8.csv': 164, '9.csv': 190, '10.csv': 48,
            'test.csv': 25}


def bfs_distancia(grid, inicio, alvo):
    """BFS independente, usada como referência para o A*."""
    dist = {inicio: 0}
    fila = deque([inicio])
    while fila:
        atual = fila.popleft()
        if atual == alvo:
            return dist[atual]
        for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            v = (atual[0] + di, atual[1] + dj)
            if livre(grid, v) and v not in dist:
                dist[v] = dist[atual] + 1
                fila.append(v)
    return None


def caminho_valido(grid, caminho, inicio, alvo):
    assert caminho[0] == inicio and caminho[-1] == alvo
    for a, b in zip(caminho, caminho[1:]):
        assert manhattan(a, b) == 1, f'salto entre {a} e {b}'
    for celula in caminho:
        assert livre(grid, celula), f'{celula} é parede'
    assert len(set(caminho)) == len(caminho), 'caminho repete célula'


@pytest.mark.parametrize('nome', nomes_dos_mapas())
def test_astar_nos_mapas_do_simulador(nome):
    grid = carregar_mapa(nome)
    inicio, alvo = encontrar(grid, 'r'), encontrar(grid, 't')
    caminho, expandidas = astar(grid, inicio, alvo)
    caminho_valido(grid, caminho, inicio, alvo)
    assert len(caminho) - 1 == bfs_distancia(grid, inicio, alvo)
    if nome in ESPERADO:
        assert len(caminho) - 1 == ESPERADO[nome]
    assert expandidas > 0


@pytest.mark.parametrize('semente', range(15))
def test_astar_otimo_em_labirintos_gerados_com_ciclos(semente):
    grid = gerar_labirinto(semente, ciclos=semente * 3)
    inicio, alvo = encontrar(grid, 'r'), encontrar(grid, 't')
    caminho, _ = astar(grid, inicio, alvo)
    caminho_valido(grid, caminho, inicio, alvo)
    assert len(caminho) - 1 == bfs_distancia(grid, inicio, alvo)


def test_sem_caminho_retorna_none():
    grid = [list('bbbbb'),
            list('brbtb'),
            list('bbbbb')]
    caminho, _ = astar(grid, (1, 1), (1, 3))
    assert caminho is None


def test_inicio_igual_ao_alvo():
    grid = [list('bbb'), list('brb'), list('bbb')]
    caminho, _ = astar(grid, (1, 1), (1, 1))
    assert caminho == [(1, 1)]


def test_caminho_para_direcoes():
    caminho = [(1, 1), (2, 1), (2, 2), (1, 2), (1, 1)]
    assert caminho_para_direcoes(caminho) == ['down', 'right', 'up', 'left']


def test_grid_de_lista_achatada():
    assert grid_de_lista_achatada(list('abcdef'), 2, 3) == [list('abc'), list('def')]
