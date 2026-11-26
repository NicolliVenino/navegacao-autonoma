import pytest
from cg_solver.exploracao import (ErroOdometria, Explorador, comparar_com_real,
                                  mapa_para_grid)
from cg_solver.planejamento import astar, encontrar
from fake_sim import FakeSim, carregar_mapa, gerar_labirinto, nomes_dos_mapas


def explorar_e_provar(grid_real, sensor_atrasado=False):
    """Roda a Parte 2 inteira contra o simulador falso e confere a prova."""
    sim = FakeSim(grid_real, sensor_atrasado=sensor_atrasado)
    inicio = sim.pos
    resultado = Explorador(sim.sense, sim.move, inicio).explorar()

    # Nunca pisou no alvo (senão o jogo acabaria) e terminou de volta no início
    assert not sim.venceu
    assert sim.pos == inicio
    # Achou o alvo só pelos sensores
    assert resultado.alvo == encontrar(grid_real, 't')

    # O mapa gerado não contradiz o real
    divergencias, _ = comparar_com_real(resultado.mapa, grid_real)
    assert divergencias == []

    # A* no mapa gerado == A* no mapa real (mesmas células, não só o mesmo tamanho)
    grid_gerado = mapa_para_grid(resultado.mapa, inicio, resultado.alvo)
    caminho_gerado, _ = astar(grid_gerado, inicio, resultado.alvo)
    caminho_real, _ = astar(grid_real, inicio, encontrar(grid_real, 't'))
    assert caminho_gerado == caminho_real

    # Reexecutar a rota do mapa gerado leva o robô ao alvo
    sim_novo = FakeSim(grid_real)
    for a, b in zip(caminho_gerado, caminho_gerado[1:]):
        direcao = {(-1, 0): 'up', (1, 0): 'down',
                   (0, -1): 'left', (0, 1): 'right'}[(b[0] - a[0], b[1] - a[1])]
        assert sim_novo.move(direcao)[0]
    assert sim_novo.venceu
    return resultado


@pytest.mark.parametrize('nome', nomes_dos_mapas())
def test_exploracao_nos_mapas_do_simulador(nome):
    resultado = explorar_e_provar(carregar_mapa(nome))
    # DFS: cada corredor no máximo ida e volta
    assert resultado.movimentos <= 2 * resultado.celulas_livres


@pytest.mark.parametrize('semente', range(15))
def test_exploracao_em_labirintos_gerados_com_ciclos(semente):
    explorar_e_provar(gerar_labirinto(semente, ciclos=semente * 3))


@pytest.mark.parametrize('nome', ['3.csv', 'test.csv'])
def test_exploracao_resiste_a_leitura_atrasada_do_sensor(nome):
    resultado = explorar_e_provar(carregar_mapa(nome), sensor_atrasado=True)
    assert resultado.releituras > 0


def test_alvo_bloqueando_corredor_nao_e_atravessado():
    # A única passagem para a direita é pelo alvo: o robô não pode entrar nele.
    grid = [list('bbbbbbb'),
            list('brfftfb'),
            list('bbbbbbb')]
    sim = FakeSim(grid)
    resultado = Explorador(sim.sense, sim.move, sim.pos).explorar()
    assert not sim.venceu
    assert resultado.alvo == (1, 4)
    assert (1, 5) not in resultado.mapa or resultado.mapa[(1, 5)] == 'f'


def test_posicao_inicial_errada_e_detectada():
    sim = FakeSim(carregar_mapa('1.csv'))
    with pytest.raises(ErroOdometria):
        Explorador(sim.sense, sim.move, (2, 2)).explorar()
