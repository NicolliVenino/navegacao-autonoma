"""Parte 1: navegação com mapa.

Fluxo: /reset -> /get_map -> A* -> direções -> /move_command passo a passo.
Salva a rota em JSON para a Parte 2 comparar.

Uso:
    ros2 run cg_solver parte1 [--map 3.csv] [--delay 0.05] [--saida rota_parte1.json]
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args

from .planejamento import (ALVO, ROBO, astar, caminho_para_direcoes, encontrar,
                           grid_para_texto)
from .ros_io import RosIO


def ler_argumentos():
    parser = argparse.ArgumentParser(description='Parte 1: rota ótima com A* usando o mapa')
    parser.add_argument('--map', default='',
                        help='mapa a carregar via /reset (ex.: 3.csv). Vazio = mapa atual')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='pausa (s) após cada movimento, para visualizar')
    parser.add_argument('--saida', default='rota_parte1.json',
                        help='arquivo onde a rota é salva')
    return parser.parse_args(remove_ros_args(sys.argv)[1:])


def main():
    args = ler_argumentos()
    rclpy.init()
    io = RosIO(delay=args.delay)
    log = io.get_logger().info
    codigo = 0
    try:
        nome_mapa = io.reset(args.map)
        log(f'Mapa carregado: {nome_mapa}')

        grid = io.get_map()
        inicio, alvo = encontrar(grid, ROBO), encontrar(grid, ALVO)
        log(f'Mapa {len(grid)}x{len(grid[0])}, robô em {inicio}, alvo em {alvo}')

        t0 = time.perf_counter()
        caminho, expandidas = astar(grid, inicio, alvo)
        tempo_ms = (time.perf_counter() - t0) * 1000
        if caminho is None:
            raise RuntimeError('Não existe caminho até o alvo')
        direcoes = caminho_para_direcoes(caminho)
        log(f'A*: {len(direcoes)} passos, {expandidas} células expandidas, {tempo_ms:.1f} ms')
        print(grid_para_texto(grid, caminho), flush=True)

        log('Executando a rota no simulador...')
        pos, alvo_sim = io.executar_caminho(caminho)
        chegou = pos == alvo_sim == alvo
        log(f'Posição final {pos}, alvo {alvo_sim}: '
            f'{"CHEGOU AO ALVO" if chegou else "NÃO chegou ao alvo"}')
        codigo = 0 if chegou else 1

        with open(args.saida, 'w') as arquivo:
            json.dump({
                'mapa': nome_mapa,
                'inicio': list(inicio),
                'alvo': list(alvo),
                'passos': len(direcoes),
                'expandidas': expandidas,
                'caminho': [list(c) for c in caminho],
                'direcoes': direcoes,
            }, arquivo, indent=2)
        log(f'Rota salva em {args.saida}')
    except Exception as erro:  # mostra o erro de forma legível e sai com código 1
        io.get_logger().error(str(erro))
        codigo = 1
    finally:
        io.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(codigo)


if __name__ == '__main__':
    main()
