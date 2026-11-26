"""Parte 2: mapeamento do labirinto e prova de que o mapa reproduz a rota da Parte 1.

Fluxo:
  1. /reset e exploração por DFS usando SÓ os sensores + a posição inicial
     (o /get_map não é usado para decidir nenhum movimento);
  2. monta o mapa gerado, salva em CSV e roda o mesmo A* da Parte 1 sobre ele;
  3. prova: compara com a rota da Parte 1 (JSON) e, como gabarito, com o
     /get_map consultado só DEPOIS da exploração;
  4. /reset e executa a rota obtida do mapa gerado até o alvo.

Uso:
    ros2 run cg_solver parte2 [--map 3.csv] [--start 1 1] [--delay 0.02]
"""
import argparse
import csv
import json
import os
import sys

import rclpy
from rclpy.utilities import remove_ros_args

from .exploracao import Explorador, comparar_com_real, mapa_para_grid
from .planejamento import ALVO, astar, encontrar, grid_para_texto
from .ros_io import RosIO


def ler_argumentos():
    parser = argparse.ArgumentParser(description='Parte 2: mapeamento + reprodução da rota')
    parser.add_argument('--map', default='',
                        help='mapa a carregar via /reset (ex.: 3.csv). Vazio = mapa atual')
    parser.add_argument('--start', type=int, nargs=2, default=[1, 1],
                        metavar=('LINHA', 'COLUNA'), help='posição inicial do robô')
    parser.add_argument('--delay', type=float, default=0.0,
                        help='pausa (s) após cada movimento, para visualizar')
    parser.add_argument('--rota-parte1', default='rota_parte1.json',
                        help='JSON gerado pela Parte 1, para comparação')
    parser.add_argument('--saida-mapa', default='mapa_gerado.csv',
                        help='arquivo onde o mapa gerado é salvo')
    return parser.parse_args(remove_ros_args(sys.argv)[1:])


def main():
    args = ler_argumentos()
    rclpy.init()
    io = RosIO(delay=args.delay)
    log = io.get_logger().info
    checagens = {}
    try:
        nome_mapa = io.reset(args.map)
        inicio = tuple(args.start)
        log(f'Mapa carregado: {nome_mapa}. Explorando a partir de {inicio} só com sensores...')

        # 1. Exploração ------------------------------------------------------
        explorador = Explorador(io.sense, io.move, inicio, log=log)
        resultado = explorador.explorar()
        log(f'Exploração concluída: {resultado.movimentos} movimentos, '
            f'{resultado.celulas_livres} células livres mapeadas, '
            f'{resultado.releituras} releituras de sensor')
        if resultado.alvo is None:
            raise RuntimeError('O alvo não foi encontrado durante a exploração')

        # 2. Mapa gerado + A* -------------------------------------------------
        grid_gerado = mapa_para_grid(resultado.mapa, inicio, resultado.alvo)
        with open(args.saida_mapa, 'w', newline='') as arquivo:
            csv.writer(arquivo).writerows(grid_gerado)
        log(f'Mapa gerado salvo em {args.saida_mapa}')

        caminho, expandidas = astar(grid_gerado, inicio, resultado.alvo)
        if caminho is None:
            raise RuntimeError('O mapa gerado não tem caminho até o alvo')
        passos = len(caminho) - 1
        log(f'A* no mapa gerado: {passos} passos, {expandidas} células expandidas')
        print(grid_para_texto(grid_gerado, caminho), flush=True)

        # 3a. Prova: comparação com a Parte 1 ----------------------------------
        if os.path.exists(args.rota_parte1):
            with open(args.rota_parte1) as arquivo:
                parte1 = json.load(arquivo)
            if parte1.get('mapa') != nome_mapa:
                log(f'{args.rota_parte1} é do mapa {parte1.get("mapa")}, não de {nome_mapa}: '
                    'comparação com a Parte 1 ignorada')
            else:
                caminho_p1 = [tuple(c) for c in parte1['caminho']]
                checagens['mesmo nº de passos da Parte 1'] = passos == parte1['passos']
                checagens['mesmas células da Parte 1'] = caminho == caminho_p1
                log(f'Parte 1: {parte1["passos"]} passos | Parte 2: {passos} passos')
        else:
            log(f'{args.rota_parte1} não encontrado: rode a Parte 1 antes para comparar')

        # 3b. Prova: gabarito do /get_map (consultado só DEPOIS da exploração) --
        grid_real = io.get_map()
        caminho_real, _ = astar(grid_real, inicio, encontrar(grid_real, ALVO))
        divergencias, nao_exploradas = comparar_com_real(resultado.mapa, grid_real)
        checagens['mapa sem divergências do real'] = not divergencias
        checagens['rota ótima igual à do mapa real'] = caminho == caminho_real
        log(f'Gabarito /get_map: rota ótima real = {len(caminho_real) - 1} passos; '
            f'{len(divergencias)} divergências; '
            f'{len(nao_exploradas)} células livres não exploradas (atrás do alvo)')
        for celula, mapeado, real in divergencias[:10]:
            log(f'  divergência em {celula}: mapeado {mapeado!r}, real {real!r}')

        # 4. Reexecução da rota do mapa gerado --------------------------------
        io.reset('')  # mesmo mapa, robô de volta ao início
        log('Executando no simulador a rota obtida do mapa gerado...')
        pos, alvo_sim = io.executar_caminho(caminho)
        checagens['robô chegou ao alvo'] = pos == alvo_sim == resultado.alvo

        log('===== Resultado da Parte 2 =====')
        for descricao, ok in checagens.items():
            log(f'  [{"OK" if ok else "FALHOU"}] {descricao}')
        codigo = 0 if all(checagens.values()) else 1
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
