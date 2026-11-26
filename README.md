# Navegação Autônoma

Nome: Nicolli Venino Santana

# 1. Tomadas de Decisão e Justificativas

## 1.1 Algoritmo de Busca do Menor Caminho

### 1.1.1 Porque escolhi A*
Minha primeira escolha foi sobre o algoritmo para encontrar o menor caminho nos mapas. De primeira análise, pensei em usar o A*, em razãoo da heurística que torna o processo mais otimizado (e também porque foi o algoritmo mais recente que estudei e já estava bem consolidado na minha memória). No entanto, comecei a refletir mais e entrei em um empasse, pensando que haja vista que o mapa configura-se como um grafo não ponderado, e, assim como Dijkstra, A* é voltados para grafos com pesos, comecei a questionar se faria sentido usá-lo, ou se fazia mais sentido aplicar um BFS. Nesse raciocínio, estudei mais de ambos os algoritmos e entendi que o ganho do A* não está em considerar os pesos, mas sim na heurística, que permite visitar menos células. Dessa forma, tomei a decisão de usar A* e adimitir que cada aresta no grafo que configura o mapa tem peso 1. 

### 1.1.2 Cálculo da Heurística 
Escolhi o calculo da distâcia de Manhattan em função dos movimentos pertidos do robô dentro do mapa (nas 4 direções, cima, baixo, direita e esquerda), pois ela não considera calculo na diagonal como Chebyshev e Octile, nem em demais ângulos como a Euclidina. Nesse sentido, se eu usasse por exemplo a distância Euclidiana no grafo de 4 direções, ainda acharia o melhor caminho, porém o algoritmo iria testar mais caminhos errados antes, subestimando, da mesma forma que, paralelamente, se eu usasse Manhattan em um grafo que permite diagonais, iria superestimar, ficando mais rápido, porém podendo ignorar o caminho de fato melhor.

### 1.1.3 Como pensei no algoritmo 
Para fins de planejamento e de consolidação ainda maior do algoritmo e da atividade para meu aprendizado pessoal, escrevi a mão o pseudocódigo abaixo:

```
FUNÇÃO A*(labirinto, inicio, alvo):

    g[inicio] ← 0 // andei 0 passos até o início
    abertos ← { inicio } // células que ainda vou olhar
    fechados ← { } // células que já olhei
    veio_de ← { } // para lembrar o caminho

    ENQUANTO abertos não estiver vazio:

        atual ← célula de abertos com o MENOR f (f = g + h)
        tira atual de abertos

        SE atual == alvo:
            RETORNA o caminho, voltando por veio_de até o início

        coloca atual em fechados

        PARA CADA vizinho de atual (cima, baixo, esquerda, direita):

            SE vizinho é parede OU já está em fechados:
                pula para o próximo vizinho

            novo_g ← g[atual] + 1

            SE nunca vi esse vizinho OU novo_g < g[vizinho]:
                g[vizinho] ← novo_g //achei um jeito melhor de chegar nele
                veio_de[vizinho] ← atual // cheguei nele vindo do atual
                coloca vizinho em abertos com f = novo_g + h(vizinho)

    RETORNA "não existe caminho"
```
# 2. Arquitetura

## 2.1 Partes

 Na primeira análise da atividade, fiquei pensativa sobre como eu iria separar a primeira parte do algoritmo de busca do melhor caminho da exploração e navegação do robô. Nesse mesmo sentido, também fiquei reflexiva sobre como integrar o simulador nisso tudo. Depois de alguns rascunhos, cheguei na seguintes partes da arquitetura:

 - planejamento.py: contempla o algoritmo de busca (A*).
 - exploração.py: contempla o algoritmo de exploração do mapa.  
 - ros.io.py: nó cliente do ROS2
 - simulador cg: nó cliente do culling_games.

 ## 2.2 Operações que o algoritmo usa

### 2.2.1 get_map()

```
get_map(), só na Parte 1

def get_map(self):
    resp = self._call(self.map_cli, GetMap.Request())
    rows, cols = resp.occupancy_grid_shape
    flat = resp.occupancy_grid_flattened
    return [flat[i*cols:(i+1)*cols] for i in range(rows)]   # matriz 29x29
```

### 2.2.2 move(direção)

```
def move(self, direction):
    req = MoveCmd.Request(); req.direction = direction
    resp = self._call(self.move_cli, req)
    self.last_move_time = time.monotonic()
    if self.delay: time.sleep(self.delay)      # opcional, para o vídeo
    return resp.success, tuple(resp.robot_pos), tuple(resp.target_pos)
```

### 2.2.3 sense()

```
def sense(self):
    limite = self.last_move_time + 0.03          # descarta os primeiros ~30 ms
    while self.last_sensor is None or self.last_sensor_time < limite:
        rclpy.spin_once(self, timeout_sec=0.01)  # deixa o callback rodar
    m = self.last_sensor
    return {(-1,0): m.up, (1,0): m.down, (0,-1): m.left, (0,1): m.right,
            (-1,-1): m.up_left, (-1,1): m.up_right,
```

### 2.2.4 reset()
```
def reset(self, map_name=''):
    req = Reset.Request(); req.is_random = False; req.map_name = map_name
    self._call(self.reset_cli, req)
    time.sleep(0.5)
```

