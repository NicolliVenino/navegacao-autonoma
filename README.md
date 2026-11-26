# Navegação Autônoma

Nome: Nicolli Venino Santana

# 1. Tomadas de Decisão e Justificativas

## 1.1 Algoritmo de Busca do Menor Caminho (parte 1)

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

## 1.2 Algoritmo de Exploração do Mapa (parte 2)
Escolhi usar DFS com backtracking, pensando que DFS sempre se move para a célula vizinha e que cada passagem entre duas células é percorrida no máximo duas vezes (uma na ida e uma na volta).

### 1.2.1 Porque explorar o mapa inteiro

Pensei em parar a exploração assim que o sensor encontrasse o alvo, mas percebi que isso não garantiria a rota ótima. Os mapas do simulador têm ciclos, ou seja, mais de um caminho até o alvo, e o caminho mais curto poderia passar por uma região ainda não explorada. Explorando toda a área alcançável, o grafo do mapa gerado fica igual ao do mapa real, e assim o A* aplicado sobre ele obrigatoriamente encontra a mesma rota da Parte 1. 

### 1.2.2 Partes 
Aqui o robô começa sem saber nada do labirinto e precisa descobri-lo andando e "olhando" ao redor. Depois, precisa mostrar que o mapa que ele chegou configura-se como suficiente para chegar na rota ótima. Nesse sentido, tracei as ações passo a passo que pensei como necessárias para essa etapa:

### 1.2.2.1 Sensoriamento
O robô vai precisar ler o sensor e anotar os 8 vizinhos no mapa dele, criando um mapa de ocupação, conforme o exemplo abaixo:

Antes:

|  |  |  |   |  |
|---|---|---|---|---|          
| ? | ? | ? | ?| ? |        
| ? | R | ? | ?| ? |
| ? | ? | ? | ?| ? |    

Depois de sentir no ponto inicial:

|  |  |  |   |  |
|---|---|---|---|---|          
| # | # | # | ?| ? |        
| # | R | . | ?| ? |
| # | . | # | ?| ? |    

Onde:
|  |
|---|
|'?' = não visitado|
|'#' = ocupado|
|'.' = livre|

### 1.2.2.2 Decidir 
O robô olha para cima, baixo, direita e esqueda, verifica se existe um vizinho livre ou não visitado, e toma uma decisão da seguinte forma:

- Se existe, vai para essa célula e guarda numa pilha o caminho que fez.
- Se não existe, volta um passo, configurando uma lógica de backtracking. 

### 1.2.2.3 Mover e Conferir 
O robô manda o /move_command e confere se o robot_pos da a resposta e a célula esperada. 

### 1.2.2.4 Repetir o sensoriamento e mover e conferir
Quando a pilha volta para o início e não existe mais nada novo, toda a completude do labirinto alcançavel foi visitado. 

### 1.2.2.5 Montar o mapa
Gera o que pegou numa matriz 29x29 e salva em um arquivo mapa_gerado.csv.

### 1.2.2.6 Planejar
roda o mesmo A* da primeira parte, sob o mapa construído. 

### 1.2.2.7 Provar
Pensei em três comparações para fazer essa prova, conforme abaixo:

- A rota do mapa gerado tem as mesmas células da rota salva pela Parte 1. 
- O /get_map é consultado somente agora, depois da exploração e o mapa gerado não pode ter nenhuma célula diferente do real.
- com /reset o robô executa a rota do mapa gerado e chega ao alvo.  

### 1.2.3 Como pensei no algoritmo
Desenvolvi o pseudocódigo abaixo pensando em como ficaria as minhas decisões supracitadas no código:

```
FUNÇÃO EXPLORAR(inicio):

    mapa ← { inicio: livre }
    visitados ← { inicio }
    pilha ← [ inicio ] // caminho até a célula atual, para poder voltar
    atual ← inicio

    REPETE:
        SE ainda não li o sensor em atual:
            lê os 8 vizinhos e anota no mapa // se ver o alvo, guarda a posição

        candidatos ← vizinhos (cima, baixo, esquerda, direita) que estão
                     livres no mapa, ainda não visitados e não são o alvo

        SE existe candidato:
            destino ← primeiro candidato
            marca destino como visitado
            empilha destino // avancei
        SENÃO SE a pilha tem mais de uma célula:
            desempilha
            destino ← topo da pilha // backtracking: volto um passo
        SENÃO:
            PARA // voltei ao início e não há nada novo: mapa completo

        move até destino e confere a posição com o simulador
        atual ← destino

    RETORNA mapa, pos
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

# 3. Como rodar

## 3.1 Pré-requisitos

- Ubuntu 24.04 com **ROS 2 Jazzy** instalado (`/opt/ros/jazzy`)
- `colcon` e `pygame`:

```bash
sudo apt install python3-colcon-common-extensions python3-pygame
```

## 3.2 Compilando

O workspace ROS é a pasta `culling_games/`. Ela contém o simulador (pacotes `cg`, `cg_interfaces` e `cg_teleop`) e a minha solução (pacote `cg_solver`).

```bash
git clone <url-do-repositorio>
cd navegacao-autonoma/culling_games
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

> Em **todo terminal novo** é preciso rodar `source /opt/ros/jazzy/setup.bash` e `source install/setup.bash` dentro de `culling_games/`.

## 3.3 Abrindo o simulador (terminal 1)

```bash
ros2 run cg maze -- --map 3.csv
```

- `--map N.csv` escolhe um mapa de `src/cg/maps/` (`1.csv` a `10.csv` ou `test.csv`).
- Sem `--map`, o simulador sorteia um mapa.
- `--generate` cria um labirinto novo na hora.

Deixe esse terminal aberto: a janela do pygame mostra o robô (azul) e o alvo (vermelho).

## 3.4 Parte 1: navegação com mapa (terminal 2)

```bash
ros2 run cg_solver parte1 --map 3.csv
```

O programa:
1. reinicia o mapa (`/reset`);
2. pede o mapa completo (`/get_map`);
3. calcula a rota ótima com A*;
4. executa a rota no simulador passo a passo (`/move_command`).

A rota é salva em `rota_parte1.json`, para a Parte 2 comparar.

Saída esperada:
```
Mapa carregado: 3.csv
Mapa 29x29, robô em (1, 1), alvo em (14, 14)
A*: 192 passos, 304 células expandidas, 0.3 ms
Posição final (14, 14), alvo (14, 14): CHEGOU AO ALVO
Rota salva em rota_parte1.json
```

## 3.5 Parte 2: mapeamento do labirinto (terminal 2)

Rode na mesma pasta em que rodou a Parte 1, para que ela encontre o `rota_parte1.json`:

```bash
ros2 run cg_solver parte2 --map 3.csv
```

O programa:
1. reinicia o mapa;
2. explora o labirinto com DFS usando **só os sensores** (`/culling_games/robot_sensors`) e a posição inicial do robô;
3. salva o mapa construído em `mapa_gerado.csv`;
4. roda o mesmo A* sobre o mapa construído;
5. prova que ele reproduz a rota da Parte 1. O `/get_map` só é consultado **depois** da exploração, como gabarito;
6. reinicia o mapa e executa a nova rota até o alvo.

## 3.6 Opções

| Opção | Parte | O que faz |
|---|---|---|
| `--map N.csv` | 1 e 2 | Carrega esse mapa antes de começar. Sem ela, usa o mapa que está aberto. |
| `--delay 0.05` | 1 e 2 | Pausa (em segundos) após cada movimento, para dar tempo de ver o robô andando. |
| `--start L C` | 2 | Posição inicial do robô (padrão `1 1`). |
| `--saida arquivo.json` | 1 | Onde salvar a rota (padrão `rota_parte1.json`). |
| `--rota-parte1 arquivo.json` | 2 | Rota da Parte 1 usada na comparação. |
| `--saida-mapa arquivo.csv` | 2 | Onde salvar o mapa gerado (padrão `mapa_gerado.csv`). |

## 3.7 Rodando os testes

Pedi para a minha IA desenvolver um conjunto de testes para que eu conseguisse validar a qualidade do meu algoritmo, e pode ser rodado com as instruções abaixo: 

Os testes não precisam do simulador. Eles usam um simulador falso (`test/fake_sim.py`) que segue as mesmas regras do real.

```bash
cd culling_games
colcon test --packages-select cg_solver
colcon test-result --verbose
```

Ou direto com o pytest:

```bash
python3 -m pytest culling_games/src/cg_solver/test -v
```

São 60 testes. Eles conferem que:
- o A* acha o menor caminho em todos os mapas e em labirintos gerados com ciclos;
- a exploração nunca pisa no alvo;
- o mapa gerado reproduz exatamente a rota ótima;
- leituras atrasadas do sensor são descartadas.

# 4. Declaração de uso de IA
Conforme descrito nessa documentação, as tomadas de decisão no que tange ao algoritmo de busca, ao algoritmo de exploração e à arquitetura foram completamente minhas e escritas por mim. A sintaxe do código e o conjunto de testes de validação, assim como a seção de como rodar, foram criados com ajuda do Claude Code.