import math
from typing import List, Dict

class Nodes:
    def __init__(self, x: int, y: int):
        self.x: int = x
        self.y: int = y
        self.f: float = 0.0
        self.g: float = 0.0
        self.h: float = 0.0
        self.vizinhos: List['Nodes'] = []

    def heuristica(self, objetivo: 'Nodes') -> float:
        return math.sqrt((self.x - objetivo.x) ** 2 + (self.y - objetivo.y) ** 2)

class AlgoritmoAStar:
    @staticmethod
    def a_estrela(inicio: Nodes, objetivo: Nodes, matriz: 'List[List[int]]') -> List[Nodes]:
        abertos: List[Nodes] = []
        veio_de: Dict[Nodes, Nodes] = {}
        custo_g: Dict[Nodes, float] = {}

        abertos.append(inicio)
        custo_g[inicio] = 0.0
        inicio.h = inicio.heuristica(objetivo)
        inicio.f = inicio.h

        while abertos:
            atual: Nodes = min(abertos, key=lambda no: no.f)
            abertos.remove(atual)

            if atual == objetivo:
                return AlgoritmoAStar.reconstruir_caminho(veio_de, inicio, objetivo)

            for vizinho in atual.vizinhos:
                if vizinho and matriz[vizinho.y][vizinho.x] == 0:  # Ajuste aqui
                    custo_g_tentativo: float = custo_g.get(atual, math.inf) + 1

                    if custo_g_tentativo < custo_g.get(vizinho, math.inf):
                        veio_de[vizinho] = atual
                        custo_g[vizinho] = custo_g_tentativo
                        vizinho.h = vizinho.heuristica(objetivo)
                        vizinho.f = vizinho.h + custo_g_tentativo
                        if vizinho not in abertos:
                            abertos.append(vizinho)
        return []

    @staticmethod
    def reconstruir_caminho(veio_de: Dict[Nodes, Nodes], inicio: Nodes, objetivo: Nodes) -> List[Nodes]:
        caminho: List[Nodes] = []
        atual: Nodes = objetivo
        while atual != inicio:
            caminho.append(atual)
            atual = veio_de.get(atual, inicio) 
        caminho.append(inicio)
        caminho.reverse()
        return caminho
    

def conexoesMatriz(linhas: int, colunas: int, nos: 'List[List[Nodes | None]]'):
    for i in range(linhas):  # y
        for j in range(colunas):  # x
            if nos[i][j]:
                # Corrigindo as direções com base na nova definição de x e y
                if i > 0 and nos[i - 1][j]:  # Acima
                    nos[i][j].vizinhos.append(nos[i - 1][j])
                if i < linhas - 1 and nos[i + 1][j]:  # Abaixo
                    nos[i][j].vizinhos.append(nos[i + 1][j])
                if j > 0 and nos[i][j - 1]:  # Esquerda
                    nos[i][j].vizinhos.append(nos[i][j - 1])
                if j < colunas - 1 and nos[i][j + 1]:  # Direita
                    nos[i][j].vizinhos.append(nos[i][j + 1])


def atualizar_orientacao(orientacao_atual, dx, dy):
    direcoes = ['N', 'L', 'S', 'O']  # Norte, Leste, Sul, Oeste
    if dx > 0:  # Movendo-se para leste
        nova_orientacao = 'L'
    elif dx < 0:  # Movendo-se para oeste
        nova_orientacao = 'O'
    elif dy > 0:  # Movendo-se para norte
        nova_orientacao = 'N'
    elif dy < 0:  # Movendo-se para sul
        nova_orientacao = 'S'
    else:
        return orientacao_atual  # Sem movimento, mantém a orientação

    # Calcula a rotação necessária
    rotacao = direcoes.index(nova_orientacao) - direcoes.index(orientacao_atual)
    if rotacao == 1 or rotacao == -3:
        print("Virar para a direita")
    elif rotacao == -1 or rotacao == 3:
        print("Virar para a esquerda")
    elif abs(rotacao) == 2:
        print("Virar para trás")
    return nova_orientacao

def irParaLocal(lista: 'List[Nodes]'):
    if not lista:
        print("Lista vazia.")
        return

    quadranteAtual = (2, 4)  # Ponto de partida
    orientacao_atual = 'N'  # Inicia olhando para o norte
    for no in lista[1:]:  # Ignora o primeiro nó
        dx = no.x - quadranteAtual[0]
        dy = no.y - quadranteAtual[1]

        orientacao_atual = atualizar_orientacao(orientacao_atual, dx, dy)

        if dx != 0:
            print(f"Ir para frente {abs(dx)} quadrante(s)")
        elif dy != 0:
            print(f"Ir para frente {abs(dy)} quadrante(s)")

        quadranteAtual = (no.x, no.y)

def main():
    matriz = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1],
        [1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1],
        [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    ]

    matriz2 = [
        [0,0,1,1],
        [0,0,1,0],
        [0,0,0,0],
        [0,1,0,0],
    ]

    # Criando nós a partir da matriz
    linhas = len(matriz)
    colunas = len(matriz[0])
    nos = [[Nodes(j, i) if matriz[i][j] == 0 else None for j in range(colunas)] for i in range(linhas)]

    # Estabelecendo conexões entre os nós
    conexoesMatriz(linhas, colunas, nos)

    # Definindo nó de início e destino
    inicio_x =  11
    inicio_y =  8
    objetivo_x = 1
    objetivo_y = 1

    '''inicio_x =  0
    inicio_y =  0
    objetivo_x = 3
    objetivo_y = 3'''


    inicio = nos[inicio_y][inicio_x]
    objetivo = nos[objetivo_y][objetivo_x]


    caminho = AlgoritmoAStar.a_estrela(inicio, objetivo, matriz)

    if caminho:
        print([(no.y, no.x) for no in caminho])
        irParaLocal(caminho)
    else:
        print("Caminho não encontrado.")



if __name__ == '__main__':
    main()