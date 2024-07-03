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
                if vizinho and matriz[vizinho.x][vizinho.y] == 0:
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
    for i in range(linhas):
        for j in range(colunas):
            if nos[i][j]:
                if i > 0 and nos[i - 1][j]:
                    nos[i][j].vizinhos.append(nos[i - 1][j])  # Acima
                if i < linhas - 1 and nos[i + 1][j]:
                    nos[i][j].vizinhos.append(nos[i + 1][j])  # Abaixo
                if j > 0 and nos[i][j - 1]:
                    nos[i][j].vizinhos.append(nos[i][j - 1])  # Esquerda
                if j < colunas - 1 and nos[i][j + 1]:
                    nos[i][j].vizinhos.append(nos[i][j + 1])  # Direita

def construirCaminho(caminho: List[Nodes]):
    anterior = None
    atual = None
    proximo = None

    for i in range(len(caminho)):
        if i == 0:
            atual = caminho[i]
            proximo = caminho[i+1]
        elif i == len(caminho) - 1:
            anterior = caminho[i-1]
            atual = caminho[i]
        else:
            anterior = caminho[i-1]
            atual = caminho[i]
            proximo = caminho[i+1]

        if anterior and proximo:
            if anterior.x == atual.x and atual.x == proximo.x:
                print("Frente")
            elif anterior.y == atual.y and atual.y == proximo.y:
                print("Direita")
            elif anterior.x < atual.x and atual.y < atual.y:
                print("Diagonal Direita")
            elif anterior.x < atual.x and atual.y > atual.y:
                print("Diagonal Esquerda")
            elif anterior.x > atual.x and atual.y < atual.y:
                print("Diagonal Direita")
            elif anterior.x > atual.x and atual.y > atual.y:
                print("Diagonal Esquerda")

def main():
    
    matriz2 = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
    ]

    matriz = [
        [0,0,1,1],
        [0,0,1,0],
        [0,0,0,0],
        [0,1,0,0],
    ]

    # Criando nós a partir da matriz
    linhas = len(matriz)
    colunas = len(matriz[0])
    nos = [[Nodes(i, j) if matriz[i][j] == 0 else None for j in range(colunas)] for i in range(linhas)]

    # Estabelecendo conexões entre os nós
    conexoesMatriz(linhas, colunas, nos)

    # Definindo nó de início e destino
    '''inicio_x =  4
    inicio_y =  3
    objetivo_x = 9
    objetivo_y = 10 '''

    inicio_x =  0
    inicio_y =  0
    objetivo_x = 3
    objetivo_y = 3


    inicio = nos[inicio_x][inicio_y]
    objetivo = nos[objetivo_x][objetivo_y]


    caminho = AlgoritmoAStar.a_estrela(inicio, objetivo, matriz)

    if caminho:
        #print("Caminho encontrado:", [(no.x, no.y) for no in caminho])
        construirCaminho(caminho)
    else:
        print("Caminho não encontrado.")



if __name__ == '__main__':
    main()