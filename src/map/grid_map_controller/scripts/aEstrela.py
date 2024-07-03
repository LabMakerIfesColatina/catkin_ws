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