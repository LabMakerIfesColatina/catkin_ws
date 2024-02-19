import math

class Nodes:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = 0
        self.g = 0
        self.h = 0
        self.vizinhos = []

    def heuristica(self, objetivo):
        return math.sqrt((self.x - objetivo.x) ** 2 + (self.y - objetivo.y) ** 2)

class AlgoritmoAStar:
    @staticmethod
    def a_estrela(inicio, objetivo, matriz, nos):
        abertos = []
        veio_de = {}
        custo_g = {}

        abertos.append(inicio)
        custo_g[inicio] = 0.0
        inicio.h = inicio.heuristica(objetivo)
        inicio.f = inicio.h

        while abertos:
            atual = min(abertos, key=lambda no: no.f)
            abertos.remove(atual)

            if atual == objetivo:
                return AlgoritmoAStar.reconstruir_caminho(veio_de, inicio, objetivo)

            for vizinho in atual.vizinhos:
                if vizinho and matriz[vizinho.x][vizinho.y] == 0:
                    custo_g_tentativo = custo_g.get(atual, math.inf) + 1

                    if custo_g_tentativo < custo_g.get(vizinho, math.inf):
                        veio_de[vizinho] = atual
                        custo_g[vizinho] = custo_g_tentativo
                        vizinho.h = vizinho.heuristica(objetivo)
                        vizinho.f = vizinho.h + custo_g_tentativo
                        abertos.append(vizinho)

        return []

    @staticmethod
    def reconstruir_caminho(veio_de, inicio, objetivo):
        caminho = []
        atual = objetivo
        while atual != inicio:
            caminho.append(atual)
            atual = veio_de[atual]
        caminho.append(inicio)
        caminho.reverse()
        return caminho
