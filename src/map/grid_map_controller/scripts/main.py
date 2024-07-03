from aEstrela import Nodes, AlgoritmoAStar
from typing import List

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
        print("Caminho encontrado:", [(no.x, no.y) for no in caminho])
    else:
        print("Caminho não encontrado.")



if __name__ == '__main__':
    main()