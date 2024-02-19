import math
import rospy
from std_msgs.msg import String
import serial



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


def conexoes(linhas, colunas, nos):
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
 
                    
def movimento_robo(coordenadas, serial_connection):
    #comando = 'WAWDWAWDWAWD'
    comando = ''
    for i in range(1, len(coordenadas)):
        x_anterior = coordenadas[i - 1].x
        y_anterior = coordenadas[i - 1].y


        x_atual = coordenadas[i].x
        y_atual = coordenadas[i].y

        if x_atual > x_anterior:
            comando += 'W'
        elif x_atual == x_anterior and y_atual > y_anterior:
            comando += 'AWD'
        

    if comando and serial_connection:
        print(f"Enviando comando {comando} para a porta serial")
        #enviar_para_serial(serial_connection, comando)
        
    if serial_connection:
        serial_connection.close()

def ler_string(msg, valores):
    rospy.loginfo("Caminho lido no nó assinante: %s", msg.data)
    
    valores_str = msg.data.split(',')
    
    try:      
        x = int(valores_str[0])
        y = int(valores_str[1])
    except ValueError:
        rospy.logwarn("Erro ao converter os valores para inteiros.")
        x = 0
        y = 0
    
    valores.extend([x, y])

def ler_caminho(valores):
    rospy.init_node('assinante_string', anonymous=True)
    rospy.Subscriber('minha_string', String, ler_string, callback_args=valores)
    rospy.spin() 


#######################################---MAIN---#######################################

# Definição manual da matriz

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
conexoes(linhas,colunas,nos)
valores = []
ler_caminho(valores)

# Definindo nó de início e destino
inicio_x =  valores[0]
inicio_y =  valores[1]
objetivo_x, objetivo_y = 3 ,3  

#inicio_x, inicio_y = 4, 2  
#objetivo_x, objetivo_y = 9, 10 

inicio = nos[inicio_x][inicio_y]
objetivo = nos[objetivo_x][objetivo_y]


caminho = AlgoritmoAStar.a_estrela(inicio, objetivo, matriz, nos)

if caminho:
    print("Caminho encontrado:", [(no.x, no.y) for no in caminho])
    #movimento_robo(caminho,serial_connection)
else:
    print("Caminho não encontrado.")

