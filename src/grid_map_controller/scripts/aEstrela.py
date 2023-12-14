import math
import serial
import rospy
from std_msgs.msg import String  


class Node:
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


def publicar_caminho(caminho):
    rospy.init_node('publicador_caminho', anonymous=True)
    pub = rospy.Publisher('caminho_encontrado', String, queue_size=10)
    rate = rospy.Rate(1)  # Defina a frequência de publicação (1 Hz neste exemplo)

    while not rospy.is_shutdown():
        coordenadas = ""
        for no in caminho:
            coordenadas += f"({no.x}, {no.y}) "

        rospy.loginfo(coordenadas)
        pub.publish(coordenadas)
        rate.sleep()

def movimento_robo(coordenadas, serial_connection):
    
    for i in range(1, len(coordenadas)):
        x_anterior, y_anterior = coordenadas[i - 1].x, coordenadas[i - 1].y
        x_atual, y_atual = coordenadas[i].x, coordenadas[i].y

        comando = ''

        if y_atual > y_anterior:
            comando += 'W'
        elif y_atual < y_anterior:
            comando += 'S'

        if x_atual > x_anterior:
            comando += 'D'
        elif x_atual < x_anterior:
            comando += 'A'

        # Envia comando para porta serial (substitua isso pela lógica real de enviar para a porta serial)
        if comando and serial_connection:
            print(f"Enviando comando {comando} para a porta serial")
            enviar_para_serial(serial_connection, comando)
    
    if serial_connection:
        serial_connection.close()



def inicializar_serial(porta_serial):
    try:
        ser = serial.Serial(porta_serial, baudrate=9600, timeout=1)
        return ser
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta serial: {e}")
        return None

# Envia comando para a porta serial
def enviar_para_serial(ser, comando):
    try:
        if ser:
            ser.write(comando.encode()) 
            print(f"Enviando comando '{comando}' para a porta serial")
        else:
            print("Erro: Porta serial não está aberta.")
    except serial.SerialException as e:
        print(f"Erro ao enviar comando pela porta serial: {e}")


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

serial_port = '/dev/ttyUSB0'  
serial_connection = inicializar_serial(serial_port)

# Criando nós a partir da matriz
linhas = len(matriz)
colunas = len(matriz[0])
nos = [[Node(i, j) if matriz[i][j] == 0 else None for j in range(colunas)] for i in range(linhas)]

# Estabelecendo conexões entre os nós
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

# Definindo nó de início e destino
inicio_x, inicio_y = 0, 0 #4, 2  # Coordenadas de início
objetivo_x, objetivo_y = 3 ,3 #9, 10  # Coordenadas de chegada

inicio = nos[inicio_x][inicio_y]
objetivo = nos[objetivo_x][objetivo_y]

# Encontrando o caminho usando A*
caminho = AlgoritmoAStar.a_estrela(inicio, objetivo, matriz, nos)

# Exibindo o caminho encontrado
if caminho:
    print("Caminho encontrado:", [(no.x, no.y) for no in caminho])
    movimento_robo(caminho,serial_connection)
else:
    print("Caminho não encontrado.")