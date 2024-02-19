import rospy
from std_msgs.msg import String

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
    #rospy.spin() 
