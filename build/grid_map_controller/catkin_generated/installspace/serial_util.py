import serial

def inicializar_serial(porta_serial):
    try:
        ser = serial.Serial(porta_serial, baudrate=9600, timeout=1)
        return ser
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta serial: {e}")
        return None

def enviar_para_serial(ser, comando):
    try:
        if ser:
            ser.write(comando.encode()) 
        else:
            print("Erro: Porta serial não está aberta.")
    except serial.SerialException as e:
        print(f"Erro ao enviar comando pela porta serial: {e}")
