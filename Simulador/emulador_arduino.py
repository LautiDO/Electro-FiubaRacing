import os.path
import serial
import time
import subprocess
from simulador_datos import Simulador, FREQ_ARDUINO

"""
Simualdor de datos en tiempo real.
La idea es usar esto para emular lo que se recibiría en un puerto usb en tiempo real con los datos de telemetría.
De esta manera no tenemos que esperar a tener un sistema de radio/4g para poder probar el programa con un stream de datos.
Usando el simulador de datos, se transmiten datos constantemente al puerto virtual.
Si queres saber lo que se está escribiendo en el puerto virtual ttyArduino,
podes hacer 'cat < /tmp/ttyReader' en una nueva consola de Linux, mientras está este programa en ejecución.
En este momento el emulador solo funciona para linux (lo probé con Ubuntu 24.04)
"""

BAUDRATE = 115200
VIRTUAL_PORT_ARDUINO = '/tmp/ttyArduino'
VIRTUAL_PORT_READER = '/tmp/ttyReader'

class VirtualArduino:
    def __init__(self):
        self.puerto = None
        self.simulador = Simulador()
        self.socat_process = None
        self.ser = None

    """
    Con socat (comando de linux) tiene que haber una fuente y un destino.
    Lo que ttyArduino escribe, ttyReader lee.
    """
    def crear_puertos_virtuales_linux(self): #Windows?
        try:
            self.socat_process = subprocess.Popen([
                'socat', '-d0',
                f'pty,raw,echo=0,link={VIRTUAL_PORT_ARDUINO}',
                f'pty,raw,echo=0,link={VIRTUAL_PORT_READER}'
            ])

            while not (os.path.exists(VIRTUAL_PORT_ARDUINO) and os.path.exists(VIRTUAL_PORT_READER)):
                time.sleep(0.1)
            time.sleep(0.5)
            self.puerto = VIRTUAL_PORT_ARDUINO
            print("Puertos virtuales creados:")
            print(f"   Arduino simulado: {VIRTUAL_PORT_ARDUINO}")
            print(f"   Lector: {VIRTUAL_PORT_READER}")

        except FileNotFoundError:
            print("socat no encontrado. Instalar con: sudo apt install socat")
            return False
        except Exception as e:
            print (f"Error creando puertos: {e}")
            return False
        return True

    def simular_arduino_serial(self):
        if not self.crear_puertos_virtuales_linux():
            return

        try:
            #Por lo que entiendo el baudrate, 115200, es el valor que se le pasa al inicio del archivo .ino,
            #y es la cantidad de unidades de señal por segundo
            self.ser = serial.Serial(self.puerto, BAUDRATE, timeout=1)
            if not self.ser.is_open:
                print("No se pudo abrir el puerto")
                return
            print(f"Simulador corriendo en {self.puerto}")

            t_actual = 0
            while True:
                data = self.simulador.generar_formato_csv(t_actual)
                if self.ser and self.ser.is_open:
                    self.ser.write(data.encode())
                    time.sleep(FREQ_ARDUINO)
                    t_actual += FREQ_ARDUINO


        except serial.SerialException as e:
            print(f"Error de puerto serie: {e}")
        except KeyboardInterrupt:
            print("Simulador detenido")

        finally:
            self.cleanup()

    def cleanup(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.socat_process:
            self.socat_process.terminate()
            try:
                self.socat_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.socat_process.kill()
        print("Puertos cerrados")

if __name__ == "__main__":
    virtual_arduino = VirtualArduino()
    virtual_arduino.simular_arduino_serial()