import os.path
import platform
import serial
import time
import subprocess
from simulador_datos import Simulador, FREQ_ARDUINO

BAUDRATE = 115200
#Se pueden modificar los puertos si se quieren crear en otro lado
VIRTUAL_PORT_ARDUINO_LINUX = '/tmp/ttyArduino'
VIRTUAL_PORT_READER_LINUX = '/tmp/ttyReader'
VIRTUAL_PORT_ARDUINO_WINDOWS = "COM3"
VIRTUAL_PORT_READER_WINDOWS = "COM4"

class VirtualArduino:
    def __init__(self):
        self.puerto = None
        self.simulador = Simulador()
        self.socat_process = None
        self.ser = None
        self.sistema = platform.system().lower()


    def crear_puertos_linux_macos(self):
        """
        Con socat (comando de linux) tiene que haber una fuente y un destino.
        Lo que ttyArduino escribe, ttyReader lee.
        """
        try:
            self.socat_process = subprocess.Popen([
                'socat', '-d0',
                f'pty,raw,echo=0,link={VIRTUAL_PORT_ARDUINO_LINUX}',
                f'pty,raw,echo=0,link={VIRTUAL_PORT_READER_LINUX}'
            ])

            while not (os.path.exists(VIRTUAL_PORT_ARDUINO_LINUX) and os.path.exists(VIRTUAL_PORT_READER_LINUX)):
                time.sleep(0.1)
            time.sleep(0.5)
            self.puerto = VIRTUAL_PORT_ARDUINO_LINUX
            print("Puertos virtuales creados:")
            print(f"   Arduino simulado: {VIRTUAL_PORT_ARDUINO_LINUX}")
            print(f"   Lector: {VIRTUAL_PORT_READER_LINUX}")

        except FileNotFoundError:
            print("socat no encontrado. Instalar con: sudo apt install socat")
            return False
        except Exception as e:
            print (f"Error creando puertos: {e}")
            return False
        return True
    
    def crear_puertos_windows(self):
        """Lamentablemente no hay una forma tan fácil de hacer esto como en Linux.
        La única forma que encontré es con un programa llamado com0com.
        Asegurarse de cambiar los puertos arduino y reader de windows a los correspondientes en esa aplicación."""
        try:
            test_ard = serial.Serial(VIRTUAL_PORT_ARDUINO_WINDOWS, BAUDRATE, timeout=0.1)
            test_ard.close()
            test_read = serial.Serial(VIRTUAL_PORT_READER_WINDOWS, BAUDRATE, timeout=0.1)
            test_read.close()
            self.puerto = VIRTUAL_PORT_ARDUINO_WINDOWS
            print("Puertos virtuales creados:")
            print(f"   Arduino simulado: {VIRTUAL_PORT_ARDUINO_WINDOWS}")
            print(f"   Lector: {VIRTUAL_PORT_READER_WINDOWS}")
            return True
        except Exception as e:
            print (f"Error creando puertos: {e}")
            return False

    def simular_arduino_serial(self):
        if self.sistema == "linux" or self.sistema == "darwin":
            if not self.crear_puertos_linux_macos():
                return
        elif self.sistema == "windows":
            if not self.crear_puertos_windows():
                return
        else:
            print(f"Sistema operativo {self.sistema} incompatible")
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