import math
import random
import time
import os
from sys import argv

"""Simulador de valores leidos por sensores.
Todos los valores son de ejemplo y no corresponden necesariamente
a valores posibles leídos por el auto de FSAE.
Véase, los rpm min y max están basados en el glorioso Honda Fit.
La idea es poder generar archivos formateados csv,
y archivos txt como los que recibiriamos del arduino, 
para poder empezar a trabajar con datos"""

FREQ_ARDUINO = 0.02

class Simulador:
    def __init__(self):
        self.motor = {
            "rpm" : 0,
            "t_motor": 0,
            "lambda": 10,
            "t_aire": round(random.uniform(15, 30), 2),
            "tps": 0
        }

    def _t_motor(self):
        """La temperatura del motor tiene un piso de 60, y está más caliente cuanto mayor se la temperatura del aire y los rpm.
        Va desde los 75,6 grados hasta los 114."""
        self.motor["t_motor"] = round((60 + self.motor["t_aire"] + 0.004*self.motor["rpm"]), 2)

    def _lambda(self):
        """El lambda está influenciado únicamente por los rpm.
        Esto no es del todo realista pero debería dar valores medianamente
        cercanos."""
        if self.motor["rpm"] < 2000:
            lambda_base = 14.5
        elif self.motor["rpm"] < 4000:
            rpm_factor = (self.motor["rpm"] - 2000) / 2000
            lambda_base = 14.5 - (rpm_factor * 1.5) #Va de 14.5 a 11.5
        else:
            lambda_base = 11

        ruido = round(random.uniform(-0.3, 0.3), 2)
        self.motor["lambda"] = round(lambda_base + ruido,2)
        return self.motor["lambda"]

    def _tps(self, t_actual):
        """En resumen, cada 10 segundos se repite el ciclo,
        inicialmente se mantiene el acelerador suelto (5%),
        se va subiendo gradualmente hasta 100%,
        se mantiene unos segundos y vuelve a bajar a 5%."""
        ciclo = 10 #cada 10s repite el ciclo
        fase = (t_actual % ciclo) / ciclo #Normalizado 0 a 1

        if fase < 0.2:
            valor = 5
        elif fase < 0.4:
            transicion = (fase - 0.2) / 0.2
            sine = math.sin(math.pi * transicion / 2)
            valor = 5 + sine * 95
        elif fase < 0.7:
            valor = 100
        elif fase < 0.9:
            transicion = (fase - 0.7) / 0.2
            sine = math.sin(math.pi * transicion / 2)
            valor = 100 - (sine * 95)
        else:
            valor = 5
        ruido = random.randint(-2, 2)
        self.motor["tps"] = max(0, min(100, int(valor + ruido))) #Para que no se salga del rango 0-100%
        return self.motor["tps"]

    def _rpm(self):
        """Dependen del tps"""
        rpm_idle = 120
        rpm_max = 6600

        tps_normalizado = self.motor["tps"] / 100
        factor_rpm = tps_normalizado ** 1.5
        rpm_base = rpm_idle + (rpm_max - rpm_idle) * factor_rpm

        ruido = random.randint(-50, 50)
        self.motor["rpm"] = int(max(rpm_idle, min(rpm_base + ruido, rpm_max))) #Para que no se salga del rango 120-6600
        return self.motor["rpm"]

    def _generar_datos(self, t_actual):
        self._tps(t_actual)
        self._rpm()
        self._t_motor()
        self._lambda()

    def generar_formato_arduino(self, t_actual):
        """Esta función es utilizada únicamente por el archivo de puerto virtual"""
        self._generar_datos(t_actual)

        output = f"""Tiempo: {t_actual:.2f}s
        RPM: {self.motor['rpm']}
        temperatura motor: {self.motor['t_motor']}
        AFR: {self.motor['lambda']}
        temperatura aire: {self.motor['t_aire']}
        TPS: {self.motor['tps']}%\n
        """
        return output

    def imprimir_datos_inf(self):
        t_actual = 0
        while True:
            self._generar_datos(t_actual)
            print(f"t={t_actual:.2f}s | RPM: {self.motor['rpm']:4d} | "
                  f"TPS: {self.motor['tps']:3d}% | T_motor: {self.motor['t_motor']:5.1f}°C | "
                  f"Lambda: {self.motor['lambda']:4.1f} | T_aire: {self.motor['t_aire']:4.1f}°C")
            time.sleep(FREQ_ARDUINO)
            t_actual += FREQ_ARDUINO

    def generar_archivo(self, comando, tiempo, archivo):
        """Crea los archivos csv o txt en una subcarpeta llamada pruebas,
        que es creada si no existe"""
        if not os.path.exists("pruebas"):
            os.makedirs("pruebas")

        if comando == "arduino":
            ext = "txt"
        else: ext = "csv"

        path = os.path.join("pruebas", f"{archivo}.{ext}")

        with open(path, 'w+') as f: #Ojo que esto sobreescribe un archivo si ya existe
            if comando == "csv":
                f.write("tiempo,rpm,tps,t_motor,lambda,t_aire\n") #Header

            total_samples = int(tiempo / FREQ_ARDUINO) #Se manda una actualizacion de los datos cada 20ms
            for i in range(total_samples):
                t_actual = i * FREQ_ARDUINO
                self._generar_datos(t_actual)

                if comando == "csv":
                    f.write(f"{t_actual:.2f},{self.motor['rpm']},{self.motor['tps']},"
                       f"{self.motor['t_motor']},{self.motor['lambda']},{self.motor['t_aire']}\n")
                else:
                    f.write(f"\nRPM: {self.motor["rpm"]}\n"
                            f"temperatura motor: {self.motor["t_motor"]}\n"
                            f"AFR: {self.motor["lambda"]}\n"
                            f"temperatura aire: {self.motor["t_aire"]}\n"
                            f"TPS: {self.motor["tps"]}\n"
                            f"Tiempo: {t_actual:.2f}\n")
            print(f"Archivo generado en {path}")




if __name__ == "__main__":
    """El primer parámetro tiene que ser el formato que se desea (csv o el formato definido en el .ino).
    El segundo parámetro es el tiempo que se quiere simular. No es necesario llenar este parámetro, por default son 300s.
    El tercero es el nombre del archivo al que se escriben estos datos (solo el nombre, sin la extension).
    Por default se van a escribir a un archivo llamado datos_prueba.
    ej: python3 simualador_datos.py csv 300 ejemplo
    simula 5 minutos (300s) de datos y los guarda con formato csv en ejemplo.csv
    (No es que tarda 5 minutos en correr).
    Si no se recibe ningún parámetro adicional,
    simplemente imprime datos por pantalla indefinidamente"""
    sim = Simulador()
    if len(argv) > 1:
        comando = argv[1]
        if comando in ["arduino", "csv"]:
            tiempo = int(argv[2]) if len(argv) > 2 else 300
            archivo = argv[3] if len(argv) > 3 else "datos_prueba"
            sim.generar_archivo(comando, tiempo, archivo)
    else:
        sim.imprimir_datos_inf()