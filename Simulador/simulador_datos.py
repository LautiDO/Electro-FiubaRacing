import math
import random
import time
import os
from sys import argv

FREQ_ARDUINO = 0.02 #El tiempo de actualización del arduino, 20ms

class Simulador:
    def __init__(self):
        self.motor = {
            "TPS": 0,
            "RPM" : 0,
            "T_aire": round(random.uniform(15, 30), 2),
            "T_motor": 0,
            "Lambda": 0,
            "Velocidad": 0,
            "A_lineal": 0,
            "FG_mag" : 0,
            "FG_angle" : 0,
            "Longitud": 0,
            "Latitud": 0,
            "Pos_volante": 0,
            "F_suspensión": 0,
            "Pres_neumáticos": 0,
            "T_neumáticos": 0
        }

    def _t_motor(self):
        """La temperatura del motor tiene un piso de 60, y está más caliente cuanto mayor se la temperatura del aire y los RPM.
        Va desde los 75,6 grados hasta los 114."""
        self.motor["T_motor"] = round((60 + self.motor["T_aire"] + 0.004*self.motor["RPM"]), 2)
        return self.motor["T_motor"]

    def _t_aire(self):
        """La temperatura del aire varía +- 0.2 grados en cada instante de tiempo"""
        self.motor["T_aire"] += random.uniform(-0.2, 0.2)
        self.motor["T_aire"] = round(self.motor["T_aire"], 2)
        return self.motor["T_aire"]

    def _lambda(self):
        """El Lambda está influenciado únicamente por los RPM.
        Esto no es del todo realista pero debería dar valores medianamente
        cercanos."""
        if self.motor["RPM"] < 2000:
            lambda_base = 1.70
        elif self.motor["RPM"] < 4000:
            rpm_factor = (self.motor["RPM"] - 2000) / 2000
            lambda_base = 1.70 - (rpm_factor * 0.65) #va de 1.70 a 0.4
        else:
            lambda_base = 0.3

        ruido = round(random.uniform(-0.3, 0.3), 2)
        self.motor["Lambda"] = round(lambda_base + ruido,2)
        return self.motor["Lambda"]

    def _tps(self, t_actual):
        """En resumen, cada 10 segundos se repite el ciclo,
        inicialmente se mantiene el acelerador suelto (5%),
        se va subiendo gradualmente hasta 100%,
        se mantiene unos segundos y vuelve a bajar a 5%."""
        ciclo = 10 #cada 10s repite el ciclo
        fase = (t_actual % ciclo) / ciclo #Normalizado 0 a 1
        tps_idle = 0
        tps_max = 100

        if fase < 0.2:
            valor = tps_idle
        elif fase < 0.4:
            transicion = (fase - 0.2) / 0.2
            sine = math.sin(math.pi * transicion / 2)
            valor = tps_idle + sine * (tps_max - tps_idle)
        elif fase < 0.7:
            valor = tps_max
        elif fase < 0.9:
            transicion = (fase - 0.7) / 0.2
            sine = math.sin(math.pi * transicion / 2)
            valor = tps_max - (sine * 95)
        else:
            valor = tps_idle
        ruido = random.randint(-1, 1)
        self.motor["TPS"] = max(0, min(100, int(valor + ruido))) #Para que no se salga del rango 0-100%
        return self.motor["TPS"]

    def _rpm(self): #En un futuro podrias hacer ciclos mas largos, que vayan simulando los cambios de marcha, y al final vuelve a marcha 1
        """Dependen del TPS"""
        rpm_idle = 0
        rpm_max = 8400

        tps_normalizado = self.motor["TPS"] / 100
        factor_rpm = tps_normalizado ** 1.5
        rpm_base = rpm_idle + (rpm_max - rpm_idle) * factor_rpm

        ruido = random.randint(-50, 50)
        self.motor["RPM"] = int(max(rpm_idle, min(rpm_base + ruido, rpm_max))) #Para que no se salga del rango idle-max
        return self.motor["RPM"]

    def _velocidad(self):
        """Que la velocidad dependa del TPS está medio flojo de papeles...
        Básicamente, si el % de TPS x la v_max es mayor a la velocidad registrada en el instante de tiempo previo,
        se suben hasta 2 km/h.
        Si es menor, disminuye hasta 3 km/h."""
        v_min = 0
        v_max = 120
        velocidad_objetivo = (self.motor["TPS"] / 100) * v_max
        velocidad_actual = self.motor["Velocidad"]

        cambio = 0
        if velocidad_objetivo > velocidad_actual:
            cambio = min(2, velocidad_objetivo - velocidad_actual)
        elif velocidad_objetivo < velocidad_actual:
            cambio = max(-3, velocidad_objetivo - velocidad_actual)

        nueva_velocidad = velocidad_actual + cambio
        ruido = random.uniform(-1, 1)
        self.motor["Velocidad"] = round(max(v_min, min(v_max, nueva_velocidad + ruido)), 1)
        return self.motor["Velocidad"]

    def _a_lineal(self):
        return self.motor["A_lineal"]

    def _fg_mag(self):
        return self.motor["FG_mag"]

    def _fg_angle(self):
        return self.motor["FG_angle"]

    def _longitud(self):
        return self.motor["Longitud"]

    def _latitud(self):
        return self.motor["Latitud"]

    def _pos_volante(self):
        return self.motor["Pos_volante"]

    def _f_suspension(self):
        return self.motor["F_suspensión"]

    def _pres_neumaticos(self):
        return self.motor["Pres_neumáticos"]

    def _t_neumaticos(self):
        return self.motor["T_neumáticos"]

    def _generar_datos(self, t_actual):
        self._tps(t_actual)
        self._rpm()
        self._t_aire()
        self._t_motor()
        self._lambda()
        self._velocidad()
        self._a_lineal()
        self._fg_mag()
        self._fg_angle()
        self._longitud()
        self._latitud()
        self._pos_volante()
        self._f_suspension()
        self._pres_neumaticos()
        self._t_neumaticos()

    def generar_formato_csv(self, t_actual):
        """Esta función es utilizada únicamente por el archivo de puerto virtual"""
        self._generar_datos(t_actual)

        output = [f"{t_actual:.2f}s"] + [str(self.motor[i]) for i in self.motor]
        return ",".join(output) + "\n"

    def imprimir_datos_inf(self):
        t_actual = 0
        while True:
            self._generar_datos(t_actual)
            output = [f"t={t_actual:.2f}s"]
            for key, value in self.motor.items():
                output.append(f"{key}: {value}")
            print(" | ".join(output) + "\n")
            time.sleep(FREQ_ARDUINO)
            t_actual += FREQ_ARDUINO

    def generar_archivo(self, comando, tiempo, archivo):
        if not os.path.exists("pruebas"):
            os.makedirs("pruebas")

        if comando == "arduino":
            ext = "txt"
        else: ext = "csv"

        path = os.path.join("pruebas", f"{archivo}.{ext}")

        with open(path, 'w+') as f: #Ojo que esto sobreescribe un archivo si ya existe
            if comando == "csv":
                header = ["tiempo"] + list(self.motor.keys())
                f.write(",".join(header) + "\n")

            total_samples = int(tiempo / FREQ_ARDUINO)
            for i in range(total_samples):
                t_actual = i * FREQ_ARDUINO
                self._generar_datos(t_actual)

                if comando == "csv":
                    fila = [f"{t_actual:.2f}"] + [str(i) for i in self.motor.values()]
                    f.write(",".join(fila) + "\n")
                else:
                    f.write(f"\nTiempo: {t_actual:.2f}\n")
                    for key, value in self.motor.items():
                        f.write(f"{key}: {value}\n")
            print(f"Archivo generado en {path}")



if __name__ == "__main__":
    sim = Simulador()
    if len(argv) > 1:
        comando = argv[1]
        if comando in ["arduino", "csv"]:
            tiempo = int(argv[2]) if len(argv) > 2 else 300
            archivo = argv[3] if len(argv) > 3 else "datos_prueba"
            sim.generar_archivo(comando, tiempo, archivo)
    else:
        sim.imprimir_datos_inf()