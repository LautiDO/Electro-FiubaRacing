import math
import random
import time
import os
from sys import argv

FREQ_ARDUINO = 0.02 #El tiempo de actualización del arduino, 20ms

class Simulador:
    def __init__(self):
        self.sensores = {
            "TPS": 0,
            "RPM" : 0,
            "T_aire": round(random.uniform(15, 30), 2),
            "T_motor": 0,
            "Lambda": 0,
            "Velocidad": 0,
            "A_lineal": 0,
            "FG_mag" : 0,
            "FG_angle" : 0,
            "Longitud": -58.45991018745831, #coordenadas del Autódromo Oscar y Juan Gálvez
            "Latitud": -34.69564672421455,
            "Pos_volante": 0,
            "F_suspensión": 0,
            "Pres_neumáticos": 0,
            "T_neumáticos": 0
        }
        self.centro_lat = -34.69564672421455
        self.centro_long = -58.45991018745831
        self.radio_pista = 0.008 #aprox 900 metros de radio, es un ejemplo
        self.angulo_actual = 0


    def _t_motor(self):
        """La temperatura del motor tiene un piso de 60, y está más caliente cuanto mayor se la temperatura del aire y los RPM.
        Va desde los 75,6 grados hasta los 114."""
        self.sensores["T_motor"] = round((60 + self.sensores["T_aire"] + 0.004*self.sensores["RPM"]), 2)
        return self.sensores["T_motor"]

    def _t_aire(self):
        """La temperatura del aire varía +- 0.2 grados en cada instante de tiempo"""
        self.sensores["T_aire"] += random.uniform(-0.2, 0.2)
        self.sensores["T_aire"] = round(self.sensores["T_aire"], 2)
        return self.sensores["T_aire"]

    def _lambda(self):
        """El Lambda está influenciado únicamente por los RPM.
        Si bien tiene que ver, esto no es del todo realista pero debería dar valores medianamente
        cercanos."""
        if self.sensores["RPM"] < 2000:
            lambda_base = 1.70
        elif self.sensores["RPM"] < 4000:
            rpm_factor = (self.sensores["RPM"] - 2000) / 2000
            lambda_base = 1.70 - (rpm_factor * 0.65) #va de 1.70 a 0.4
        else:
            lambda_base = 0.3

        ruido = round(random.uniform(-0.3, 0.3), 2)
        self.sensores["Lambda"] = round(lambda_base + ruido,2)
        return self.sensores["Lambda"]

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
        self.sensores["TPS"] = int(valor)
        return self.sensores["TPS"]

    def _rpm(self): #En un futuro podrias hacer ciclos mas largos, que vayan simulando los cambios de marchas, y al final vuelve a marcha 1
        """Dependen del TPS"""
        rpm_idle = 0
        rpm_max = 8400

        tps_normalizado = self.sensores["TPS"] / 100
        factor_rpm = tps_normalizado ** 1.5
        rpm_base = rpm_idle + (rpm_max - rpm_idle) * factor_rpm

        ruido = random.randint(-50, 50)
        self.sensores["RPM"] = int(max(rpm_idle, min(rpm_base + ruido, rpm_max))) #Para que no se salga del rango idle-max
        return self.sensores["RPM"]

    def _velocidad(self):
        """Que la velocidad dependa del TPS está medio flojo de papeles...
        Básicamente, si el % de TPS x la v_max es mayor a la velocidad registrada en el instante de tiempo previo,
        se sube hasta 1 km/h.
        Si es menor, disminuye hasta 1 km/h."""
        v_min = 0
        v_max = 120
        velocidad_objetivo = (self.sensores["TPS"] / 100) * v_max
        velocidad_actual = self.sensores["Velocidad"]

        cambio = 0
        if velocidad_objetivo > velocidad_actual:
            cambio = min(1, velocidad_objetivo - velocidad_actual)
        elif velocidad_objetivo < velocidad_actual:
            cambio = max(-1, velocidad_objetivo - velocidad_actual)

        nueva_velocidad = velocidad_actual + cambio
        ruido = random.uniform(-0.2, 0.2)
        self.sensores["Velocidad"] = round(max(v_min, min(v_max, nueva_velocidad + ruido)), 1)
        return self.sensores["Velocidad"]

    def _a_lineal(self):
        """Depende del cambio de velocidades entre instantes."""
        #NO CONDICE CON LOS VALORES BASE 0-3 QUE ME PASARON...
        velocidad_anterior = getattr(self, '_velocidad_anterior', self.sensores["Velocidad"])
        velocidad_actual = self.sensores["Velocidad"]

        delta_v_kmh = velocidad_actual - velocidad_anterior
        delta_v_ms = delta_v_kmh * (1000 / 3600)
        aceleracion = delta_v_ms / FREQ_ARDUINO
        self._velocidad_anterior = velocidad_actual

        self.sensores["A_lineal"] = round(aceleracion, 2)
        return self.sensores["A_lineal"]

    def _calcular_aceleracion_lateral(self):
        if self.sensores["Velocidad"] == 0 or self.sensores["Pos_volante"] == 0:
            return 0
        velocidad_ms = self.sensores["Velocidad"] * (1000 / 3600)
        angulo_giro = (self.sensores["Pos_volante"] - 90) * 0.5 #Normalizarlo a -90,90 y reducir a un angulo mas realista
        angulo_volante_rad = math.radians(abs(angulo_giro))
        #Lo que viene ahora sale de una fórmula llamada ángulo de Ackermann
        wheelbase = 1 #distancia entre ejes delantero y trasero en metros
        if abs(angulo_volante_rad) > 0.01:
            radio_giro = wheelbase / math.tan(abs(angulo_volante_rad))
            a_lateral = (velocidad_ms**2) / radio_giro
            return a_lateral * (1 if self.sensores["Pos_volante"] > 90 else -1)
        return 0


    def _fg_mag(self):
        a_long = self.sensores["A_lineal"]
        a_lateral = self._calcular_aceleracion_lateral()
        total = math.sqrt(a_long ** 2 + a_lateral ** 2)
        self.sensores["FG_mag"] = round(total / 9.81, 2)
        return self.sensores["FG_mag"]

    def _fg_angle(self):
        """Va de 0-360°.
        0° = Frontal (aceleración hacia adelante)
        90° = Lateral derecha
        180° = Trasera
        270° = Lateral izquierda.
        No necesariamente está implementado así en el auto!"""
        a_long = self.sensores["A_lineal"]
        a_lateral = self._calcular_aceleracion_lateral()

        if a_long == 0 and a_lateral == 0:
            angle = 0
        else:
            angle_rad = math.atan2(a_lateral, a_long)
            angle = math.degrees(angle_rad)
            if angle < 0:
                angle += 360

        self.sensores["FG_angle"] = int(angle)
        return self.sensores["FG_angle"]

    def _longitud(self):
        if self.sensores["Velocidad"] == 0:
            return self.sensores["Longitud"]

        velocidad_ms = self.sensores["Velocidad"] * (1000/3600)
        perimetro_pista = 2 * math.pi * self.radio_pista * 111320 #Perimetro es 2pi*radio, y metros = grados de long/lat * 1113200 (const)
        if perimetro_pista > 0:
            velocidad_angular = (velocidad_ms * FREQ_ARDUINO) / perimetro_pista * 360
            self.angulo_actual += velocidad_angular

            if self.angulo_actual >= 360:
                self.angulo_actual -= 360

            long_offset = self.radio_pista * math.cos(math.radians(self.angulo_actual))
            self.sensores["Longitud"] = self.centro_long + long_offset

        self.sensores["Longitud"] = round(self.sensores["Longitud"], 8)
        return self.sensores["Longitud"]

    def _latitud(self):
        if self.sensores["Velocidad"] == 0:
            return self.sensores["Latitud"]

        lat_offset = self.radio_pista * math.sin(math.radians(self.angulo_actual))
        factor_latitud = math.cos(math.radians(self.centro_lat))
        lat_offset_ajustado = lat_offset / factor_latitud
        self.sensores["Latitud"] = self.centro_lat + lat_offset_ajustado

        self.sensores["Latitud"] = round(self.sensores["Latitud"], 8)
        return self.sensores["Latitud"]

    def _pos_volante(self):
        """Va de 0 a 180° (0 = izquierda, 90 = centro, 180 = derecha)"""
        if self.sensores["Velocidad"] > 5:
            base = 85
            cambio = random.randint(-4, 4)
            pos = base + cambio
            self.sensores["Pos_volante"] = max(0, min(180, pos))
        else:
            self.sensores["Pos_volante"] = 90
        return self.sensores["Pos_volante"]

    def _f_suspension(self):
        peso_base = 300 * 9.81
        a_lateral = abs(self._calcular_aceleracion_lateral())
        a_long = abs(self.sensores["A_lineal"])

        return self.sensores["F_suspensión"]

    def _pres_neumaticos(self):
        p_base = 32 #PSI
        factor_temp = 1 + (self.sensores["T_aire"] - 20) * 0.003
        ruido = random.uniform(-1.5, 1.5)
        presion = p_base * factor_temp + ruido
        self.sensores["Pres_neumáticos"] = round(max(26, min(40, presion)), 1)
        return self.sensores["Pres_neumáticos"]

    def _t_neumaticos(self):
        t_base = self.sensores["T_aire"] + 15
        t_vel = self.sensores["Velocidad"] * 0.3
        t_fg = self.sensores["FG_mag"] * 6
        ruido = random.uniform(-3, 3)
        t_final = t_base + t_vel + t_fg + ruido
        self.sensores["T_neumáticos"] = round(max(15, min(100, t_final)), 1)
        return self.sensores["T_neumáticos"]

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

        output = [f"{t_actual:.2f}s"] + [str(self.sensores[i]) for i in self.sensores]
        return ",".join(output) + "\n"

    def imprimir_datos_inf(self):
        t_actual = 0
        while True:
            self._generar_datos(t_actual)
            output = [f"t={t_actual:.2f}s"]
            for key, value in self.sensores.items():
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
                header = ["tiempo"] + list(self.sensores.keys())
                f.write(",".join(header) + "\n")

            total_samples = int(tiempo / FREQ_ARDUINO)
            for i in range(total_samples):
                t_actual = i * FREQ_ARDUINO
                self._generar_datos(t_actual)

                if comando == "csv":
                    fila = [f"{t_actual:.2f}"] + [str(i) for i in self.sensores.values()]
                    f.write(",".join(fila) + "\n")
                else:
                    f.write(f"\nTiempo: {t_actual:.2f}\n")
                    for key, value in self.sensores.items():
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