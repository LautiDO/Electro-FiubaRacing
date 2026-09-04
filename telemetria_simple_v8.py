#!/usr/bin/env python3
"""
TELEMETRÍA UNIFICADA v3 — Raspberry Pi

La "pantalla" real es el display del cockpit conectado por CAN (no la consola
de la Raspberry). Con eso en claro, el fix es:

  - ECU en SU PROPIO HILO (como en la versión "unificada"): se lee de forma
    continua e independiente del resto, y el estado se actualiza SOLO cuando
    llega un frame válido — si una lectura puntual falla, se mantiene el
    último valor bueno (nunca cae a 0/None). Así el tablero no parpadea.


  - Envío CAN a la pantalla en SU PROPIO HILO, a una frecuencia fija (10 Hz),
    leyendo siempre el último estado "sticky" de la ECU y de la marcha. No
    depende de si el ciclo de lectura de la ECU tuvo éxito justo en ese
    instante — por eso antes se veía mal: se mandaban ceros cada vez que el
    poll serial no llegaba a tiempo dentro del mismo ciclo de 40 ms.

  - CSV por cola en su propio hilo (como en la versión "simplificada"): el
    disco nunca frena al resto.

  - Consola de la Raspberry (debug local, no es la pantalla del cockpit): en
    su propio hilo también, a 10 Hz, solo para verificar en el lugar.

Fuentes:
  STM32 → SPI     (marcha, ángulo, acel_lateral, accel xyz, giro xyz, dx1..dx5, presiones STM)
  ECU   → Serial  (RPM, TPS, AFR, temperaturas, batería) — hilo propio, sticky
  GPS   → CAN RX  (latitud, longitud, velocidad)
Salidas:
  CAN TX    → hilo propio, 10 Hz, manda el último estado sticky a la pantalla del cockpit
  CSV       → hilo propio con cola, no bloquea nada
  Consola   → hilo propio, debug local en la Raspberry
"""

import can, spidev, serial, struct, csv, time, os, threading, queue, subprocess
from datetime import datetime

# =============================================================
# CONFIGURACIÓN
# =============================================================

CICLO_S = 0.04        # 25 Hz — loop principal (SPI + armado de fila CSV)
DISPLAY_HZ = 10        # refresco de la consola LOCAL de la Raspberry (debug, no es el tablero)
DISPLAY_PERIODO_S = 1.0 / DISPLAY_HZ

CAN_TX_PERIODO_S = 0.1  # 10 Hz — cadencia del hilo que manda datos a la pantalla del cockpit

# --- SPI (STM32) ---
SPI_BUS, SPI_DEVICE = 1, 0
SPI_VELOCIDAD = 1_000_000
SPI_MODO = 0b00
STM32_SYNC = 0xAA

STM32_TAM_PKT = 66        # 1 sync + 15 floats + 1 int32 (cambio) + 1 checksum
STM32_FORMATO = '<15fi'  # little-endian: 15 floats + 1 int32 (cambio)

# --- ECU (Serial) ---
ECU_PUERTO    = "/dev/ttyUSB0"
ECU_BAUDRATE  = 115200
ECU_HEADER    = b'\xFF\x7F'
ECU_FOOTER    = b'\x7F\xFF'

ECU_PERIODO_S        = 0.04   # 25 Hz — cada cuánto se reenvía el pedido a la ECU
ECU_RETRY_ESPERA      = 3.0   # segundos entre reintentos de apertura del puerto
ECU_BUFFER_MAX        = 2048  # tope de bytes en buffer sin lograr sync
ECU_RESYNC_TIMEOUT_S  = 2.0   # si no se arma un frame válido en este tiempo, se purga el buffer

# NOTA: el protocolo de la ECU tal como está implementado solo pide el comando 6,
# que trae rpm/tps/afr/temp_refrigerante/temp_aire/batería. presion_combustible,
# presion_aceite y temp_combustible NO vienen de la ECU por este comando — si tu
# ECU los expone por otro comando (ej. comando 7, como se intentaba sin éxito en
# la versión "unificada"), avisame el formato de esa trama y lo agrego acá.

# --- CAN ---
CAN_CANAL   = 'can0'
CAN_BITRATE = 1_000_000

CAN_ID_MOTOR       = 0x640
CAN_ID_PRES_COMB   = 0x641
CAN_ID_PRES_ACEITE = 0x644
CAN_ID_TEMPS       = 0x649
CAN_ID_MARCHA      = 0x64D
CAN_ID_LAMBDA       = 0x460   # sonda lambda (Ls LTC 1) — byte0-1, unsigned, resolución 0.001

CAN_ID_GPS_LATLON = 0x680
CAN_ID_GPS_TIEMPO = 0x681
CAN_ID_GPS_FECHA  = 0x682

SIG_RPM         = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1,    'add': 0}
SIG_TPS         = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 10, 'add': 0} #FIX ACA, OVERFLOW CON DIV: 1000
SIG_PRES_COMB   = {'off': 4, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_PRES_ACEITE = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_TEMP_REFRIG = {'off': 0, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_AIRE   = {'off': 1, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_COMB   = {'off': 2, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_MARCHA      = {'off': 6, 'len': 1, 'mask': 0x0F,   'mult': 1,    'div': 1,    'add': 0}
SIG_BATERIA     = {'off': 5, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 10,   'add': 0}
SIG_LAMBDA      = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}  # resolución 0.001

# --- CSV ---


CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
os.makedirs(CSV_DIR, exist_ok=True)
def _nombre_csv_unico(): #SACAR EL TIEMPO SOLO FUNCIONA SI HAY INTERNET!! ASI QUE AHORA SI EL ARCHIVO YA EXISTIA LO PONE EN OTRO ARCHIVO Y LE CAMBIA EL NOMBRE A nombre_1, nombre_2 etc
    base = datetime.now().strftime('%Y%m%d_%H-%M-%S')
    candidato = os.path.join(CSV_DIR, f"sesion_{base}.csv")
    n = 1
    while os.path.exists(candidato):
        candidato = os.path.join(CSV_DIR, f"sesion_{base}_{n}.csv")
        n += 1
    return candidato

CSV_NOMBRE = _nombre_csv_unico()
FSYNC_CADA_S = 0.5

CSV_COLUMNAS = [
    "tiempo_pc_ms",
    "marcha", "angulo", "acel_lateral",
    "acel_x", "acel_y", "acel_z",
    "giro_x", "giro_y", "giro_z",
    "dx1", "dx2", "dx3", "dx4", "dx5",
    "stm_presion1", "stm_presion2",
    "rpm", "tps", "afr",
    "temp_refrigerante", "temp_aire", "temp_combustible",
    "presion_combustible", "presion_aceite", "bateria_v",
    "gps_lat", "gps_lon", "gps_vel_kmh",
]

STM_CAMPOS = ["marcha", "angulo", "acel_lateral", "acel_x", "acel_y", "acel_z",
              "giro_x", "giro_y", "giro_z", "dx1", "dx2", "dx3", "dx4", "dx5",
              "stm_presion1", "stm_presion2"]
ECU_CAMPOS = ["rpm", "tps", "afr", "temp_refrigerante", "temp_aire", "temp_combustible",
              "presion_combustible", "presion_aceite", "bateria_v"]
GPS_CAMPOS = ["gps_lat", "gps_lon", "gps_vel_kmh"]

NOMBRES_MARCHA = {0: "N", 1: "1ª", 2: "2ª", 3: "3ª", 4: "4ª", 5: "5ª", 6: "6ª"}

# =============================================================
# ESTADO COMPARTIDO
# =============================================================

_lock_gps = threading.Lock()
_estado_gps = {"gps_lat": None, "gps_lon": None, "gps_vel_kmh": None}

def guardar_gps(datos):
    with _lock_gps:
        _estado_gps.update(datos)

def leer_gps():
    with _lock_gps:
        return dict(_estado_gps)

# Estado STICKY de la ECU: solo se actualiza cuando llega un frame válido.
# Si una lectura puntual falla, se mantiene el último valor bueno — nunca
# cae a 0/None por un timeout aislado. Esto es lo que evita el parpadeo
# en la pantalla del cockpit.
_lock_ecu = threading.Lock()
_estado_ecu = {
    "rpm": None, "tps": None, "afr": None,
    "temp_refrigerante": None, "temp_aire": None, "temp_combustible": None,
    "presion_combustible": None, "presion_aceite": None, "bateria_v": None,
}

def guardar_ecu(datos):
    with _lock_ecu:
        _estado_ecu.update(datos)

def leer_ecu_sticky():
    with _lock_ecu:
        return dict(_estado_ecu)

# Marcha STICKY (la lee el hilo de CAN TX; la actualiza el loop principal
# cada vez que llega un paquete válido del STM32).
_lock_marcha = threading.Lock()
_marcha = 0

def guardar_marcha(valor):
    global _marcha
    with _lock_marcha:
        _marcha = int(valor)

def leer_marcha():
    with _lock_marcha:
        return _marcha

# Estado STICKY del STM32: igual que con la ECU, solo se actualiza cuando el
# frame pasa el filtro de plausibilidad (ver _stm_frame_valida). Si un frame
# viene desalineado/corrupto (lectura SPI que agarra al STM32 a mitad de
# sobreescribir su buffer), se descarta entero y se conserva el último valor
# bueno — así los picos de ruido no llegan ni al CSV ni a la pantalla.
_lock_stm = threading.Lock()
_estado_stm_sticky = {k: None for k in STM_CAMPOS}

def guardar_stm(datos):
    with _lock_stm:
        _estado_stm_sticky.update(datos)

def leer_stm_sticky():
    with _lock_stm:
        return dict(_estado_stm_sticky)

# Rangos físicos plausibles + salto máximo permitido entre dos ciclos
# consecutivos (CICLO_S = 40 ms). AJUSTAR estos valores a los rangos reales
# de tus sensores (fondo de escala del acelerómetro/giróscopo, recorrido
# físico de los potenciómetros de suspensión, tope del volante, etc.) —
# los que puse acá son conservadores/genéricos, no vinieron de tu hardware.
STM_LIMITES = {
    "angulo":       {"min": -450,  "max": 450,  "delta_max": 200},   # ° volante
    "acel_lateral": {"min": -10,   "max": 10,   "delta_max": 5},     # g
    "acel_x":       {"min": -16,   "max": 16,   "delta_max": 8},     # g
    "acel_y":       {"min": -16,   "max": 16,   "delta_max": 8},     # g
    "acel_z":       {"min": -16,   "max": 16,   "delta_max": 8},     # g
    "giro_x":       {"min": -2000, "max": 2000, "delta_max": 1000},  # °/s
    "giro_y":       {"min": -2000, "max": 2000, "delta_max": 1000},  # °/s
    "giro_z":       {"min": -2000, "max": 2000, "delta_max": 1000},  # °/s
    "dx1":          {"min": 0,     "max": 150,  "delta_max": 60},    # mm
    "dx2":          {"min": 0,     "max": 150,  "delta_max": 60},    # mm
    "dx3":          {"min": 0,     "max": 150,  "delta_max": 60},    # mm
    "dx4":          {"min": 0,     "max": 150,  "delta_max": 60},    # mm
    "dx5":          {"min": 0,     "max": 150,  "delta_max": 60},    # mm
}

def _stm_frame_valida(nuevo, anterior):
    """True si todos los campos del frame nuevo están dentro de rango físico
    plausible y no pegaron un salto imposible respecto del último valor
    bueno. Un solo campo fuera de rango tira todo el frame (si el paquete
    vino desalineado, lo más probable es que TODOS los campos estén mal,
    no uno solo)."""
    for campo, lim in STM_LIMITES.items():
        val = nuevo.get(campo)
        if val is None:
            continue
        if not (lim["min"] <= val <= lim["max"]):
            return False
        prev = anterior.get(campo)
        if prev is not None and abs(val - prev) > lim["delta_max"]:
            return False
    return True

# Estado para la PANTALLA. Se actualiza (barato, solo un dict) desde el loop
# principal, y lo lee el hilo de consola a su propio ritmo. No bloquea nada.
_lock_display = threading.Lock()
_ultimo_estado_display = None

def actualizar_display(estado):
    global _ultimo_estado_display
    with _lock_display:
        _ultimo_estado_display = estado

def leer_estado_display():
    with _lock_display:
        return _ultimo_estado_display

# =============================================================
# ECU — protocolo serial propietario
# =============================================================

def _checksum_ecu(trama):
    return struct.pack('<I', sum(trama) & 0xFFFFFFFF)

def _armar_pedido_ecu(cmd):
    n = 10
    base = ECU_HEADER + bytes([cmd[0], cmd[1], cmd[2], n]) + ECU_FOOTER
    chk = _checksum_ecu(base)
    return bytearray(ECU_HEADER + bytes([cmd[0], cmd[1]]) + chk + bytes([cmd[2], n]) + ECU_FOOTER)

def _decodificar_ecu(payload):
    try:
        rpm        = struct.unpack_from('>h', payload, 8)[0]
        afr        = struct.unpack_from('B',  payload, 10)[0] / 10.0
        tps        = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp_motor = struct.unpack_from('>h', payload, 18)[0] / 10.0
        temp_aire  = struct.unpack_from('>h', payload, 20)[0] / 10.0
        vbat       = struct.unpack_from('>h', payload, 22)[0] / 100.0
    except struct.error:
        return None
    return {
        "rpm": rpm, "tps": tps, "afr": afr,
        "temp_refrigerante": temp_motor, "temp_aire": temp_aire, "temp_combustible": 0,
        "presion_combustible": 0,
        "presion_aceite": 0,
        "bateria_v": vbat,
    }

def hilo_ecu(stop):
    """
    Copiado del pedido de paquetes de telemetria_unificada_v12.py (el que
    andaba bien). Diferencias clave contra la versión anterior de este
    archivo:
      - Puerto serial NO bloqueante (timeout=0): se lee lo que haya
        disponible en cada vuelta con ser.in_waiting, sin esperar.
      - El pedido a la ECU se reenvía por tiempo transcurrido
        (time.perf_counter() >= ECU_PERIODO_S desde el último pedido),
        no arrancando un write+read nuevo cada ciclo — así la respuesta
        puede llegar repartida en varias vueltas del loop sin perderse.
      - Buffer persistente entre iteraciones: los bytes que llegan se van
        acumulando y se van sacando frames completos (HEADER...FOOTER) a
        medida que se arman, en vez de tirar todo lo que no llegó a
        completarse en una sola lectura.
      - Watchdog: si el buffer crece de más o pasa mucho tiempo sin poder
        armar un frame válido, se purga y se vuelve a sincronizar desde cero.
    Esto es lo que evita el "todo 0": antes se descartaba cualquier
    respuesta que no llegara completa dentro de los 64 bytes leídos en el
    mismo instante del write, y con el puerto no bloqueando eso pasaba seguido.
    """
    ser = None
    intentos = 0
    while ser is None and not stop.is_set():
        try:
            ser = serial.Serial(ECU_PUERTO, ECU_BAUDRATE, timeout=0)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception as e:
            intentos += 1
            if intentos == 1 or intentos % 10 == 0:
                print(f"[ECU] No se pudo abrir {ECU_PUERTO} (intento {intentos}): {e} — reintentando cada {ECU_RETRY_ESPERA:.0f}s...")
            time.sleep(ECU_RETRY_ESPERA)

    if stop.is_set():
        return  # se pidió salir mientras esperábamos el puerto

    print(f"[ECU] Conectado en {ECU_PUERTO} @ {ECU_BAUDRATE} baud")

    try:
        ser.write(_armar_pedido_ecu([0, 0, 0]))
        time.sleep(0.05)
        ser.reset_input_buffer()
        print("[ECU] Handshake OK")
    except Exception as e:
        print(f"[ECU] Error en handshake: {e}")

    pedido = _armar_pedido_ecu([6, 0, 0])
    buffer = bytearray()
    ultimo = 0.0
    ultimo_frame_ok = time.perf_counter()

    while not stop.is_set():
        try:
            ahora = time.perf_counter()

            if ahora - ultimo >= ECU_PERIODO_S:
                ser.write(pedido)
                ultimo = ahora

            n = ser.in_waiting
            if n:
                buffer.extend(ser.read(n))

            hubo_frame = False
            while ECU_HEADER in buffer and ECU_FOOTER in buffer:
                ini = buffer.find(ECU_HEADER)
                fin = buffer.find(ECU_FOOTER, ini)
                if ini == -1 or fin == -1:
                    break
                trama  = buffer[ini: fin + 2]
                buffer = buffer[fin + 2:]

                if len(trama) > 12 and trama[2] == 180:
                    datos = _decodificar_ecu(trama[10:-2])
                    if datos:
                        guardar_ecu(datos)
                        hubo_frame = True

            if hubo_frame:
                ultimo_frame_ok = ahora

            # --- watchdog anti-desincronización ---
            if len(buffer) > ECU_BUFFER_MAX:
                print(f"[ECU] Buffer > {ECU_BUFFER_MAX} bytes sin sync, purgando...")
                buffer.clear()
                ultimo_frame_ok = ahora
            elif (ahora - ultimo_frame_ok) > ECU_RESYNC_TIMEOUT_S and len(buffer) > 0:
                print(f"[ECU] Sin frames válidos hace {ECU_RESYNC_TIMEOUT_S:.0f}s, purgando buffer ({len(buffer)} bytes)...")
                buffer.clear()
                ultimo_frame_ok = ahora

            time.sleep(0.001)

        except serial.SerialException as e:
            print(f"[ECU] Puerto perdido: {e} — reintentando en 3 s...")
            buffer.clear()
            time.sleep(3.0)
            try:
                ser.close()
                ser.open()
                ser.reset_input_buffer()
                print("[ECU] Reconectado OK")
            except Exception as e2:
                print(f"[ECU] Reconexión fallida: {e2}")

        except Exception as e:
            print(f"[ECU] Error inesperado (se sigue intentando): {e}")
            buffer.clear()
            time.sleep(0.1)

    ser.close()
    print("[ECU] Puerto cerrado.")

# =============================================================
# STM32 — lectura SPI
# =============================================================

def leer_stm32(spi):
    for _ in range(STM32_TAM_PKT * 2):
        b = spi.xfer2([0x00])[0]
        if b == STM32_SYNC:
            resto = bytes(spi.xfer2([0x00] * (STM32_TAM_PKT - 1)))
            raw = bytes([STM32_SYNC]) + resto

            # Verificación de checksum (XOR de los bytes [1..64], comparado
            # contra el byte [65]). Si no coincide, el frame vino
            # desalineado/torn — se descarta directamente en vez de
            # decodificar floats basura.
            chk_calculado = 0
            for byte in raw[1:65]:
                chk_calculado ^= byte
            chk_recibido = raw[65]
            if chk_calculado != chk_recibido:
                return None

            try:
                (angulo, acel_lateral,
                 ax, ay, az, gx, gy, gz,
                 dx1, dx2, dx3, dx4, dx5,
                 presion1, presion2,
                 cambio) = struct.unpack_from(STM32_FORMATO, raw, 1)
            except struct.error:
                return None
            return {
                "marcha": cambio, "angulo": angulo, "acel_lateral": acel_lateral,
                "acel_x": ax, "acel_y": ay, "acel_z": az,
                "giro_x": gx, "giro_y": gy, "giro_z": gz,
                "dx1": dx1, "dx2": dx2, "dx3": dx3, "dx4": dx4, "dx5": dx5,
                "stm_presion1": presion1, "stm_presion2": presion2,
            }
    return None

# =============================================================
# GPS / CAN RX (hilo aparte)
# =============================================================

def _decodificar_gps_latlon(data):
    lat = struct.unpack_from('>i', data, 0)[0] * 1e-7
    lon = struct.unpack_from('>i', data, 4)[0] * 1e-7
    return {"gps_lat": lat, "gps_lon": lon}

def _decodificar_gps_tiempo(data):
    return {"gps_vel_kmh": data[5] * 0.1}

GPS_DECODIFICADORES = {
    CAN_ID_GPS_LATLON: _decodificar_gps_latlon,
    CAN_ID_GPS_TIEMPO: _decodificar_gps_tiempo,
}

def hilo_gps(bus_can, stop):
    print("[GPS] Escuchando mensajes GPS desde CAN...")
    while not stop.is_set():
        try:
            msg = bus_can.recv(timeout=0.1)
        except can.CanError:
            time.sleep(0.1)
            continue
        if msg is None:
            continue
        decoder = GPS_DECODIFICADORES.get(msg.arbitration_id)
        if decoder:
            guardar_gps(decoder(bytes(msg.data)))
    print("[GPS] Hilo detenido.")

# =============================================================
# CAN TX (loop principal)
# =============================================================

def _empaquetar_senal(payload, valor, sig):
    val = valor if valor is not None else 0
    raw = int((val + sig['add']) * sig['div'] * sig['mult']) & sig['mask']
    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']]     = (raw >> 8) & 0xFF
        payload[sig['off'] + 1] =  raw        & 0xFF
    return payload

def enviar_can(bus_can, ecu, marcha):
    def enviar(can_id, señales):
        payload = [0] * 8
        for valor, sig in señales:
            _empaquetar_senal(payload, valor, sig)
        bus_can.send(can.Message(arbitration_id=can_id, data=payload, is_extended_id=False))

    datos_ecu = {
        'rpm': 0, 'tps': 0, 'afr': 0, 'temp_refrigerante': 0,
        'temp_aire': 0, 'temp_combustible': 0, 'bateria_v': 0,
        'presion_combustible': 0, 'presion_aceite': 0
    }
    if ecu is not None:
        datos_ecu.update(ecu)

    enviar(CAN_ID_MOTOR,       [(datos_ecu['rpm'], SIG_RPM), (datos_ecu['tps'], SIG_TPS)])
    enviar(CAN_ID_TEMPS,       [(datos_ecu['temp_refrigerante'], SIG_TEMP_REFRIG),
                                (datos_ecu['temp_aire'],         SIG_TEMP_AIRE),
                                (datos_ecu['temp_combustible'],    SIG_TEMP_COMB),
                                (datos_ecu['bateria_v'],         SIG_BATERIA)])
    enviar(CAN_ID_PRES_COMB,   [(datos_ecu['presion_combustible'], SIG_PRES_COMB)])
    enviar(CAN_ID_PRES_ACEITE, [(datos_ecu['presion_aceite'],      SIG_PRES_ACEITE)])
    enviar(CAN_ID_MARCHA,      [(marcha, SIG_MARCHA)])
    enviar(CAN_ID_LAMBDA,      [(datos_ecu['afr'], SIG_LAMBDA)])

def hilo_can_pantalla(bus_can, stop):
    """
    Envía el estado STICKY (último valor bueno de la ECU + última marcha)
    a la pantalla del cockpit, a un ritmo fijo. No depende de si la ECU
    contestó justo en este instante, así que la pantalla no parpadea.
    """
    print(f"[CAN-TX] Enviando a la pantalla cada {CAN_TX_PERIODO_S*1000:.0f} ms")
    while not stop.is_set():
        t0 = time.monotonic()
        try:
            ecu = leer_ecu_sticky()
            marcha = leer_marcha()
            enviar_can(bus_can, ecu, marcha)
        except can.CanError as e:
            print(f"[CAN-TX] Error de bus: {e}")
        except Exception as e:
            print(f"[CAN-TX] Error inesperado: {e}")

        t_espera = CAN_TX_PERIODO_S - (time.monotonic() - t0)
        if t_espera > 0:
            time.sleep(t_espera)
    print("[CAN-TX] Hilo detenido.")


class CanManager:
    def __init__(self, canal, bitrate, intentos=10, espera_s=1.0):
        self.canal = canal
        self.bitrate = bitrate
        self._lock = threading.Lock()

        # Reintenta abrir el bus (y relevanta la interfaz si hace falta)
        # en vez de morir en el primer intento fallido. Esto cubre el caso
        # en que el script arranca antes de que can0 termine de quedar UP.
        ultimo_error = None
        for intento in range(1, intentos + 1):
            try:
                self.bus = self._abrir()
                if intento > 1:
                    print(f"[CAN] {canal} abierto en el intento {intento}")
                return
            except Exception as e:
                ultimo_error = e
                print(f"[CAN] Intento {intento}/{intentos} de abrir {canal} falló: {e}")
                estado = _estado_interfaz(canal)
                print(f"[CAN] Estado detectado: {estado}")
                if estado != 'UP':
                    _levantar_interfaz(canal, bitrate)
                time.sleep(espera_s)

        raise RuntimeError(f"No se pudo abrir {canal} tras {intentos} intentos: {ultimo_error}")

    def _abrir(self):
        return can.interface.Bus(channel=self.canal, bustype='socketcan')

    def _reconectar(self):
        with self._lock:
            try:
                self.bus.shutdown()
            except Exception:
                pass

            estado = _estado_interfaz(self.canal)
            print(f"[CAN] Estado detectado: {estado}")

            if estado != 'UP':
                _levantar_interfaz(self.canal, self.bitrate)

            try:
                self.bus = self._abrir()
                print("[CAN] Bus reabierto en Python")
            except Exception as e:
                print(f"[CAN] No se pudo reabrir el bus: {e}")

    def send(self, msg):
        try:
            with self._lock:
                bus = self.bus
            bus.send(msg)
        except (can.CanError, OSError) as e:
            print(f"[CAN] Error al enviar ({e}), reconectando...")
            self._reconectar()

    def recv(self, timeout):
        try:
            with self._lock:
                bus = self.bus
            return bus.recv(timeout=timeout)
        except (can.CanError, OSError) as e:
            print(f"[CAN] Error al recibir ({e}), reconectando...")
            self._reconectar()
            return None

    def shutdown(self):
        with self._lock:
            try:
                self.bus.shutdown()
            except Exception:
                pass

def _estado_interfaz(canal):
    """Devuelve 'UP' solo si está operacionalmente arriba (no solo administrativamente)."""
    try:
        with open(f'/sys/class/net/{canal}/operstate') as f:
            operstate = f.read().strip()
        # operstate puede ser: 'up', 'down', 'unknown' (a veces CAN reporta 'unknown' aun sano)
        return 'UP' if operstate == 'up' else 'DOWN'
    except Exception as e:
        print(f"[CAN] No se pudo leer operstate de {canal}: {e}")
        return None

def _levantar_interfaz(canal, bitrate):
    print(f"[CAN] Levantando {canal}...")
    try:
        subprocess.run(['sudo', 'ip', 'link', 'set', canal, 'down'], timeout=2)
        subprocess.run(['sudo', 'ip', 'link', 'set', canal, 'type', 'can',
                         'bitrate', str(bitrate)], timeout=2)
        subprocess.run(['sudo', 'ip', 'link', 'set', canal, 'up'], timeout=2)
        time.sleep(0.2)
        print(f"[CAN] {canal} levantado")
        return True
    except Exception as e:
        print(f"[CAN] Error levantando {canal}: {e}")
        return False

# =============================================================
# CSV — hilo aparte con cola (esto es lo que hace que se guarde bien)
# =============================================================

def hilo_csv(cola, csv_file, csv_writer, stop):
    ultimo_fsync = time.monotonic()
    while not stop.is_set() or not cola.empty():
        try:
            fila = cola.get(timeout=0.1)
        except queue.Empty:
            continue

        csv_writer.writerow(fila)
        csv_file.flush()

        if time.monotonic() - ultimo_fsync >= FSYNC_CADA_S:
            try:
                os.fsync(csv_file.fileno())
            except Exception as e:
                print(f"[CSV] Error en fsync: {e}")
            ultimo_fsync = time.monotonic()

    try:
        os.fsync(csv_file.fileno())
    except Exception:
        pass
    print("[CSV] Hilo detenido y archivo guardado.")

# =============================================================
# CONSOLA — hilo aparte, dashboard completo, no bloquea el loop real-time
# =============================================================

VERDE = "\033[92m"; AMARILLO = "\033[93m"; ROJO = "\033[91m"
CYAN  = "\033[96m"; GRIS     = "\033[90m"; RESET = "\033[0m"
BOLD  = "\033[1m";  CLEAR    = "\033[2J\033[H"

def _fmt(val, dec=1):
    return f"{'---':>10}" if val is None else f"{val:>10.{dec}f}"

def mostrar_consola(estado):
    tasa_err = (estado['n_stm_fail'] / estado['n_muestras'] * 100) if estado['n_muestras'] else 0.0
    marcha   = NOMBRES_MARCHA.get(estado['marcha'], str(estado['marcha']))
    c_err    = VERDE if tasa_err < 1 else (AMARILLO if tasa_err < 5 else ROJO)
    c_ecu    = VERDE if estado['ecu_ok'] else ROJO

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*56}{RESET}")
    print(f"{BOLD}{CYAN}   TELEMETRÍA — STM32 + ECU + GPS{RESET}")
    print(f"{CYAN}{'='*56}{RESET}")
    print(f"\n  {BOLD}Muestra #{estado['n_muestras']:<6}{RESET}  t_PC={estado['t_ms']:.1f} ms\n")

    print(f"  {'Marcha':<22}  {marcha:>10}")
    print(f"  {'Ángulo volante':<22}  {_fmt(estado['angulo'], 1)}  °")
    print(f"  {'Acel. lateral':<22}  {_fmt(estado['acel_lateral'], 3)}  g")
    print(f"  {'Acel X/Y/Z':<22}  {_fmt(estado['acel_x'],2)} {_fmt(estado['acel_y'],2)} {_fmt(estado['acel_z'],2)}")
    print(f"  {'Giro X/Y/Z':<22}  {_fmt(estado['giro_x'],2)} {_fmt(estado['giro_y'],2)} {_fmt(estado['giro_z'],2)}")
    print(f"  {'dx1':<22}  {_fmt(estado['dx1'], 1)}  mm")
    print(f"  {'dx2':<22}  {_fmt(estado['dx2'], 1)}  mm")
    print(f"  {'dx3':<22}  {_fmt(estado['dx3'], 1)}  mm")
    print(f"  {'dx4':<22}  {_fmt(estado['dx4'], 1)}  mm")
    print(f"  {'dx5':<22}  {_fmt(estado['dx5'], 1)}  mm")
    print(f"  {'Presión 1 (STM)':<22}  {_fmt(estado['stm_presion1'], 1)}")
    print(f"  {'Presión 2 (STM)':<22}  {_fmt(estado['stm_presion2'], 1)}")

    print(f"  {'RPM':<22}  {_fmt(estado['rpm'], 0)}  rpm")
    print(f"  {'TPS':<22}  {_fmt(estado['tps'], 1)}  %")
    print(f"  {'AFR':<22}  {_fmt(estado['afr'], 1)}")
    print(f"  {'Temp. refrigerante':<22}  {_fmt(estado['temp_refrigerante'])}  °C")
    print(f"  {'Temp. aire':<22}  {_fmt(estado['temp_aire'])}  °C")
    print(f"  {'Batería':<22}  {_fmt(estado['bateria_v'], 2)}  V")
    print(f"  {c_ecu}{'ECU':<22}  {'OK' if estado['ecu_ok'] else 'SIN DATOS'}{RESET}")

    print(f"  {'Latitud':<22}  {_fmt(estado['gps_lat'], 7)}  °")
    print(f"  {'Longitud':<22}  {_fmt(estado['gps_lon'], 7)}  °")
    print(f"  {'Velocidad':<22}  {_fmt(estado['gps_vel_kmh'])}  km/h")

    print(f"\n  Lecturas SPI fallidas: {c_err}{estado['n_stm_fail']} ({tasa_err:.1f}%){RESET}")
    print(f"  CSV perdidos (cola llena): {estado['n_csv_perdidas']}")
    print(f"\n  {GRIS}CSV → {CSV_NOMBRE}{RESET}")
    print(f"  {CYAN}Ctrl+C para detener{RESET}\n")

def hilo_consola(stop):
    while not stop.is_set():
        estado = leer_estado_display()
        if estado is not None:
            try:
                mostrar_consola(estado)
            except Exception as e:
                print(f"[DISPLAY] Error mostrando estado: {e}")
        time.sleep(DISPLAY_PERIODO_S)
    print("[DISPLAY] Hilo detenido.")

# =============================================================
# MAIN
# =============================================================

def main():
    print("[SPI] Abriendo STM32...")
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_VELOCIDAD
    spi.mode = SPI_MODO
    spi.bits_per_word = 8

    print(f"[CAN] Inicializando canal único {CAN_CANAL}...")
    try:
        bus_can = CanManager(CAN_CANAL, CAN_BITRATE)
    except Exception as e:
        print(f"[CAN] Error crítico al abrir interfaz CAN: {e}")
        return

    csv_file = open(CSV_NOMBRE, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNAS)
    csv_writer.writeheader()
    csv_file.flush()
    os.fsync(csv_file.fileno())
    print(f"[CSV] Guardando en: {CSV_NOMBRE}")

    stop = threading.Event()
    cola_csv = queue.Queue(maxsize=500)

    hilos = [
        threading.Thread(target=hilo_ecu,         args=(stop,), daemon=True),
        threading.Thread(target=hilo_can_pantalla, args=(bus_can, stop), daemon=True),
        threading.Thread(target=hilo_gps,         args=(bus_can, stop), daemon=True),
        threading.Thread(target=hilo_csv,         args=(cola_csv, csv_file, csv_writer, stop), daemon=True),
        threading.Thread(target=hilo_consola,     args=(stop,), daemon=True),
    ]
    for h in hilos:
        h.start()

    t_inicio = time.monotonic()
    n_muestras = n_stm_fail = n_csv_perdidas = 0

    try:
        while True:
            t0 = time.monotonic()

            try:
                stm = leer_stm32(spi)
            except Exception as e:
                print(f"[SPI] Error: {e}")
                stm = None

            if stm is not None:
                anterior_stm = leer_stm_sticky()
                if _stm_frame_valida(stm, anterior_stm):
                    guardar_marcha(stm["marcha"])
                    guardar_stm(stm)
                else:
                    n_stm_fail += 1
                    # Frame descartado por valores fuera de rango o salto
                    # abrupto (probable lectura SPI desalineada). Se
                    # mantiene el último valor bueno (sticky), no se guarda.
            else:
                n_stm_fail += 1

            stm_sticky = leer_stm_sticky()   # último frame bueno, nunca cae a 0/None
            ecu = leer_ecu_sticky()   # último estado bueno, sin bloquear
            gps = leer_gps()
            marcha_actual = leer_marcha()

            fila = {"tiempo_pc_ms": round((time.monotonic() - t_inicio) * 1000, 3)}
            fila.update({k: None for k in STM_CAMPOS})
            fila.update(ecu)          # ya trae los 9 campos de ECU_CAMPOS (o None si nunca hubo dato)
            fila.update({k: None for k in GPS_CAMPOS})
            fila.update(stm_sticky)
            fila.update(gps)

            try:
                cola_csv.put_nowait(fila)
            except queue.Full:
                n_csv_perdidas += 1

            n_muestras += 1

            # Actualizar estado para la consola local (barato: un dict, sin I/O)
            fila_display = dict(fila)
            fila_display.update({
                "marcha": marcha_actual,
                "t_ms": fila["tiempo_pc_ms"],
                "n_muestras": n_muestras,
                "n_stm_fail": n_stm_fail,
                "n_csv_perdidas": n_csv_perdidas,
                "ecu_ok": ecu.get("rpm") is not None,
            })
            actualizar_display(fila_display)

            t_espera = CICLO_S - (time.monotonic() - t0)
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print("\n[OK] Interrupción por teclado. Deteniendo...")

    finally:
        stop.set()
        for h in hilos:
            h.join(timeout=2.0)

        spi.close()
        bus_can.shutdown()
        csv_file.close()
        print(f"[FIN] Proceso terminado. CSV guardado correctamente.")

if __name__ == "__main__":
    main()
