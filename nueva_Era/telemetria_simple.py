#!/usr/bin/env python3
"""
TELEMETRÍA SIMPLIFICADA — Raspberry Pi

Loop principal a 25 Hz, con 2 hilos de apoyo (no más que eso):
  - Loop principal: pide trama a la ECU (Serial), pide trama al STM32 (SPI),
    manda ECU + marcha por CAN a la pantalla, y encola la fila para el CSV.
    Es el único que le importa el tiempo real (25 Hz), así que nunca espera
    a disco ni al bus CAN de entrada.
  - Hilo GPS: la pantalla mide posición/velocidad y se las manda de vuelta
    a la Raspi por CAN (bidireccional). Este hilo solo escucha ese bus y
    guarda el último valor recibido en una variable compartida.
  - Hilo CSV: saca filas de una cola y las escribe a disco. Si la SD anda
    lenta, se atrasa él solo — nunca frena el loop principal.

No se maneja timestamp del STM32 ni matcheo por tiempo entre ECU y STM32:
como ambos se piden dentro del mismo ciclo de 40 ms, ya quedan alineados
fila a fila.
"""

import can, spidev, serial, struct, csv, time, os, threading, queue
from datetime import datetime

# =============================================================
# CONFIGURACIÓN
# =============================================================

CICLO_S = 0.04  # 25 Hz

# --- SPI (STM32) ---
SPI_BUS, SPI_DEVICE = 1, 0
SPI_VELOCIDAD = 1_000_000
SPI_MODO = 0b00
STM32_SYNC = 0xAA

# Paquete armado en armarPaquete() del .ino (65 bytes, sin tiempo_us):
#   [0]      0xAA          sync
#   [1..4]   angulo        float
#   [5..8]   acel_lateral  float
#   [9..12]  ax            float   (IMU cruda)
#   [13..16] ay            float   (IMU cruda)
#   [17..20] az            float   (IMU cruda)
#   [21..24] gx            float   (IMU cruda)
#   [25..28] gy            float   (IMU cruda)
#   [29..32] gz            float   (IMU cruda)
#   [33..36] dx1           float
#   [37..40] dx2           float
#   [41..44] dx3           float
#   [45..48] dx4           float
#   [49..52] dx5           float
#   [53..56] presion1      float
#   [57..60] presion2      float
#   [61..64] cambio        int32
STM32_TAM_PKT = 65
STM32_FORMATO = '<15fi'  # little-endian: 15 floats seguidos de 1 int32 (cambio)

# --- ECU (Serial) ---
ECU_PUERTO    = "/dev/ttyUSB0"
ECU_BAUDRATE  = 115200
ECU_TIMEOUT_S = 0.03      # tope de espera de respuesta, dentro del ciclo de 40 ms
ECU_HEADER    = b'\xFF\x7F'
ECU_FOOTER    = b'\x7F\xFF'

# --- CAN ---
CAN_CANAL   = 'can0'
CAN_BITRATE = 1_000_000

# IDs de salida (Raspi → pantalla)
CAN_ID_MOTOR       = 0x640
CAN_ID_PRES_COMB   = 0x641
CAN_ID_PRES_ACEITE = 0x644
CAN_ID_TEMPS       = 0x649
CAN_ID_MARCHA      = 0x64D

# IDs de entrada (pantalla → Raspi): posición y velocidad medidas por la pantalla
CAN_ID_GPS_LATLON = 0x680
CAN_ID_GPS_TIEMPO = 0x681  # incluye la velocidad (ver _decodificar_gps_tiempo)
CAN_ID_GPS_FECHA  = 0x682

# --- Señales CAN TX (offset, largo, máscara, multiplicador, divisor, offset_valor) ---
SIG_RPM         = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1,    'add': 0}
SIG_TPS         = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1000, 'add': 0}
SIG_PRES_COMB   = {'off': 4, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_PRES_ACEITE = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_TEMP_REFRIG = {'off': 0, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_AIRE = {'off': 1, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_COMB   = {'off': 2, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_MARCHA      = {'off': 6, 'len': 1, 'mask': 0x0F,   'mult': 1,    'div': 1,    'add': 0}
SIG_BATERIA     = {'off': 5, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 10,   'add': 0}

# --- CSV ---
CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
os.makedirs(CSV_DIR, exist_ok=True)
CSV_NOMBRE = os.path.join(CSV_DIR, f"sesion_{datetime.now().strftime('%Y%m%d_%H-%M-%S')}.csv")
FSYNC_CADA_S = 0.5   # el hilo de CSV hace fsync a disco como máximo cada X segundos

CSV_COLUMNAS = [
    "tiempo_pc_ms",
    "marcha", "angulo", "acel_lateral",
    "acel_x", "acel_y", "acel_z",
    "giro_x", "giro_y", "giro_z",
    "dx1", "dx2", "dx3", "dx4", "dx5",
    "stm_presion1", "stm_presion2",
    "rpm", "tps",
    "temp_refrigerante", "temp_aire", "temp_combustible",
    "presion_combustible", "presion_aceite", "bateria_v",
    "gps_lat", "gps_lon", "gps_vel_kmh","afr",
]

STM_CAMPOS = ["marcha", "angulo", "acel_lateral", "acel_x", "acel_y", "acel_z",
              "giro_x", "giro_y", "giro_z", "dx1", "dx2", "dx3", "dx4", "dx5",
              "stm_presion1", "stm_presion2"]
ECU_CAMPOS = ["rpm", "tps", "temp_refrigerante", "temp_aire", "temp_combustible",
              "presion_combustible", "presion_aceite", "bateria_v", "afr"]
GPS_CAMPOS = ["gps_lat", "gps_lon", "gps_vel_kmh"]


# =============================================================
# ESTADO COMPARTIDO (solo el GPS lo necesita: lo escribe su hilo,
# lo lee el loop principal en cada ciclo)
# =============================================================

_lock_gps = threading.Lock()
_estado_gps = {"gps_lat": None, "gps_lon": None, "gps_vel_kmh": None}

def guardar_gps(datos):
    with _lock_gps:
        _estado_gps.update(datos)

def leer_gps():
    with _lock_gps:
        return dict(_estado_gps)


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
        rpm  = struct.unpack_from('>h', payload, 8)[0]
        afr  = struct.unpack_from('B',  payload, 10)[0] / 10.0
        tps  = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp_motor = struct.unpack_from('>h', payload, 18)[0] / 10.0
        temp_aire  = struct.unpack_from('>h', payload, 20)[0] / 10.0
        vbat = struct.unpack_from('>h', payload, 22)[0] / 100.0
    except struct.error:
        return None
    return {
        "rpm": rpm, "tps": tps,
        "temp_refrigerante": temp_motor, "temp_aire": temp_aire, "temp_combustible": 0,
        # % TODO: "presion_combustible" y "presion_aceite" en realidad están
        # devolviendo afr y vbat (heredado de v15), no hay una presión real
        # decodificada todavía. Revisar offsets cuando tengas el protocolo
        # completo de la ECU a mano.
        "presion_combustible": 0,
        "presion_aceite": 0,
        "afr":afr,
        "bateria_v": vbat,
    }

def leer_ecu(ser, pedido):
    """Pide una trama a la ECU y devuelve el dict decodificado, o None si no llegó a tiempo."""
    try:
        ser.reset_input_buffer()
        ser.write(pedido)
        data = ser.read(64)  # se corta solo por ECU_TIMEOUT_S (timeout del puerto)
    except serial.SerialException:
        return None

    ini = data.find(ECU_HEADER)
    if ini == -1:
        return None
    fin = data.find(ECU_FOOTER, ini)
    if fin == -1:
        return None
    trama = data[ini:fin + 2]
    if len(trama) <= 12 or trama[2] != 180:
        return None
    return _decodificar_ecu(trama[10:-2])


# =============================================================
# STM32 — lectura SPI
# =============================================================

def leer_stm32(spi):
    """
    Escanea byte a byte buscando el sync (0xAA) y arma el paquete de 65 bytes.
    Hace falta el escaneo porque el maestro (Pi) no está sincronizado en fase
    con el índice del buffer circular del STM32 (ver ISR __irq_spi1 en el .ino).
    """
    for _ in range(STM32_TAM_PKT * 2):
        b = spi.xfer2([0x00])[0]
        if b == STM32_SYNC:
            resto = bytes(spi.xfer2([0x00] * (STM32_TAM_PKT - 1)))
            raw = bytes([STM32_SYNC]) + resto
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
# GPS/velocidad — llegan por CAN RX desde la pantalla (bidireccional)
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

def hilo_gps(bus_rx, stop):
    """Escucha el CAN RX de la pantalla y guarda el último valor recibido."""
    print("[GPS] Escuchando posición/velocidad de la pantalla...")
    while not stop.is_set():
        try:
            msg = bus_rx.recv(timeout=1.0)
        except can.CanError as e:
            print(f"[GPS] error de bus: {e}")
            time.sleep(0.5)
            continue
        if msg is None:
            continue
        decoder = GPS_DECODIFICADORES.get(msg.arbitration_id)
        if decoder:
            guardar_gps(decoder(bytes(msg.data)))
    print("[GPS] Hilo detenido.")


# =============================================================
# CAN TX — envío de datos a la pantalla del cockpit (corre en el loop
# principal, a 25 Hz, para no perder continuidad de datos en pantalla)
# =============================================================

def _empaquetar_senal(payload, valor, sig):
    raw = int((valor + sig['add']) * sig['div'] * sig['mult']) & sig['mask']
    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']]     = (raw >> 8) & 0xFF
        payload[sig['off'] + 1] =  raw       & 0xFF
    return payload

def enviar_can(bus_tx, ecu, marcha):
    def enviar(can_id, señales):
        payload = [0] * 8
        for valor, sig in señales:
            _empaquetar_senal(payload, valor if valor is not None else 0, sig)
        bus_tx.send(can.Message(arbitration_id=can_id, data=payload, is_extended_id=False))

    enviar(CAN_ID_MOTOR,       [(ecu['rpm'], SIG_RPM), (ecu['tps'], SIG_TPS)])
    enviar(CAN_ID_TEMPS,       [(ecu['temp_refrigerante'], SIG_TEMP_REFRIG),
                                 (ecu['temp_aire'],       SIG_TEMP_AIRE),
                                 (ecu['temp_combustible'],  SIG_TEMP_COMB),
                                 (ecu['bateria_v'],         SIG_BATERIA)])
    enviar(CAN_ID_PRES_COMB,   [(ecu['presion_combustible'], SIG_PRES_COMB)])
    enviar(CAN_ID_PRES_ACEITE, [(ecu['presion_aceite'],      SIG_PRES_ACEITE)])
    enviar(CAN_ID_MARCHA,      [(marcha, SIG_MARCHA)])


# =============================================================
# CSV — se escribe en un hilo aparte para que un fsync lento en la SD
# nunca retrase el envío por CAN ni la lectura de sensores
# =============================================================

def hilo_csv(cola, csv_file, csv_writer, stop):
    ultimo_fsync = time.monotonic()
    while not stop.is_set() or not cola.empty():
        try:
            fila = cola.get(timeout=0.2)
        except queue.Empty:
            continue
        csv_writer.writerow(fila)
        csv_file.flush()
        if time.monotonic() - ultimo_fsync >= FSYNC_CADA_S:
            try:
                os.fsync(csv_file.fileno())
            except Exception as e:
                print(f"[CSV] error en fsync: {e}")
            ultimo_fsync = time.monotonic()
    try:
        os.fsync(csv_file.fileno())
    except Exception:
        pass
    print("[CSV] Hilo detenido, archivo sincronizado.")


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

    print(f"[ECU] Abriendo {ECU_PUERTO}...")
    ser = serial.Serial(ECU_PUERTO, ECU_BAUDRATE, timeout=ECU_TIMEOUT_S)
    ser.write(_armar_pedido_ecu([0, 0, 0]))  # handshake
    time.sleep(0.05)
    ser.reset_input_buffer()

    print(f"[CAN] Abriendo {CAN_CANAL} (TX a pantalla)...")
    bus_tx = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan')
    print(f"[CAN] Abriendo {CAN_CANAL} (RX de pantalla)...")
    bus_rx = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan')

    csv_file = open(CSV_NOMBRE, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNAS)
    csv_writer.writeheader()
    csv_file.flush()
    os.fsync(csv_file.fileno())
    print(f"[CSV] {CSV_NOMBRE}")

    stop = threading.Event()
    cola_csv = queue.Queue(maxsize=500)  # ~20 s de margen a 25 Hz si la SD se atrasa

    hilos = [
        threading.Thread(target=hilo_gps, args=(bus_rx, stop), daemon=True),
        threading.Thread(target=hilo_csv, args=(cola_csv, csv_file, csv_writer, stop), daemon=True),
    ]
    for h in hilos:
        h.start()

    pedido_ecu = _armar_pedido_ecu([6, 0, 0])
    ultima_marcha = 0
    t_inicio = time.monotonic()
    n = 0
    n_csv_perdidas = 0

    try:
        while True:
            t0 = time.monotonic()

            try:
                ecu = leer_ecu(ser, pedido_ecu)
            except Exception as e:
                print(f"[ECU] error: {e}")
                ecu = None

            try:
                stm = leer_stm32(spi)
            except Exception as e:
                print(f"[SPI] error: {e}")
                stm = None

            if stm is not None:
                ultima_marcha = stm["marcha"]

            # --- Lo más importante: la pantalla nunca se queda sin datos ---
            if ecu is not None:
                try:
                    enviar_can(bus_tx, ecu, ultima_marcha)
                except can.CanError as e:
                    print(f"[CAN] error de bus: {e}")

            gps = leer_gps()

            fila = {"tiempo_pc_ms": round((time.monotonic() - t_inicio) * 1000, 3)}
            fila.update({k: None for k in STM_CAMPOS})
            fila.update({k: None for k in ECU_CAMPOS})
            fila.update(gps)
            if stm is not None:
                fila.update(stm)
            if ecu is not None:
                fila.update(ecu)

            try:
                cola_csv.put_nowait(fila)
            except queue.Full:
                n_csv_perdidas += 1  # la SD está más lenta que 25 Hz; se prioriza CAN/sensores

            n += 1
            if n % 25 == 0:  # un print por segundo aprox
                print(f"[{n}] marcha={ultima_marcha}  rpm={fila['rpm']}  tps={fila['tps']}  "
                      f"vel={fila['gps_vel_kmh']}  ecu={'OK' if ecu else '--'}  "
                      f"stm={'OK' if stm else '--'}  csv_perdidas={n_csv_perdidas}")

            t_espera = CICLO_S - (time.monotonic() - t0)
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print("\n[OK] Deteniendo...")

    finally:
        stop.set()
        for h in hilos:
            h.join(timeout=3.0)
        spi.close()
        ser.close()
        bus_tx.shutdown()
        bus_rx.shutdown()
        csv_file.close()
        print(f"[CSV] Guardado en {CSV_NOMBRE}")


if __name__ == "__main__":
    main()
