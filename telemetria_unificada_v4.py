#!/usr/bin/env python3
"""
=============================================================
  TELEMETRÍA UNIFICADA — Raspberry Pi
=============================================================
  - Lee paquete SPI del STM32 (45 bytes, sin checksum)
  - Lee parámetros ECU (RPM, TPS, presiones, temps)
  - Lee GPS desde CAN (0x680, 0x681, 0x682)
  - Guarda todo en CSV
  - Manda datos a pantalla por CAN

  Estructura paquete SPI STM32 (45 bytes):
    [0]        → 0xAA  sync byte
    [1..4]     → angulo         float  [°]
    [5..8]     → acel_lateral   float  [g]
    [9..12]    → dx1            float  [mm]
    [13..16]   → dx2            float  [mm]
    [17..20]   → dx3            float  [mm]
    [21..24]   → dx4            float  [mm]
    [25..28]   → dx5            float  [mm]
    [29..32]   → presion1       float
    [33..36]   → presion2       float
    [37..40]   → cambio         uint32
    [41..44]   → tiempo_us      uint32
=============================================================
"""

import can
import spidev
import serial
import struct
import csv
import time
import os
import threading
from datetime import datetime


# ============================================================
#                  CONFIGURACIÓN CAN / PANTALLA
# ============================================================

ID_MOTOR      = 0x640
ID_FUEL_PRESS = 0x641
ID_OIL_PRESS  = 0x644
ID_TEMP       = 0x649
ID_GEAR       = 0x64D

# IDs GPS (solo lectura)
ID_GPS_LAT_LON = 0x680
ID_GPS_TIME    = 0x681
ID_GPS_DATE    = 0x682

LEN_16BIT  = 2
LEN_8BIT   = 1

MASK_16BIT = 0xFFFF
MASK_8BIT  = 0xFF
MASK_4BIT  = 0x0F

MULT_BASE = 1
MULT_TEMP = 1
DIV_BASE  = 1
ADD_BASE  = 0
ADD_TEMP  = 40

SIG_RPM        = {'off': 0, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': MULT_BASE, 'div': 2,    'add': ADD_BASE}
SIG_TPS        = {'off': 6, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': MULT_BASE, 'div': 1000, 'add': ADD_BASE}
SIG_FUEL_PRESS = {'off': 4, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': 1000,      'div': 1,    'add': ADD_BASE}
SIG_OIL_PRESS  = {'off': 6, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': 1000,      'div': 1,    'add': ADD_BASE}
SIG_COOLANT    = {'off': 0, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_OIL        = {'off': 1, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_FUEL       = {'off': 2, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_GEAR       = {'off': 6, 'len': LEN_8BIT,  'mask': MASK_4BIT,  'mult': MULT_BASE, 'div': DIV_BASE, 'add': ADD_BASE}

CAN_CHANNEL = 'can0'
BITRATE     = 1_000_000
SEND_DELAY  = 0.5   # segundos entre envíos CAN


# ============================================================
#                  CONFIGURACIÓN ECU (Serial)
# ============================================================

ECU_PORT   = "/dev/ttyUSB0"
ECU_BAUD   = 115200
ECU_PERIOD = 0.04   # 25 Hz — igual que DAQ_mejora_de_latencia


# ============================================================
#                  CONFIGURACIÓN SPI / STM32
# ============================================================

SPI_BUS      = 1
SPI_DEVICE   = 0
SPI_SPEED_HZ = 1_000_000
SPI_MODE     = 0b00

PERIODO_MS   = 20
PERIODO_S    = PERIODO_MS / 1000.0

SYNC_BYTE    = 0xAA
PACKET_SIZE  = 45          # 1 sync + 9 floats×4 + 2 uint32×4
MAX_BUSQUEDA = PACKET_SIZE * 2

# Formato struct: 9 floats + 2 uint32, little-endian, a partir del byte 1
STRUCT_FMT   = '<9fII'

# "estricto" | "sync" | "ninguno"
MODO_VALIDACION = "sync"


# ============================================================
#                         CSV
# ============================================================

TIMESTAMP_SESION = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_DIR      = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
CSV_FILENAME = os.path.join(CSV_DIR, f"sesion_{TIMESTAMP_SESION}.csv")

CSV_HEADER = [
    # tiempo
    "tiempo_pc_ms",
    "tiempo_stm_us",
    # STM32
    "angulo_deg",
    "acel_lateral_g",
    "dx1_mm", "dx2_mm", "dx3_mm", "dx4_mm", "dx5_mm",
    "presion1",
    "presion2",
    "cambio",
    # ECU
    "rpm", "tps", "coolant", "oil_temp", "fuel_temp",
    "fuel_press_ecu", "oil_press_ecu",
    # GPS
    "gps_utc",
    "gps_lat",
    "gps_lon",
    "gps_speed_kmh",
    "gps_altitude_m",
    "gps_course_deg",
    "gps_satellites",
    "gps_valid",
    "gps_date",
]


# ============================================================
#         ESTADO COMPARTIDO ENTRE HILOS
# ============================================================

# --- Marcha (STM32 → hilo CAN salida) ---
_lock_gear   = threading.Lock()
_gear_actual = 0

def set_gear(valor: int):
    global _gear_actual
    with _lock_gear:
        _gear_actual = int(valor)

def get_gear() -> int:
    with _lock_gear:
        return _gear_actual


# --- GPS (hilo lector CAN → hilo principal) ---
_lock_gps = threading.Lock()
_gps_data = {
    "gps_utc":        "",
    "gps_lat":        None,
    "gps_lon":        None,
    "gps_speed_kmh":  None,
    "gps_altitude_m": None,
    "gps_course_deg": None,
    "gps_satellites": None,
    "gps_valid":      "",
    "gps_date":       "",
}

def set_gps(datos: dict):
    with _lock_gps:
        _gps_data.update(datos)

def get_gps() -> dict:
    with _lock_gps:
        return dict(_gps_data)


# --- ECU (hilo serial → hilo principal) ---
_lock_ecu = threading.Lock()
_ecu_data = {
    "rpm":        None,
    "tps":        None,
    "coolant":    None,
    "oil_temp":   None,
    "fuel_temp":  None,
    "fuel_press": None,
    "oil_press":  None,
}

def set_ecu(datos: dict):
    with _lock_ecu:
        _ecu_data.update(datos)

def get_ecu() -> dict:
    with _lock_ecu:
        return dict(_ecu_data)


# ============================================================
#               HILO ECU (lectura Serial)
# ============================================================

def _calcular_checksum_ecu(trama):
    total = sum(trama) & 0xFFFFFFFF
    return struct.pack('<I', total)

def _armar_trama_ecu(cmd_bytes, payload=b''):
    header = b'\xFF\x7F'
    footer = b'\x7F\xFF'
    n_bytes_val = 10 + len(payload)
    componentes = (
        header + bytes([cmd_bytes[0]]) + bytes([cmd_bytes[1]]) +
        bytes([cmd_bytes[2]]) + bytes([n_bytes_val]) + payload + footer
    )
    chk = _calcular_checksum_ecu(componentes)
    trama = bytearray()
    trama += header + bytes([cmd_bytes[0]]) + bytes([cmd_bytes[1]])
    trama += chk
    trama += bytes([cmd_bytes[2]]) + bytes([n_bytes_val]) + payload + footer
    return trama

def _decodificar_payload_ecu(payload):
    """
    Decodifica el payload de respuesta de la ECU (big-endian).
    Mismos offsets y factores que DAQ_mejora_de_latencia.py.
    Devuelve dict con los valores, o None si hay error de estructura.

    Offsets conocidos del protocolo ECU:
      [8..9]   → RPM        (int16 BE, valor directo)
      [10]     → AFR        (uint8,  /10.0)
      [14..15] → TPS        (int16 BE, /10.0  → %)
      [18..19] → Temp motor (int16 BE, /10.0  → °C)
      [22..23] → Vbat       (int16 BE, /100.0 → V)

    ⚠️  Los offsets de fuel_press y oil_press aún no están confirmados.
        Están marcados con TODO para ajustar cuando se disponga del mapa
        completo de la ECU.
    """
    try:
        rpm        = struct.unpack_from('>h', payload,  8)[0]
        afr        = struct.unpack_from('B',  payload, 10)[0] / 10.0   # TODO: ¿es fuel_press?
        tps        = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp_motor = struct.unpack_from('>h', payload, 18)[0] / 10.0
        vbat       = struct.unpack_from('>h', payload, 22)[0] / 100.0  # TODO: ¿es oil_press?
        return {
            "rpm":        rpm,
            "tps":        tps,
            "coolant":    temp_motor,   # TODO: separar coolant/oil/fuel si hay offsets distintos
            "oil_temp":   temp_motor,
            "fuel_temp":  temp_motor,
            "fuel_press": afr,          # TODO: ajustar offset real de presión combustible
            "oil_press":  vbat,         # TODO: ajustar offset real de presión aceite
        }
    except struct.error:
        return None

def hilo_ecu_serial(stop_event: threading.Event):
    """
    Hilo independiente que lee la ECU por serial a 25 Hz
    y actualiza el estado compartido _ecu_data.
    Protocolo idéntico al de DAQ_mejora_de_latencia.py.
    """
    try:
        ser = serial.Serial(ECU_PORT, ECU_BAUD, timeout=0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[ECU] ERROR abriendo {ECU_PORT}: {e}")
        print(f"[ECU] Los valores de ECU quedarán en None.")
        return

    print(f"[ECU] Puerto abierto: {ECU_PORT} @ {ECU_BAUD} baud")

    # Handshake inicial
    try:
        ser.write(_armar_trama_ecu(bytes([0, 0, 0])))
        time.sleep(0.05)
        ser.reset_input_buffer()
        print("[ECU] Handshake enviado OK.")
    except Exception as e:
        print(f"[ECU] Error en handshake: {e}")

    packet_req = _armar_trama_ecu(bytes([6, 0, 0]))
    buffer_rx  = bytearray()
    last_req   = 0.0

    HEADER = b'\xFF\x7F'
    FOOTER = b'\x7F\xFF'

    try:
        while not stop_event.is_set():
            now = time.perf_counter()

            # Petición periódica a 25 Hz
            if now - last_req >= ECU_PERIOD:
                ser.write(packet_req)
                last_req = now

            # Lectura no bloqueante
            n = ser.in_waiting
            if n:
                buffer_rx.extend(ser.read(n))

            # Procesar tramas completas en el buffer
            while HEADER in buffer_rx and FOOTER in buffer_rx:
                si = buffer_rx.find(HEADER)
                ei = buffer_rx.find(FOOTER, si)
                if si == -1 or ei == -1:
                    break
                trama = buffer_rx[si: ei + 2]
                buffer_rx = buffer_rx[ei + 2:]

                # cmd byte [2] == 180 indica respuesta de datos online
                if len(trama) > 12 and trama[2] == 180:
                    payload = trama[10:-2]
                    datos = _decodificar_payload_ecu(payload)
                    if datos:
                        set_ecu(datos)

            time.sleep(0.001)

    except Exception as e:
        print(f"[ECU] Error en hilo: {e}")
    finally:
        ser.close()
        print("[ECU] Puerto serial cerrado.")



# ============================================================
#               DECODIFICADORES GPS (CAN)
# ============================================================

def _decode_int32_be(data, offset):
    return struct.unpack_from('>i', data, offset)[0]

def _decode_uint16_be(data, offset):
    return struct.unpack_from('>H', data, offset)[0]

def _decode_int16_be(data, offset):
    return struct.unpack_from('>h', data, offset)[0]


def decode_0x680(data):
    lat = _decode_int32_be(data, 0) * 1e-7
    lon = _decode_int32_be(data, 4) * 1e-7
    return {"gps_lat": lat, "gps_lon": lon}


def decode_0x681(data):
    word_hi  = _decode_uint16_be(data, 0)
    word_lo  = _decode_uint16_be(data, 2)
    total_s  = (word_hi * 60_000 + word_lo) / 1000.0
    hh = int(total_s // 3600) % 24
    mm = int((total_s % 3600) // 60)
    ss = total_s % 60
    return {
        "gps_utc":        f"{hh:02d}:{mm:02d}:{ss:06.3f}",
        "gps_speed_kmh":  data[5] * 0.1,
        "gps_altitude_m": _decode_uint16_be(data, 6) * 0.1,
    }


def decode_0x682(data):
    raw24  = (data[0] << 16) | (data[1] << 8) | data[2]
    di     = int(raw24 / 256.0)
    dd, mo, yy = di // 10000, (di % 10000) // 100, di % 100
    return {
        "gps_date":       f"{dd:02d}/{mo:02d}/{yy:02d}",
        "gps_valid":      chr(data[3]) if 32 <= data[3] <= 126 else "?",
        "gps_course_deg": _decode_int16_be(data, 4) * 0.1,
        "gps_satellites": data[7],
    }


GPS_DECODERS = {
    ID_GPS_LAT_LON: decode_0x680,
    ID_GPS_TIME:    decode_0x681,
    ID_GPS_DATE:    decode_0x682,
}

# IDs que el hilo CAN de salida NO debe filtrar al leer
GPS_IDS = set(GPS_DECODERS.keys())


# ============================================================
#               FUNCIONES CAN / PANTALLA  (salida)
# ============================================================

def preparar_payload(payload, valor, sig):
    raw = int((valor + sig['add']) * sig['div'] * sig['mult'])
    raw &= sig['mask']
    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']]     = (raw >> 8) & 0xFF
        payload[sig['off'] + 1] =  raw       & 0xFF
    return payload


def hilo_can_salida(ecu_ignorado, stop_event: threading.Event):
    """
    Hilo independiente que manda mensajes CAN a la pantalla.
    Los datos de ECU se leen en tiempo real con get_ecu().
    La marcha se toma siempre del último valor leído del STM32.
    """
    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
    except OSError:
        print(f"[CAN-TX] ERROR: {CAN_CHANNEL} no encontrado.")
        print(f"         Ejecuta: sudo ip link set {CAN_CHANNEL} up type can bitrate {BITRATE}")
        stop_event.set()
        return

    print(f"[CAN-TX] Bus abierto en {CAN_CHANNEL} @ {BITRATE/1000:.0f} kbps")

    _ECU_DEFAULTS = {"rpm": 0, "tps": 0, "coolant": 0, "oil_temp": 0,
                     "fuel_temp": 0, "fuel_press": 0, "oil_press": 0}

    try:
        while not stop_event.is_set():
            gear = get_gear()   # marcha real del STM32
            ecu  = {**_ECU_DEFAULTS, **{k: v for k, v in get_ecu().items() if v is not None}}

            # 0x640: RPM + TPS
            p640 = [0] * 8
            p640 = preparar_payload(p640, ecu['rpm'],  SIG_RPM)
            p640 = preparar_payload(p640, ecu['tps'],  SIG_TPS)
            bus.send(can.Message(arbitration_id=ID_MOTOR,
                                 data=p640, is_extended_id=False))

            # 0x649: Temperaturas
            p649 = [0] * 8
            p649 = preparar_payload(p649, ecu['coolant'],  SIG_COOLANT)
            p649 = preparar_payload(p649, ecu['oil_temp'], SIG_OIL)
            p649 = preparar_payload(p649, ecu['fuel_temp'],SIG_FUEL)
            bus.send(can.Message(arbitration_id=ID_TEMP,
                                 data=p649, is_extended_id=False))

            # 0x641: Presión combustible (ECU)
            p641 = [0] * 8
            p641 = preparar_payload(p641, ecu['fuel_press'], SIG_FUEL_PRESS)
            bus.send(can.Message(arbitration_id=ID_FUEL_PRESS,
                                 data=p641, is_extended_id=False))

            # 0x644: Presión aceite (ECU)
            p644 = [0] * 8
            p644 = preparar_payload(p644, ecu['oil_press'], SIG_OIL_PRESS)
            bus.send(can.Message(arbitration_id=ID_OIL_PRESS,
                                 data=p644, is_extended_id=False))

            # 0x64D: Marcha (viene del STM32)
            p64D = [0] * 8
            p64D = preparar_payload(p64D, gear, SIG_GEAR)
            bus.send(can.Message(arbitration_id=ID_GEAR,
                                 data=p64D, is_extended_id=False))

            time.sleep(SEND_DELAY)

    except Exception as e:
        print(f"[CAN-TX] Error durante envío: {e}")
    finally:
        bus.shutdown()
        print("[CAN-TX] Bus cerrado.")


# ============================================================
#               HILO GPS  (lectura CAN)
# ============================================================

def hilo_can_gps(stop_event: threading.Event):
    """
    Hilo independiente que escucha los IDs GPS (0x680/681/682)
    y actualiza el estado compartido _gps_data.
    Usa su propio Bus para no interferir con el de TX.
    """
    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
    except OSError:
        print(f"[CAN-GPS] ERROR: {CAN_CHANNEL} no encontrado — GPS deshabilitado.")
        return

    print(f"[CAN-GPS] Escuchando GPS en {CAN_CHANNEL}")

    try:
        while not stop_event.is_set():
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            decoder = GPS_DECODERS.get(msg.arbitration_id)
            if decoder is None:
                continue
            parcial = decoder(bytes(msg.data))
            set_gps(parcial)
    except Exception as e:
        print(f"[CAN-GPS] Error: {e}")
    finally:
        bus.shutdown()
        print("[CAN-GPS] Bus cerrado.")


# ============================================================
#               FUNCIONES SPI / STM32
# ============================================================

def leer_byte(spi: spidev.SpiDev) -> int:
    return spi.xfer2([0x00])[0]


def leer_paquete_sincronizado(spi: spidev.SpiDev) -> tuple:
    """
    Busca 0xAA y luego lee los 44 bytes restantes de un tirón.
    Devuelve (raw_bytes | None, bytes_descartados).
    """
    descartados = 0
    for _ in range(MAX_BUSQUEDA):
        b = leer_byte(spi)
        if b == SYNC_BYTE:
            resto = bytes(spi.xfer2([0x00] * (PACKET_SIZE - 1)))
            return bytes([SYNC_BYTE]) + resto, descartados
        descartados += 1
    return None, descartados


def parsear_paquete(raw: bytes) -> tuple:
    """
    Parsea los 45 bytes ya alineados.
    Devuelve (datos_dict, estado_str).
    """
    if len(raw) != PACKET_SIZE or raw[0] != SYNC_BYTE:
        return None, "frame_invalido"

    try:
        (angulo, acel,
         dx1, dx2, dx3, dx4, dx5,
         presion1, presion2,
         cambio, tiempo_us) = struct.unpack_from(STRUCT_FMT, raw, 1)
    except struct.error as e:
        return None, f"unpack_error: {e}"

    datos = {
        "angulo_deg":     angulo,
        "acel_lateral_g": acel,
        "dx1_mm":         dx1,
        "dx2_mm":         dx2,
        "dx3_mm":         dx3,
        "dx4_mm":         dx4,
        "dx5_mm":         dx5,
        "presion1":       presion1,
        "presion2":       presion2,
        "cambio":         cambio,
        "tiempo_stm_us":  tiempo_us,
    }
    return datos, "ok"


# ============================================================
#                   DISPLAY EN CONSOLA
# ============================================================

VERDE    = "\033[92m"
AMARILLO = "\033[93m"
ROJO     = "\033[91m"
CYAN     = "\033[96m"
GRIS     = "\033[90m"
RESET    = "\033[0m"
BOLD     = "\033[1m"
CLEAR    = "\033[2J\033[H"

MARCHAS = {0: "N", 1: "1ª", 2: "2ª", 3: "3ª", 4: "4ª", 5: "5ª", 6: "6ª"}


def mostrar_consola(stm: dict, ecu: dict, gps: dict,
                    t_pc_ms: float, n_muestra: int,
                    n_errores: int, n_resync: int):

    tasa_error = (n_errores / n_muestra * 100) if n_muestra > 0 else 0.0
    marcha_str = MARCHAS.get(stm["cambio"], str(stm["cambio"]))

    def fmt(val, decimals=1, unit=""):
        if val is None:
            return f"{'---':>10}"
        return f"{val:>10.{decimals}f}"

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*56}{RESET}")
    print(f"{BOLD}{CYAN}   TELEMETRÍA UNIFICADA  —  STM32 + ECU + GPS → CAN{RESET}")
    print(f"{CYAN}{'='*56}{RESET}")
    print(f"\n  {BOLD}Muestra #{n_muestra:>6}{RESET}   "
          f"t_PC={t_pc_ms:>10.1f} ms   "
          f"t_STM={stm['tiempo_stm_us']:>10} µs")

    print(f"\n  {'─'*52}")
    print(f"  {BOLD}STM32{RESET}")
    print(f"  {'─'*52}")
    print(f"  {'Marcha':<24}  {marcha_str:>10}")
    print(f"  {'Ángulo volante':<24}  {stm['angulo_deg']:>10.2f}  °")
    print(f"  {'Acel. lateral':<24}  {stm['acel_lateral_g']:>10.3f}  g")
    print(f"  {'Suspensión 1':<24}  {stm['dx1_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 2':<24}  {stm['dx2_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 3':<24}  {stm['dx3_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 4':<24}  {stm['dx4_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 5':<24}  {stm['dx5_mm']:>10.1f}  mm")
    print(f"  {'Presión 1':<24}  {stm['presion1']:>10.2f}")
    print(f"  {'Presión 2':<24}  {stm['presion2']:>10.2f}")

    print(f"\n  {'─'*52}")
    print(f"  {BOLD}ECU  →  CAN (pantalla){RESET}")
    print(f"  {'─'*52}")
    print(f"  {'RPM':<24}  {ecu['rpm']:>10}  rpm")
    print(f"  {'TPS':<24}  {ecu['tps']:>10}  %")
    print(f"  {'Temp. refrigerante':<24}  {ecu['coolant']:>10}  °C")
    print(f"  {'Temp. aceite':<24}  {ecu['oil_temp']:>10}  °C")
    print(f"  {'Temp. combustible':<24}  {ecu['fuel_temp']:>10}  °C")
    print(f"  {'Presión combustible':<24}  {ecu['fuel_press']:>10}  bar")
    print(f"  {'Presión aceite':<24}  {ecu['oil_press']:>10}  bar")

    print(f"\n  {'─'*52}")
    print(f"  {BOLD}GPS  ←  CAN (0x680/681/682){RESET}")
    print(f"  {'─'*52}")
    print(f"  {'UTC':<24}  {gps['gps_utc'] or '---':>10}")
    print(f"  {'Fecha':<24}  {gps['gps_date'] or '---':>10}")
    print(f"  {'Latitud':<24}  {fmt(gps['gps_lat'], 7)}  °")
    print(f"  {'Longitud':<24}  {fmt(gps['gps_lon'], 7)}  °")
    print(f"  {'Velocidad':<24}  {fmt(gps['gps_speed_kmh'], 1)}  km/h")
    print(f"  {'Altitud':<24}  {fmt(gps['gps_altitude_m'], 1)}  m")
    print(f"  {'Rumbo':<24}  {fmt(gps['gps_course_deg'], 1)}  °")
    print(f"  {'Satélites':<24}  {gps['gps_satellites'] if gps['gps_satellites'] is not None else '---':>10}")
    print(f"  {'Válido':<24}  {gps['gps_valid'] or '---':>10}")

    color_err    = VERDE if tasa_error < 1.0 else (AMARILLO if tasa_error < 5.0 else ROJO)
    color_resync = VERDE if n_resync == 0 else AMARILLO

    print(f"\n  Frames inválidos: {color_err}{n_errores} ({tasa_error:.1f}%){RESET}")
    print(f"  Bytes resincronizados: {color_resync}{n_resync}{RESET}"
          f"{GRIS}  (>0 = hubo desfase){RESET}")
    print(f"\n  {GRIS}CSV → {CSV_FILENAME}{RESET}")
    print(f"  {CYAN}Ctrl+C para detener{RESET}\n")


# ============================================================
#                         MAIN
# ============================================================

def main():

    os.makedirs(CSV_DIR, exist_ok=True)

    stop_event = threading.Event()

    # Hilo ECU serial (lectura real desde la ECU)
    t_ecu = threading.Thread(
        target=hilo_ecu_serial,
        args=(stop_event,),
        daemon=True,
    )
    t_ecu.start()

    # Hilo CAN salida (pantalla) — usa get_ecu() en cada ciclo
    t_can_tx = threading.Thread(
        target=hilo_can_salida,
        args=(None, stop_event),   # ecu=None; el hilo llama get_ecu() internamente
        daemon=True,
    )
    t_can_tx.start()

    # Hilo CAN entrada (GPS)
    t_can_gps = threading.Thread(
        target=hilo_can_gps,
        args=(stop_event,),
        daemon=True,
    )
    t_can_gps.start()

    # Iniciar SPI
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz  = SPI_SPEED_HZ
    spi.mode          = SPI_MODE
    spi.bits_per_word = 8

    print(f"[SPI] bus={SPI_BUS}  device={SPI_DEVICE}  "
          f"velocidad={SPI_SPEED_HZ/1e6:.1f} MHz  modo={SPI_MODE}")
    print(f"[CSV] Guardando en: {CSV_FILENAME}")
    print(f"[..] Iniciando en 1 segundo...\n")
    time.sleep(1.0)

    csv_file   = open(CSV_FILENAME, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADER)
    csv_writer.writeheader()

    n_muestra = 0
    n_errores = 0
    n_resync  = 0
    t_inicio  = time.monotonic()

    try:
        while True:
            t_ciclo = time.monotonic()

            raw, descartados = leer_paquete_sincronizado(spi)
            if descartados > 0:
                n_resync += descartados

            t_pc_ms    = (time.monotonic() - t_inicio) * 1000.0
            n_muestra += 1

            if raw is None:
                n_errores += 1
                continue

            stm, estado = parsear_paquete(raw)

            if stm is None:
                n_errores += 1
                continue

            # Actualizar marcha para el hilo CAN salida
            set_gear(stm["cambio"])

            # Snapshot GPS y ECU actuales
            gps = get_gps()
            ecu = get_ecu()

            # Guardar en CSV
            fila = {
                "tiempo_pc_ms":   round(t_pc_ms, 3),
                "tiempo_stm_us":  stm["tiempo_stm_us"],
                "angulo_deg":     round(stm["angulo_deg"],     3),
                "acel_lateral_g": round(stm["acel_lateral_g"], 4),
                "dx1_mm":         round(stm["dx1_mm"],         2),
                "dx2_mm":         round(stm["dx2_mm"],         2),
                "dx3_mm":         round(stm["dx3_mm"],         2),
                "dx4_mm":         round(stm["dx4_mm"],         2),
                "dx5_mm":         round(stm["dx5_mm"],         2),
                "presion1":       round(stm["presion1"],       2),
                "presion2":       round(stm["presion2"],       2),
                "cambio":         stm["cambio"],
                "rpm":            ecu["rpm"],
                "tps":            ecu["tps"],
                "coolant":        ecu["coolant"],
                "oil_temp":       ecu["oil_temp"],
                "fuel_temp":      ecu["fuel_temp"],
                "fuel_press_ecu": ecu["fuel_press"],
                "oil_press_ecu":  ecu["oil_press"],
                # GPS
                "gps_utc":        gps["gps_utc"],
                "gps_lat":        gps["gps_lat"],
                "gps_lon":        gps["gps_lon"],
                "gps_speed_kmh":  gps["gps_speed_kmh"],
                "gps_altitude_m": gps["gps_altitude_m"],
                "gps_course_deg": gps["gps_course_deg"],
                "gps_satellites": gps["gps_satellites"],
                "gps_valid":      gps["gps_valid"],
                "gps_date":       gps["gps_date"],
            }
            csv_writer.writerow(fila)
            csv_file.flush()

            mostrar_consola(stm, ecu, gps, t_pc_ms, n_muestra, n_errores, n_resync)

            t_espera = PERIODO_S - (time.monotonic() - t_ciclo)
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print(f"\n\n{VERDE}[OK] Detenido por el usuario.{RESET}")

    finally:
        stop_event.set()
        t_ecu.join(timeout=2.0)
        t_can_tx.join(timeout=2.0)
        t_can_gps.join(timeout=2.0)
        spi.close()
        csv_file.close()

        duracion  = time.monotonic() - t_inicio
        aceptados = n_muestra - n_errores
        tasa_ok   = (aceptados / n_muestra * 100) if n_muestra > 0 else 0

        print(f"\n{'─'*52}")
        print(f"  RESUMEN DE SESIÓN")
        print(f"{'─'*52}")
        print(f"  Duración:              {duracion:.1f} s")
        print(f"  Muestras totales:      {n_muestra}")
        print(f"  Aceptados:             {aceptados}  ({tasa_ok:.1f}%)")
        print(f"  Frames inválidos:      {n_errores}")
        print(f"  Bytes resincronizados: {n_resync}")
        print(f"  CSV → {CSV_FILENAME}")
        print(f"{'─'*52}\n")


if __name__ == "__main__":
    main()
