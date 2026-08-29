#!/usr/bin/env python3
"""
TELEMETRÍA UNIFICADA — Raspberry Pi
Fuentes de datos:
  - STM32   → SPI       (marcha, ángulo, acel_lateral, IMU cruda ax/ay/az/gx/gy/gz, dx1..dx5, presiones STM, tiempo)
  - ECU     → Serial    (RPM, TPS, temperaturas, presiones, batería)
  - GPS     → CAN RX    (latitud, longitud, velocidad)
Salidas:
  - CSV     → archivo por sesión en /sesiones/
  - Pantalla cockpit → CAN TX cada 500 ms
  - Consola → display en tiempo real
"""

import can, spidev, serial, struct, csv, time, os, threading, traceback
from datetime import datetime


# =============================================================
# CONFIGURACIÓN — editá acá para cambiar puertos, IDs, timings
# =============================================================

FSYNC_CADA_S = 0.5   # fsync a disco como máximo cada X segundos (independiente de la
                      # cantidad de muestras: el loop real no siempre corre a 50 Hz exactos,
                      # así que basarse en tiempo da una ventana de pérdida predecible ante
                      # un corte de alimentación abrupto).

# --- SPI (STM32) ---
SPI_BUS        = 1
SPI_DEVICE     = 0
SPI_VELOCIDAD  = 1_000_000   # Hz
SPI_MODO       = 0b00
CICLO_STM32_S  = 0.020       # 50 Hz → cada 20 ms

STM32_SYNC     = 0xAA        # byte que marca inicio de paquete

# Estructura real del paquete armado en armarPaquete() del .ino (69 bytes):
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
#   [53..56] presion1      float   (medida por el STM32)
#   [57..60] presion2      float   (medida por el STM32)
#   [61..64] cambio        int32
#   [65..68] tiempo_us     uint32
STM32_TAM_PKT  = 69          # 1 sync + 15 floats (4B c/u) + 1 int32 + 1 uint32
STM32_FORMATO  = '<15fiI'    # little-endian: 15 floats, cambio (int32), tiempo_us (uint32)

# --- ECU (Serial) ---
ECU_PUERTO     = "/dev/ttyUSB0"
ECU_BAUDRATE   = 115200
ECU_PERIODO_S  = 0.04        # 25 Hz
ECU_RETRY_ESPERA = 3.0       # segundos entre reintentos de apertura del puerto
ECU_BUFFER_MAX       = 2048  # tope de bytes en buffer sin lograr sync
ECU_RESYNC_TIMEOUT_S = 2.0   # si no se arma un frame válido en este tiempo, se purga el buffer

# --- CAN ---
CAN_CANAL      = 'can0'
CAN_BITRATE    = 1_000_000
CAN_TX_PERIODO = 0.1         # segundos entre ciclos de envío a pantalla (antes 0.5 -> 10 Hz ahora)

# --- DEBUG ---
DEBUG_CAN_TX      = True     # imprime en consola cada envío a la pantalla
CAN_TX_INTERFRAME = 0.0005   # pausa entre cada bus.send() (antes 0.003, sobraba)
CAN_TX_RETRY_ESPERA = 2.0    # segundos a esperar antes de reintentar reconectar el bus
GPS_RETRY_ESPERA    = 3.0    # segundos entre reintentos de apertura del bus GPS
SPI_RETRY_ESPERA    = 2.0    # segundos entre reintentos de apertura del SPI (STM32)

# IDs CAN de salida (pantalla cockpit)
CAN_ID_MOTOR       = 0x640
CAN_ID_PRES_COMB   = 0x641
CAN_ID_PRES_ACEITE = 0x644
CAN_ID_TEMPS       = 0x649
CAN_ID_MARCHA      = 0x64D

# IDs CAN de entrada (GPS)
CAN_ID_GPS_LATLON  = 0x680
CAN_ID_GPS_TIEMPO  = 0x681
CAN_ID_GPS_FECHA   = 0x682

# --- CSV ---
CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
os.makedirs(CSV_DIR, exist_ok=True)

# Ejemplo: sesion_19-30-05.csv
CSV_NOMBRE = os.path.join(CSV_DIR, f"sesion_{datetime.now().strftime('%H-%M-%S')}.csv")



CSV_COLUMNAS = [
    "tiempo_pc_ms", "tiempo_stm_us",
    "marcha", "angulo", "acel_lateral",
    "acel_x", "acel_y", "acel_z",
    "giro_x", "giro_y", "giro_z",
    "dx1", "dx2", "dx3", "dx4", "dx5",
    "stm_presion1", "stm_presion2",
    "rpm", "tps",
    "temp_refrigerante", "temp_aceite", "temp_combustible",
    "presion_combustible", "presion_aceite",
    "bateria_v",
    "gps_lat", "gps_lon", "gps_vel_kmh",
]

# --- Señales CAN TX (offset, largo, máscara, multiplicador, divisor, offset_valor) ---
SIG_RPM          = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1,    'add': 0}
SIG_TPS          = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1000, 'add': 0}
SIG_PRES_COMB    = {'off': 4, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_PRES_ACEITE  = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_TEMP_REFRIG  = {'off': 0, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_ACEITE  = {'off': 1, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_COMB    = {'off': 2, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_MARCHA       = {'off': 6, 'len': 1, 'mask': 0x0F,   'mult': 1,    'div': 1,    'add': 0}
SIG_BATERIA      = {'off': 5, 'len': 1, 'mask': 0xFF, 'mult': 1,  'div': 10,    'add': 0}


# =============================================================
# ESTADO COMPARTIDO ENTRE HILOS (protegido con locks)
# =============================================================

_fsync_solicitado = threading.Event()

def hilo_fsync(csv_file, stop):
    """
    Hace el os.fsync() del CSV en un hilo aparte, para que un SD lento
    (fsync puede tardar cientos de ms) no bloquee el loop principal
    (lectura SPI + envío CAN a la pantalla).
    """
    while not stop.is_set():
        if _fsync_solicitado.wait(timeout=0.5):
            _fsync_solicitado.clear()
            try:
                os.fsync(csv_file.fileno())
            except Exception as e:
                print(f"[CSV] Error en fsync: {e}")


_lock_marcha = threading.Lock()
_marcha = 0

def guardar_marcha(valor):
    global _marcha
    with _lock_marcha:
        _marcha = int(valor)

def leer_marcha():
    with _lock_marcha:
        return _marcha


_lock_gps = threading.Lock()
_estado_gps = {"gps_lat": None, "gps_lon": None, "gps_vel_kmh": None}

def guardar_gps(datos):
    with _lock_gps:
        _estado_gps.update(datos)

def leer_gps():
    with _lock_gps:
        return dict(_estado_gps)


_lock_ecu = threading.Lock()
_estado_ecu = {
    "rpm": None, "tps": None,
    "temp_refrigerante": None, "temp_aceite": None, "temp_combustible": None,
    "presion_combustible": None, "presion_aceite": None,
    "bateria_v": None,
}

def guardar_ecu(datos):
    with _lock_ecu:
        _estado_ecu.update(datos)

def leer_ecu():
    with _lock_ecu:
        return dict(_estado_ecu)


# =============================================================
# ECU — protocolo serial propietario
# =============================================================

def _checksum_ecu(trama):
    return struct.pack('<I', sum(trama) & 0xFFFFFFFF)

def _armar_pedido_ecu(cmd):
    HEADER = b'\xFF\x7F'
    FOOTER = b'\x7F\xFF'
    n = 10 + 0
    base = HEADER + bytes([cmd[0], cmd[1], cmd[2], n]) + FOOTER
    chk  = _checksum_ecu(base)
    return bytearray(HEADER + bytes([cmd[0], cmd[1]]) + chk + bytes([cmd[2], n]) + FOOTER)

def _decodificar_ecu(payload):
    try:
        rpm       = struct.unpack_from('>h', payload,  8)[0]
        afr       = struct.unpack_from('B',  payload, 10)[0] / 10.0
        tps       = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp      = struct.unpack_from('>h', payload, 18)[0] / 10.0
        vbat      = struct.unpack_from('>h', payload, 22)[0] / 100.0
        return {
            "rpm": rpm, "tps": tps,
            "temp_refrigerante":  temp,
            "temp_aceite":        temp,
            "temp_combustible":   temp,
            "presion_combustible": afr,
            "presion_aceite":      vbat,
            "bateria_v":           vbat,
        }
    except struct.error:
        return None

def hilo_ecu(stop):
    HEADER = b'\xFF\x7F'
    FOOTER = b'\x7F\xFF'

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

    pedido   = _armar_pedido_ecu([6, 0, 0])
    buffer   = bytearray()
    ultimo   = 0.0
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
            while HEADER in buffer and FOOTER in buffer:
                ini = buffer.find(HEADER)
                fin = buffer.find(FOOTER, ini)
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
            # Si el buffer se nos fue de tamaño, o pasó mucho tiempo sin
            # poder armar un frame válido, purgamos y volvemos a esperar
            # un HEADER limpio en vez de arrastrar basura para siempre.
            if len(buffer) > ECU_BUFFER_MAX:
                print(f"[ECU] {AMARILLO}Buffer > {ECU_BUFFER_MAX} bytes sin sync, purgando...{RESET}")
                buffer.clear()
                ultimo_frame_ok = ahora
            elif (ahora - ultimo_frame_ok) > ECU_RESYNC_TIMEOUT_S and len(buffer) > 0:
                print(f"[ECU] {AMARILLO}Sin frames válidos hace {ECU_RESYNC_TIMEOUT_S:.0f}s, purgando buffer ({len(buffer)} bytes)...{RESET}")
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
            # Antes esto hacía "break" y mataba el hilo para siempre.
            # Ahora logueamos con traceback completo y seguimos vivos.
            print(f"[ECU] {ROJO}Error inesperado (se sigue intentando): {e}{RESET}")
            traceback.print_exc()
            buffer.clear()
            time.sleep(0.1)

    ser.close()
    print("[ECU] Puerto cerrado.")


# =============================================================
# GPS — decodificadores de mensajes CAN
# =============================================================

def _decodificar_gps_latlon(data):
    lat = struct.unpack_from('>i', data, 0)[0] * 1e-7
    lon = struct.unpack_from('>i', data, 4)[0] * 1e-7
    return {"gps_lat": lat, "gps_lon": lon}

def _decodificar_gps_tiempo(data):
    return {"gps_vel_kmh": data[5] * 0.1}

def _decodificar_gps_fecha(data):
    return {}

GPS_DECODIFICADORES = {
    CAN_ID_GPS_LATLON: _decodificar_gps_latlon,
    CAN_ID_GPS_TIEMPO: _decodificar_gps_tiempo,
    CAN_ID_GPS_FECHA:  _decodificar_gps_fecha,
}

def hilo_gps(stop):
    bus = None
    intentos = 0
    while bus is None and not stop.is_set():
        try:
            bus = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan')
        except OSError as e:
            intentos += 1
            if intentos == 1 or intentos % 10 == 0:
                print(f"[GPS] {CAN_CANAL} no disponible (intento {intentos}): {e} — reintentando cada {GPS_RETRY_ESPERA:.0f}s...")
            time.sleep(GPS_RETRY_ESPERA)

    if stop.is_set():
        return

    print(f"[GPS] Escuchando en {CAN_CANAL}")
    try:
        while not stop.is_set():
            try:
                msg = bus.recv(timeout=1.0)
            except can.CanError as e:
                print(f"[GPS] Error de bus (reintentando): {e}")
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            decoder = GPS_DECODIFICADORES.get(msg.arbitration_id)
            if decoder:
                guardar_gps(decoder(bytes(msg.data)))
    except Exception as e:
        print(f"[GPS] Error fatal: {e}")
    finally:
        bus.shutdown()
        print("[GPS] Bus cerrado.")


# =============================================================
# CAN TX — envío de datos a la pantalla del cockpit
# =============================================================

def _empaquetar_senal(payload, valor, sig):
    raw = int((valor + sig['add']) * sig['div'] * sig['mult']) & sig['mask']
    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']]     = (raw >> 8) & 0xFF
        payload[sig['off'] + 1] =  raw       & 0xFF
    return payload


def _abrir_bus_pantalla():
    """Abre (o reabre) el bus CAN para la pantalla. Devuelve None si no está disponible."""
    try:
        return can.interface.Bus(channel=CAN_CANAL, bustype='socketcan')
    except OSError as e:
        print(f"[CAN-TX] {CAN_CANAL} no disponible: {e}")
        print(f"         (Para activarlo: sudo ip link set {CAN_CANAL} up type can bitrate {CAN_BITRATE})")
        return None


def hilo_can_pantalla(_, stop):
    """
    Envía datos de ECU y marcha a la pantalla del cockpit.
    Diseñado para NO morir ante errores transitorios de bus (ENOBUFS, bus-off,
    glitches del adaptador): cualquier excepción de una iteración se loguea
    con traceback completo y el hilo sigue vivo. Si el bus queda en un estado
    no recuperable, se cierra y se reabre en vez de terminar el hilo.
    """
    ECU_DEFAULTS = {k: 0 for k in _estado_ecu}

    bus = _abrir_bus_pantalla()
    intentos = 0
    while bus is None and not stop.is_set():
        intentos += 1
        time.sleep(CAN_TX_RETRY_ESPERA)
        bus = _abrir_bus_pantalla()
        if bus is None and intentos % 10 == 0:
            print(f"[CAN-TX] Sigo sin poder abrir {CAN_CANAL} (intento {intentos})...")

    if bus is None:  # se pidió salir (stop) antes de conseguir el bus
        return

    print(f"[CAN-TX] Enviando en {CAN_CANAL} @ {CAN_BITRATE//1000} kbps "
          f"(periodo={CAN_TX_PERIODO*1000:.0f} ms)")

    def enviar(can_id, señales):
        payload = [0] * 8
        for valor, sig in señales:
            _empaquetar_senal(payload, valor, sig)
        bus.send(can.Message(arbitration_id=can_id, data=payload, is_extended_id=False))
        if DEBUG_CAN_TX:
            print(f"[CAN-TX] {GRIS}0x{can_id:03X}{RESET}  {bytes(payload).hex(' ')}")
        if CAN_TX_INTERFRAME > 0:
            time.sleep(CAN_TX_INTERFRAME)

    while not stop.is_set():
        try:
            ecu    = {**ECU_DEFAULTS, **{k: v for k, v in leer_ecu().items() if v is not None}}
            marcha = leer_marcha()

            if DEBUG_CAN_TX:
                sello = time.strftime("%H:%M:%S")
                print(f"\n[CAN-TX] {CYAN}--- {sello} · valores a enviar ---{RESET}")
                print(f"[CAN-TX]   rpm={ecu['rpm']}  tps={ecu['tps']}  "
                      f"temp_ref={ecu['temp_refrigerante']}  temp_ace={ecu['temp_aceite']}  "
                      f"temp_comb={ecu['temp_combustible']}  bateria={ecu['bateria_v']}  "
                      f"pres_comb={ecu['presion_combustible']}  pres_ace={ecu['presion_aceite']}  "
                      f"marcha={marcha}")

            enviar(CAN_ID_MOTOR,       [(ecu['rpm'], SIG_RPM),         (ecu['tps'], SIG_TPS)])
            enviar(CAN_ID_TEMPS,       [(ecu['temp_refrigerante'], SIG_TEMP_REFRIG),
                                        (ecu['temp_aceite'],       SIG_TEMP_ACEITE),
                                        (ecu['temp_combustible'],  SIG_TEMP_COMB),
                                        (ecu['bateria_v'],         SIG_BATERIA)])
            enviar(CAN_ID_PRES_COMB,   [(ecu['presion_combustible'], SIG_PRES_COMB)])
            enviar(CAN_ID_PRES_ACEITE, [(ecu['presion_aceite'],      SIG_PRES_ACEITE)])
            enviar(CAN_ID_MARCHA,      [(marcha,           SIG_MARCHA)])

            t_espera = CAN_TX_PERIODO - (len(
                [CAN_ID_MOTOR, CAN_ID_TEMPS, CAN_ID_PRES_COMB, CAN_ID_PRES_ACEITE, CAN_ID_MARCHA]
            ) * CAN_TX_INTERFRAME)
            if t_espera > 0:
                time.sleep(t_espera)

        except can.CanError as e:
            # Error "normal" del bus (ENOBUFS, arbitration lost, etc): no matamos el hilo.
            print(f"[CAN-TX] {ROJO}Error de bus (se sigue intentando): {e}{RESET}")
            time.sleep(0.05)

        except Exception as e:
            # Cualquier otra cosa (ej: OSError si la interfaz cayó, "Network is down"):
            # logueamos traceback completo y reabrimos el bus en vez de terminar el hilo.
            print(f"[CAN-TX] {ROJO}Error inesperado, reabriendo bus: {e}{RESET}")
            traceback.print_exc()
            try:
                bus.shutdown()
            except Exception:
                pass
            time.sleep(CAN_TX_RETRY_ESPERA)
            bus = _abrir_bus_pantalla()
            while bus is None and not stop.is_set():
                time.sleep(CAN_TX_RETRY_ESPERA)
                bus = _abrir_bus_pantalla()

    try:
        bus.shutdown()
    except Exception:
        pass
    print("[CAN-TX] Bus cerrado.")


# =============================================================
# STM32 — lectura SPI
# =============================================================

def _leer_paquete_stm32(spi):
    descartados = 0
    for _ in range(STM32_TAM_PKT * 2):
        b = spi.xfer2([0x00])[0]
        if b == STM32_SYNC:
            resto = bytes(spi.xfer2([0x00] * (STM32_TAM_PKT - 1)))
            return bytes([STM32_SYNC]) + resto, descartados
        descartados += 1
    return None, descartados

def _parsear_paquete_stm32(raw):
    """
    Decodifica el paquete de 69 bytes armado por armarPaquete() en el STM32:
      [0]      sync (0xAA)
      [1..4]   angulo        float
      [5..8]   acel_lateral  float
      [9..12]  ax            float  (IMU cruda)
      [13..16] ay            float  (IMU cruda)
      [17..20] az            float  (IMU cruda)
      [21..24] gx            float  (IMU cruda)
      [25..28] gy            float  (IMU cruda)
      [29..32] gz            float  (IMU cruda)
      [33..36] dx1           float
      [37..40] dx2           float
      [41..44] dx3           float
      [45..48] dx4           float
      [49..52] dx5           float
      [53..56] presion1      float  (medida en el STM32)
      [57..60] presion2      float  (medida en el STM32)
      [61..64] cambio        int32
      [65..68] tiempo_us     uint32
    """
    if len(raw) != STM32_TAM_PKT or raw[0] != STM32_SYNC:
        return None
    try:
        (angulo, acel_lateral,
         ax, ay, az, gx, gy, gz,
         dx1, dx2, dx3, dx4, dx5,
         presion1, presion2,
         cambio, t_us) = struct.unpack_from(STM32_FORMATO, raw, 1)
    except struct.error:
        return None
    return {
        "marcha":        cambio,
        "angulo":        angulo,
        "acel_lateral":  acel_lateral,
        "acel_x": ax, "acel_y": ay, "acel_z": az,
        "giro_x": gx, "giro_y": gy, "giro_z": gz,
        "dx1": dx1, "dx2": dx2, "dx3": dx3, "dx4": dx4, "dx5": dx5,
        "stm_presion1":  presion1,
        "stm_presion2":  presion2,
        "tiempo_stm_us": t_us,
    }


# =============================================================
# CONSOLA — display en tiempo real
# =============================================================

VERDE = "\033[92m"; AMARILLO = "\033[93m"; ROJO = "\033[91m"
CYAN  = "\033[96m"; GRIS     = "\033[90m"; RESET = "\033[0m"
BOLD  = "\033[1m";  CLEAR    = "\033[2J\033[H"
NOMBRES_MARCHA = {0: "N", 1: "1ª", 2: "2ª", 3: "3ª", 4: "4ª", 5: "5ª", 6: "6ª"}

def _fmt(val, dec=1):
    return f"{'---':>10}" if val is None else f"{val:>10.{dec}f}"

def mostrar_consola(stm, ecu, gps, t_ms, n_muestras, n_errores, n_resync):
    tasa_err  = (n_errores / n_muestras * 100) if n_muestras > 0 else 0.0
    marcha    = NOMBRES_MARCHA.get(stm["marcha"], str(stm["marcha"]))
    c_err     = VERDE if tasa_err < 1 else (AMARILLO if tasa_err < 5 else ROJO)
    c_resync  = VERDE if n_resync == 0 else AMARILLO

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*56}{RESET}")
    print(f"{BOLD}{CYAN}   TELEMETRÍA — STM32 + ECU + GPS{RESET}")
    print(f"{CYAN}{'='*56}{RESET}")
    print(f"\n  {BOLD}Muestra #{n_muestras:<6}{RESET}  t_PC={t_ms:.1f} ms  t_STM={stm['tiempo_stm_us']} µs\n")

    print(f"  {'Marcha':<22}  {marcha:>10}")
    print(f"  {'Ángulo volante':<22}  {_fmt(stm['angulo'], 1)}  °")
    print(f"  {'Acel. lateral':<22}  {_fmt(stm['acel_lateral'], 3)}  g")
    print(f"  {'Acel (X/Y/Z)':<22}  {_fmt(stm['acel_x'], 2)} / {_fmt(stm['acel_y'], 2)} / {_fmt(stm['acel_z'], 2)}  m/s²")
    print(f"  {'Giro (X/Y/Z)':<22}  {_fmt(stm['giro_x'], 2)} / {_fmt(stm['giro_y'], 2)} / {_fmt(stm['giro_z'], 2)}  rad/s")
    print(f"  {'dx1':<22}  {_fmt(stm['dx1'], 1)}  mm")
    print(f"  {'dx2':<22}  {_fmt(stm['dx2'], 1)}  mm")
    print(f"  {'dx3':<22}  {_fmt(stm['dx3'], 1)}  mm")
    print(f"  {'dx4':<22}  {_fmt(stm['dx4'], 1)}  mm")
    print(f"  {'dx5':<22}  {_fmt(stm['dx5'], 1)}  mm")
    print(f"  {'Presión 1 (STM)':<22}  {_fmt(stm['stm_presion1'], 1)}")
    print(f"  {'Presión 2 (STM)':<22}  {_fmt(stm['stm_presion2'], 1)}")

    print(f"  {'RPM':<22}  {ecu['rpm'] or '---':>10}  rpm")
    print(f"  {'TPS':<22}  {ecu['tps'] or '---':>10}  %")
    print(f"  {'Temp. refrigerante':<22}  {_fmt(ecu['temp_refrigerante'])}  °C")
    print(f"  {'Temp. aceite':<22}  {_fmt(ecu['temp_aceite'])}  °C")
    print(f"  {'Temp. combustible':<22}  {_fmt(ecu['temp_combustible'])}  °C")
    print(f"  {'Presión combustible':<22}  {_fmt(ecu['presion_combustible'])}  bar")
    print(f"  {'Presión aceite':<22}  {_fmt(ecu['presion_aceite'])}  bar")
    print(f"  {'Batería':<22}  {_fmt(ecu['bateria_v'], 2)}  V")

    print(f"  {'Latitud':<22}  {_fmt(gps['gps_lat'], 7)}  °")
    print(f"  {'Longitud':<22}  {_fmt(gps['gps_lon'], 7)}  °")
    print(f"  {'Velocidad':<22}  {_fmt(gps['gps_vel_kmh'])}  km/h")

    print(f"\n  Frames inválidos:      {c_err}{n_errores} ({tasa_err:.1f}%){RESET}")
    print(f"  Bytes resincronizados: {c_resync}{n_resync}{RESET}{GRIS}  (>0 = hubo desfase){RESET}")
    print(f"\n  {GRIS}CSV → {CSV_NOMBRE}{RESET}")
    print(f"  {CYAN}Ctrl+C para detener{RESET}\n")


# =============================================================
# MAIN
# =============================================================

def main():
    os.makedirs(CSV_DIR, exist_ok=True)
    stop = threading.Event()

    hilos = [
        threading.Thread(target=hilo_ecu,          args=(stop,), daemon=True),
        threading.Thread(target=hilo_can_pantalla,  args=(None, stop), daemon=True),
        threading.Thread(target=hilo_gps,           args=(stop,), daemon=True),
    ]
    for h in hilos:
        h.start()

    # OJO: el hilo de fsync se arranca más abajo, una vez que csv_file existe

    spi = None
    intentos = 0
    try:
        while spi is None:
            try:
                spi_tmp = spidev.SpiDev()
                spi_tmp.open(SPI_BUS, SPI_DEVICE)
                spi_tmp.max_speed_hz  = SPI_VELOCIDAD
                spi_tmp.mode          = SPI_MODO
                spi_tmp.bits_per_word = 8
                spi = spi_tmp
            except Exception as e:
                intentos += 1
                if intentos == 1 or intentos % 10 == 0:
                    print(f"[SPI] No se pudo abrir SPI{SPI_BUS}.{SPI_DEVICE} (intento {intentos}): {e} "
                          f"— reintentando cada {SPI_RETRY_ESPERA:.0f}s (Ctrl+C para cancelar)...")
                time.sleep(SPI_RETRY_ESPERA)
    except KeyboardInterrupt:
        print(f"\n{AMARILLO}[SPI] Cancelado por el usuario — no se encontró el STM32.{RESET}")
        stop.set()
        for h in hilos:
            h.join(timeout=2.0)
        return

    print(f"[SPI] bus={SPI_BUS} device={SPI_DEVICE} @ {SPI_VELOCIDAD/1e6:.1f} MHz")
    print(f"[CSV] {CSV_NOMBRE}")
    time.sleep(1.0)

    csv_file   = open(CSV_NOMBRE, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNAS)
    csv_writer.writeheader()

    # Fsync inmediato del header (archivo + directorio que lo contiene). Así, aunque
    # se corte la alimentación antes de la primera fila de datos, el archivo ya
    # quedó grabado físicamente en la SD con su encabezado, en vez de depender del
    # primer fsync periódico o del writeback espontáneo del kernel (que puede
    # tardar hasta ~30 s por defecto en Linux).
    try:
        csv_file.flush()
        os.fsync(csv_file.fileno())
        dir_fd = os.open(CSV_DIR, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except Exception as e:
        print(f"[CSV] {AMARILLO}No se pudo fsyncear el header: {e}{RESET}")

    h_fsync = threading.Thread(target=hilo_fsync, args=(csv_file, stop), daemon=True)
    h_fsync.start()
    hilos.append(h_fsync)

    n_muestras = n_errores = n_resync = 0
    ultimo_fsync = time.monotonic()
    t_inicio = time.monotonic()

    try:
        while True:
            t_ciclo = time.monotonic()

            raw, descartados = _leer_paquete_stm32(spi)
            n_resync  += descartados
            t_ms       = (time.monotonic() - t_inicio) * 1000.0
            n_muestras += 1

            if raw is None:
                n_errores += 1
                if n_errores % 50 == 0:
                    print(f"[SPI] {AMARILLO}Sin sync con STM32 todavía... "
                          f"({n_errores} intentos fallidos){RESET}")
                continue

            stm = _parsear_paquete_stm32(raw)
            if stm is None:
                n_errores += 1
                continue

            guardar_marcha(stm["marcha"])

            ecu = leer_ecu()
            gps = leer_gps()

            csv_writer.writerow({
                "tiempo_pc_ms":        round(t_ms, 3),
                "tiempo_stm_us":       stm["tiempo_stm_us"],
                "marcha":              stm["marcha"],
                "angulo":              stm["angulo"],
                "acel_lateral":        stm["acel_lateral"],
                "acel_x":              stm["acel_x"],
                "acel_y":              stm["acel_y"],
                "acel_z":              stm["acel_z"],
                "giro_x":              stm["giro_x"],
                "giro_y":              stm["giro_y"],
                "giro_z":              stm["giro_z"],
                "dx1":                 stm["dx1"],
                "dx2":                 stm["dx2"],
                "dx3":                 stm["dx3"],
                "dx4":                 stm["dx4"],
                "dx5":                 stm["dx5"],
                "stm_presion1":        stm["stm_presion1"],
                "stm_presion2":        stm["stm_presion2"],
                "rpm":                 ecu["rpm"],
                "tps":                 ecu["tps"],
                "temp_refrigerante":   ecu["temp_refrigerante"],
                "temp_aceite":         ecu["temp_aceite"],
                "temp_combustible":    ecu["temp_combustible"],
                "presion_combustible": ecu["presion_combustible"],
                "presion_aceite":      ecu["presion_aceite"],
                "bateria_v":           ecu["bateria_v"],
                "gps_lat":             gps["gps_lat"],
                "gps_lon":             gps["gps_lon"],
                "gps_vel_kmh":         gps["gps_vel_kmh"],
            })
            csv_file.flush()

            if time.monotonic() - ultimo_fsync >= FSYNC_CADA_S:
                _fsync_solicitado.set()  # el hilo_fsync lo hace en background
                ultimo_fsync = time.monotonic()

            mostrar_consola(stm, ecu, gps, t_ms, n_muestras, n_errores, n_resync)

            t_espera = CICLO_STM32_S - (time.monotonic() - t_ciclo)
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print(f"\n{VERDE}[OK] Detenido.{RESET}")

    finally:
        stop.set()
        for h in hilos:
            h.join(timeout=2.0)
        spi.close()
        try:
            os.fsync(csv_file.fileno())  # último fsync sincrónico, ya sin apuro de tiempo real
        except Exception:
            pass
        csv_file.close()

        duracion  = time.monotonic() - t_inicio
        aceptados = n_muestras - n_errores
        tasa_ok   = (aceptados / n_muestras * 100) if n_muestras > 0 else 0

        print(f"\n{'─'*48}")
        print(f"  RESUMEN DE SESIÓN")
        print(f"{'─'*48}")
        print(f"  Duración:              {duracion:.1f} s")
        print(f"  Muestras totales:      {n_muestras}")
        print(f"  Aceptados:             {aceptados} ({tasa_ok:.1f}%)")
        print(f"  Frames inválidos:      {n_errores}")
        print(f"  Bytes resincronizados: {n_resync}")
        print(f"  CSV → {CSV_NOMBRE}")
        print(f"{'─'*48}\n")


if __name__ == "__main__":
    main()
