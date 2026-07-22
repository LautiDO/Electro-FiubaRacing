#!/usr/bin/env python3
"""
TELEMETRÍA UNIFICADA — Raspberry Pi
Fuentes de datos:
  - STM32   → SPI       (marcha, tiempo)
  - ECU     → Serial    (RPM, TPS, temperaturas, presiones, batería)
  - GPS     → CAN RX    (latitud, longitud, velocidad)
Salidas:
  - CSV     → archivo por sesión en /sesiones/
  - Pantalla cockpit → CAN TX cada 500 ms
  - Consola → display en tiempo real
"""

import can, spidev, serial, struct, csv, time, os, threading
from datetime import datetime


# =============================================================
# CONFIGURACIÓN — editá acá para cambiar puertos, IDs, timings
# =============================================================

FSYNC_CADA_N_MUESTRAS = 100

# --- SPI (STM32) ---
SPI_BUS        = 1
SPI_DEVICE     = 0
SPI_VELOCIDAD  = 1_000_000   # Hz
SPI_MODO       = 0b00
CICLO_STM32_S  = 0.020       # 50 Hz → cada 20 ms

STM32_SYNC     = 0xAA        # byte que marca inicio de paquete
STM32_TAM_PKT  = 9           # 1 sync + 1 int32 (cambio) + 1 uint32 (tiempo_us)
STM32_FORMATO  = '<iI'       # little-endian: cambio (int32), tiempo_us (uint32)

# --- ECU (Serial) ---
ECU_PUERTO     = "/dev/ttyUSB0"
ECU_BAUDRATE   = 115200
ECU_PERIODO_S  = 0.04        # 25 Hz

# --- CAN ---
CAN_CANAL      = 'can0'
CAN_BITRATE    = 1_000_000
CAN_TX_PERIODO = 0.5         # segundos entre envíos a pantalla

# --- DEBUG ---
DEBUG_CAN_TX      = True     # imprime en consola cada envío a la pantalla
CAN_TX_INTERFRAME = 0.003    # pausa entre cada bus.send() (alivia el buffer)

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
CSV_NOMBRE = os.path.join(CSV_DIR, f"sesion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

CSV_COLUMNAS = [
    "tiempo_pc_ms", "tiempo_stm_us",
    "marcha",
    "rpm", "tps",
    "temp_refrigerante", "temp_aceite", "temp_combustible",
    "presion_combustible", "presion_aceite",
    "bateria_v",
    "gps_lat", "gps_lon", "gps_vel_kmh",
]

# --- Señales CAN TX (offset, largo, máscara, multiplicador, divisor, offset_valor) ---
# Cada señal define cómo empaquetar un valor en un mensaje CAN de 8 bytes
SIG_RPM          = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 2,    'add': 0}
SIG_TPS          = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1000, 'add': 0}
SIG_PRES_COMB    = {'off': 4, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_PRES_ACEITE  = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_TEMP_REFRIG  = {'off': 0, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_ACEITE  = {'off': 1, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_COMB    = {'off': 2, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_MARCHA       = {'off': 6, 'len': 1, 'mask': 0x0F,   'mult': 1,    'div': 1,    'add': 0}
SIG_BATERIA      = {'off': 5, 'len': 1, 'mask': 0xFF, 'mult': 1,  'div': 10,    'add': 0}  # V × 100, bytes 3-4 del msg 0x649


# =============================================================
# ESTADO COMPARTIDO ENTRE HILOS (protegido con locks)
# =============================================================

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
    """Arma una trama de pedido para la ECU con header/footer y checksum."""
    HEADER = b'\xFF\x7F'
    FOOTER = b'\x7F\xFF'
    n = 10 + 0  # sin payload extra
    base = HEADER + bytes([cmd[0], cmd[1], cmd[2], n]) + FOOTER
    chk  = _checksum_ecu(base)
    return bytearray(HEADER + bytes([cmd[0], cmd[1]]) + chk + bytes([cmd[2], n]) + FOOTER)

def _decodificar_ecu(payload):
    """
    Decodifica el payload de respuesta de la ECU (big-endian).
    Offsets del protocolo:
      [8..9]   → RPM        (int16 BE)
      [10]     → AFR/fuel   (uint8, /10.0)   ← TODO: confirmar si es presión combustible
      [14..15] → TPS        (int16 BE, /10.0 → %)
      [18..19] → Temp motor (int16 BE, /10.0 → °C)  ← TODO: separar coolant/oil/fuel
      [22..23] → Vbat       (int16 BE, /100.0 → V)  ← TODO: confirmar si es presión aceite
    """
    try:
        rpm       = struct.unpack_from('>h', payload,  8)[0]
        afr       = struct.unpack_from('B',  payload, 10)[0] / 10.0
        tps       = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp      = struct.unpack_from('>h', payload, 18)[0] / 10.0
        vbat      = struct.unpack_from('>h', payload, 22)[0] / 100.0
        return {
            "rpm": rpm, "tps": tps,
            "temp_refrigerante":  temp,
            "temp_aceite":        temp,   # TODO: offset propio cuando esté disponible
            "temp_combustible":   temp,   # TODO: offset propio cuando esté disponible
            "presion_combustible": afr,   # TODO: confirmar offset real
            "presion_aceite":      vbat,  # TODO: confirmar offset real
            "bateria_v":           vbat,
        }
    except struct.error:
        return None

def hilo_ecu(stop):
    """Lee la ECU por serial a 25 Hz y actualiza el estado compartido."""
    HEADER = b'\xFF\x7F'
    FOOTER = b'\x7F\xFF'

    try:
        ser = serial.Serial(ECU_PUERTO, ECU_BAUDRATE, timeout=0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[ECU] No se pudo abrir {ECU_PUERTO}: {e} — ECU deshabilitada.")
        return

    print(f"[ECU] Conectado en {ECU_PUERTO} @ {ECU_BAUDRATE} baud")

    # Handshake inicial
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

    while not stop.is_set():
        try:
            ahora = time.perf_counter()

            # Enviar pedido a 25 Hz
            if ahora - ultimo >= ECU_PERIODO_S:
                ser.write(pedido)
                ultimo = ahora

            # Leer bytes disponibles sin bloquear
            n = ser.in_waiting
            if n:
                buffer.extend(ser.read(n))

            # Procesar tramas completas acumuladas en el buffer
            while HEADER in buffer and FOOTER in buffer:
                ini = buffer.find(HEADER)
                fin = buffer.find(FOOTER, ini)
                if ini == -1 or fin == -1:
                    break
                trama  = buffer[ini: fin + 2]
                buffer = buffer[fin + 2:]

                # Byte [2] == 180 → respuesta de datos online
                if len(trama) > 12 and trama[2] == 180:
                    datos = _decodificar_ecu(trama[10:-2])
                    if datos:
                        guardar_ecu(datos)

            time.sleep(0.001)

        except serial.SerialException as e:
            # Puerto se desconectó en caliente — esperar y reintentar
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
            print(f"[ECU] Error inesperado: {e}")
            break

    ser.close()
    print("[ECU] Puerto cerrado.")


# =============================================================
# GPS — decodificadores de mensajes CAN
# =============================================================

def _decodificar_gps_latlon(data):
    """CAN 0x680 → latitud y longitud."""
    lat = struct.unpack_from('>i', data, 0)[0] * 1e-7
    lon = struct.unpack_from('>i', data, 4)[0] * 1e-7
    return {"gps_lat": lat, "gps_lon": lon}

def _decodificar_gps_tiempo(data):
    """CAN 0x681 → velocidad (y hora UTC, no guardada en CSV)."""
    return {"gps_vel_kmh": data[5] * 0.1}

def _decodificar_gps_fecha(data):
    """CAN 0x682 → sin campos usados en CSV actualmente."""
    return {}

GPS_DECODIFICADORES = {
    CAN_ID_GPS_LATLON: _decodificar_gps_latlon,
    CAN_ID_GPS_TIEMPO: _decodificar_gps_tiempo,
    CAN_ID_GPS_FECHA:  _decodificar_gps_fecha,
}

def hilo_gps(stop):
    """Escucha mensajes CAN del GPS y actualiza el estado compartido."""
    try:
        bus = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan')
    except OSError:
        print("[GPS] CAN no disponible — GPS deshabilitado.")
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
    """Escribe un valor en el payload CAN según la definición de señal."""
    raw = int((valor + sig['add']) * sig['div'] * sig['mult']) & sig['mask']
    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']]     = (raw >> 8) & 0xFF
        payload[sig['off'] + 1] =  raw       & 0xFF
    return payload

def hilo_can_pantalla(_, stop):
    """Envía datos de ECU y marcha a la pantalla del cockpit cada 500 ms."""
    ECU_DEFAULTS = {k: 0 for k in _estado_ecu}

    try:
        bus = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan')
    except OSError:
        print(f"[CAN-TX] {CAN_CANAL} no disponible — pantalla cockpit deshabilitada.")
        print(f"         (Para activarla: sudo ip link set {CAN_CANAL} up type can bitrate {CAN_BITRATE})")
        return  # ← solo termina este hilo, no detiene el programa

    print(f"[CAN-TX] Enviando en {CAN_CANAL} @ {CAN_BITRATE//1000} kbps")

    def enviar(can_id, señales):
        payload = [0] * 8
        for valor, sig in señales:
            _empaquetar_senal(payload, valor, sig)
        try:
            bus.send(can.Message(arbitration_id=can_id, data=payload, is_extended_id=False))
        except can.CanError as e:
            print(f"[CAN-TX] {GRIS}0x{can_id:03X} perdido: {e}{RESET}")
            return
        if DEBUG_CAN_TX:
            print(f"[CAN-TX] {GRIS}0x{can_id:03X}{RESET}  {bytes(payload).hex(' ')}")
        time.sleep(CAN_TX_INTERFRAME)  # da tiempo a que el bus/controlador vacíe

    try:
        while not stop.is_set():
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

            time.sleep(CAN_TX_PERIODO)

    except Exception as e:
        print(f"[CAN-TX] Error: {e}")
    finally:
        bus.shutdown()
        print("[CAN-TX] Bus cerrado.")


# =============================================================
# STM32 — lectura SPI
# =============================================================

def _leer_paquete_stm32(spi):
    """
    Busca el byte de sincronía 0xAA y lee los bytes restantes del paquete.
    Retorna (paquete_bytes, bytes_descartados).
    Si no encuentra el sync, retorna (None, bytes_descartados).
    """
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
    Decodifica los bytes del paquete STM32 (marcha + tiempo).
    Retorna un dict con los valores, o None si el paquete es inválido.
    """
    if len(raw) != STM32_TAM_PKT or raw[0] != STM32_SYNC:
        return None
    try:
        cambio, t_us = struct.unpack_from(STM32_FORMATO, raw, 1)
    except struct.error:
        return None
    return {
        "marcha":        cambio,
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

    # STM32
    print(f"  {'Marcha':<22}  {marcha:>10}")

    # ECU
    print(f"  {'RPM':<22}  {ecu['rpm'] or '---':>10}  rpm")
    print(f"  {'TPS':<22}  {ecu['tps'] or '---':>10}  %")
    print(f"  {'Temp. refrigerante':<22}  {_fmt(ecu['temp_refrigerante'])}  °C")
    print(f"  {'Temp. aceite':<22}  {_fmt(ecu['temp_aceite'])}  °C")
    print(f"  {'Temp. combustible':<22}  {_fmt(ecu['temp_combustible'])}  °C")
    print(f"  {'Presión combustible':<22}  {_fmt(ecu['presion_combustible'])}  bar")
    print(f"  {'Presión aceite':<22}  {_fmt(ecu['presion_aceite'])}  bar")
    print(f"  {'Batería':<22}  {_fmt(ecu['bateria_v'], 2)}  V")

    # GPS
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

    # Arrancar hilos de fondo
    hilos = [
        threading.Thread(target=hilo_ecu,          args=(stop,), daemon=True),
        threading.Thread(target=hilo_can_pantalla,  args=(None, stop), daemon=True),
        threading.Thread(target=hilo_gps,           args=(stop,), daemon=True),
    ]
    for h in hilos:
        h.start()

    # Inicializar SPI
    try:
        spi = spidev.SpiDev()
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz  = SPI_VELOCIDAD
        spi.mode          = SPI_MODO
        spi.bits_per_word = 8
        print(f"[SPI] bus={SPI_BUS} device={SPI_DEVICE} @ {SPI_VELOCIDAD/1e6:.1f} MHz")
    except Exception as e:
        print(f"[SPI] No se pudo abrir SPI{SPI_BUS}.{SPI_DEVICE}: {e}")
        print(f"      El programa no puede continuar sin el STM32.")
        stop.set()
        for h in hilos:
            h.join(timeout=2.0)
        return
    print(f"[CSV] {CSV_NOMBRE}")
    time.sleep(1.0)

    # Abrir CSV
    csv_file   = open(CSV_NOMBRE, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNAS)
    csv_writer.writeheader()

    n_muestras = n_errores = n_resync = 0
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
                if n_errores % 50 == 0:  # cada ~50 intentos fallidos, avisa que sigue vivo
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

            # Escribir fila en CSV
            csv_writer.writerow({
                "tiempo_pc_ms":        round(t_ms, 3),
                "tiempo_stm_us":       stm["tiempo_stm_us"],
                "marcha":              stm["marcha"],
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

            if n_muestras % FSYNC_CADA_N_MUESTRAS == 0:
                os.fsync(csv_file.fileno())

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