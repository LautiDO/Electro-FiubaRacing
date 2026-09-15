#!/usr/bin/env python3
"""
TELEMETRÍA UNIFICADA v3 — Raspberry Pi

Fuentes:
  STM32 → SPI     (marcha, ángulo, acel_lateral, accel xyz, giro xyz, dx1..dx3, presiones STM)
  ECU   → Serial  (RPM, TPS, AFR, temperaturas, batería) — hilo propio, sticky
  GPS   → CAN RX  (latitud, longitud, velocidad)
Salidas:
  CAN TX    → hilo propio, 10 Hz, manda el último estado sticky a la pantalla del cockpit
  CSV       → hilo propio con cola, no bloquea nada
  Consola   → hilo propio, debug local en la Raspberry
"""

import can, spidev, serial, struct, csv, time, os, threading, queue
from datetime import datetime

# =============================================================
# CONFIGURACIÓN
# =============================================================

CICLO_S = 0.04        # 25 Hz — loop principal (SPI + armado de fila CSV)
DISPLAY_HZ = 10        # refresco de la consola LOCAL de la Raspberry (debug)
DISPLAY_PERIODO_S = 1.0 / DISPLAY_HZ

CAN_TX_PERIODO_S = 0.1  # 10 Hz — cadencia del hilo que manda datos a la pantalla del cockpit

# --- SPI (STM32) ---
SPI_BUS, SPI_DEVICE = 1, 0
SPI_VELOCIDAD = 1_000_000
SPI_MODO = 0b00
STM32_SYNC = 0xAA

STM32_TAM_PKT = 58        # 1 sync + 13 floats + 1 int32 (cambio) + 1 checksum
STM32_FORMATO = '<13fi'  # little-endian: 13 floats + 1 int32 (cambio)

# --- ECU (Serial) ---
ECU_PUERTO    = "/dev/ttyUSB0"
ECU_BAUDRATE  = 115200
ECU_HEADER    = b'\xFF\x7F'
ECU_FOOTER    = b'\x7F\xFF'

ECU_PERIODO_S        = 0.04   # 25 Hz — cada cuánto se reenvía el pedido a la ECU
ECU_RETRY_ESPERA      = 3.0   # segundos entre reintentos de apertura del puerto
ECU_BUFFER_MAX        = 2048  # tope de bytes en buffer sin lograr sync
ECU_RESYNC_TIMEOUT_S  = 2.0   # si no se arma un frame válido en este tiempo, se purga el buffer

# --- CAN ---
CAN_CANAL   = 'can0'
CAN_BITRATE = 1_000_000

CAN_ID_MOTOR       = 0x640
CAN_ID_PRES_ACEITE = 0x644
CAN_ID_TEMPS       = 0x649
CAN_ID_MARCHA      = 0x64D
CAN_ID_LAMBDA      = 0x460

CAN_ID_GPS_LATLON  = 0x680
CAN_ID_GPS_TIEMPO  = 0x681
CAN_ID_GPS_FECHA   = 0x682

SIG_RPM         = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 1,    'add': 0}
SIG_TPS         = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1,    'div': 10,   'add': 0}
SIG_PRES_ACEITE = {'off': 6, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}
SIG_TEMP_REFRIG = {'off': 0, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_TEMP_AIRE   = {'off': 1, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 1,    'add': 40}
SIG_MARCHA      = {'off': 6, 'len': 1, 'mask': 0x0F,   'mult': 1,    'div': 1,    'add': 0}
SIG_BATERIA     = {'off': 5, 'len': 1, 'mask': 0xFF,   'mult': 1,    'div': 10,   'add': 0}
SIG_LAMBDA      = {'off': 0, 'len': 2, 'mask': 0xFFFF, 'mult': 1000, 'div': 1,    'add': 0}

# --- CSV ---
CSV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
os.makedirs(CSV_DIR, exist_ok=True)

def _nombre_csv_unico():
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
    "dx1", "dx2", "dx3",
    "stm_presion1", "stm_presion2",
    "rpm", "tps", "afr",
    "temp_refrigerante", "temp_aire",
    "presion_aceite", "bateria_v",
    "gps_lat", "gps_lon", "gps_vel_kmh",
]

STM_CAMPOS = ["marcha", "angulo", "acel_lateral", "acel_x", "acel_y", "acel_z",
              "giro_x", "giro_y", "giro_z", "dx1", "dx2", "dx3",
              "stm_presion1", "stm_presion2"]
ECU_CAMPOS = ["rpm", "tps", "afr", "temp_refrigerante", "temp_aire",
              "presion_aceite", "bateria_v"]
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

_lock_ecu = threading.Lock()
_estado_ecu = {
    "rpm": None, "tps": None, "afr": None,
    "temp_refrigerante": None, "temp_aire": None,
    "presion_aceite": None, "bateria_v": None,
}

def guardar_ecu(datos):
    with _lock_ecu:
        _estado_ecu.update(datos)

def leer_ecu_sticky():
    with _lock_ecu:
        return dict(_estado_ecu)

_lock_marcha = threading.Lock()
_marcha = 0

def guardar_marcha(valor):
    global _marcha
    with _lock_marcha:
        _marcha = int(valor)

def leer_marcha():
    with _lock_marcha:
        return _marcha

# --- Estado STICKY del STM32 (Aislado por canal) ---
_lock_stm = threading.Lock()
_estado_stm_sticky = {k: None for k in STM_CAMPOS}
_fallos_consecutivos_stm = {k: 0 for k in STM_CAMPOS}
MAX_FALLOS_STICKY = 25  # 25 ciclos @ 25 Hz = 1 segundo de fallas antes de declarar sensor muerto

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
}

def _sensor_valido(campo, val, prev):
    """Evalúa plausibilidad física de un sensor individual."""
    lim = STM_LIMITES.get(campo)
    if lim is None:
        return True  # Campos sin restricciones explícitas (marcha, presiones)

    if val is None:
        return False

    if not (lim["min"] <= val <= lim["max"]):
        return False

    if prev is not None and abs(val - prev) > lim["delta_max"]:
        return False

    return True

def guardar_stm(datos):
    """Actualiza canal por canal. Si un sensor falla, se preserva el resto."""
    global _fallos_consecutivos_stm
    with _lock_stm:
        for campo, val in datos.items():
            if campo not in _estado_stm_sticky:
                continue
            prev = _estado_stm_sticky.get(campo)
            if _sensor_valido(campo, val, prev):
                _estado_stm_sticky[campo] = val
                _fallos_consecutivos_stm[campo] = 0
            else:
                _fallos_consecutivos_stm[campo] += 1
                if _fallos_consecutivos_stm[campo] > MAX_FALLOS_STICKY:
                    _estado_stm_sticky[campo] = None

def leer_stm_sticky():
    with _lock_stm:
        return dict(_estado_stm_sticky)

# --- Estado Display Consola ---
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
        "temp_refrigerante": temp_motor, "temp_aire": temp_aire,
        "presion_aceite": 0,
        "bateria_v": vbat,
    }

def hilo_ecu(stop):
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
                print(f"[ECU] No se pudo abrir {ECU_PUERTO} (intento {intentos}): {e} — reintentando...")
            time.sleep(ECU_RETRY_ESPERA)

    if stop.is_set():
        return

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

            if len(buffer) > ECU_BUFFER_MAX:
                print(f"[ECU] Buffer > {ECU_BUFFER_MAX} bytes sin sync, purgando...")
                buffer.clear()
                ultimo_frame_ok = ahora
            elif (ahora - ultimo_frame_ok) > ECU_RESYNC_TIMEOUT_S and len(buffer) > 0:
                print(f"[ECU] Sin frames válidos hace {ECU_RESYNC_TIMEOUT_S:.0f}s, purgando...")
                buffer.clear()
                ultimo_frame_ok = ahora

            time.sleep(0.001)

        except serial.SerialException as e:
            print(f"[ECU] Puerto perdido: {e} — reintentando...")
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
            buffer.clear()
            time.sleep(0.1)

    if ser and ser.is_open:
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

            chk_calculado = 0
            for byte in raw[1:STM32_TAM_PKT - 1]:
                chk_calculado ^= byte
            chk_recibido = raw[STM32_TAM_PKT - 1]
            if chk_calculado != chk_recibido:
                return None

            try:
                (angulo, acel_lateral,
                 ax, ay, az, gx, gy, gz,
                 dx1, dx2, dx3,
                 presion1, presion2,
                 cambio) = struct.unpack_from(STM32_FORMATO, raw, 1)
            except struct.error:
                return None

            return {
                "marcha": cambio, "angulo": angulo, "acel_lateral": acel_lateral,
                "acel_x": ax, "acel_y": ay, "acel_z": az,
                "giro_x": gx, "giro_y": gy, "giro_z": gz,
                "dx1": dx1, "dx2": dx2, "dx3": dx3,
                "stm_presion1": presion1, "stm_presion2": presion2,
            }
    return None

# =============================================================
# GPS / CAN RX
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
# CAN TX (Pantalla Cockpit)
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
        'temp_aire': 0, 'bateria_v': 0, 'presion_aceite': 0
    }
    if ecu is not None:
        datos_ecu.update(ecu)

    enviar(CAN_ID_MOTOR,       [(datos_ecu['rpm'], SIG_RPM), (datos_ecu['tps'], SIG_TPS)])
    enviar(CAN_ID_TEMPS,       [(datos_ecu['temp_refrigerante'], SIG_TEMP_REFRIG),
                                (datos_ecu['temp_aire'], SIG_TEMP_AIRE),
                                (datos_ecu['bateria_v'], SIG_BATERIA)])
    enviar(CAN_ID_PRES_ACEITE, [(datos_ecu['presion_aceite'], SIG_PRES_ACEITE)])
    enviar(CAN_ID_MARCHA,      [(marcha, SIG_MARCHA)])
    enviar(CAN_ID_LAMBDA,      [(datos_ecu['afr'], SIG_LAMBDA)])

def hilo_can_tx(bus_can, stop):
    print("[CAN TX] Iniciado a 10 Hz...")
    while not stop.is_set():
        inicio = time.perf_counter()
        ecu = leer_ecu_sticky()
        marcha = leer_marcha()
        try:
            enviar_can(bus_can, ecu, marcha)
        except can.CanError:
            pass

        dt = time.perf_counter() - inicio
        resto = CAN_TX_PERIODO_S - dt
        if resto > 0:
            time.sleep(resto)
    print("[CAN TX] Hilo detenido.")

# =============================================================
# CSV & CONSOLA LOCAL
# =============================================================

def hilo_csv(cola_csv, stop):
    print(f"[CSV] Guardando sesión en: {CSV_NOMBRE}")
    with open(CSV_NOMBRE, mode='w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNAS)
        writer.writeheader()
        ultimo_fsync = time.perf_counter()

        while not stop.is_set() or not cola_csv.empty():
            try:
                fila = cola_csv.get(timeout=0.1)
                writer.writerow(fila)
                ahora = time.perf_counter()
                if ahora - ultimo_fsync >= FSYNC_CADA_S:
                    f.flush()
                    os.fsync(f.fileno())
                    ultimo_fsync = ahora
            except queue.Empty:
                continue
        f.flush()
        os.fsync(f.fileno())
    print("[CSV] Archivo cerrado.")

def hilo_consola(stop):
    while not stop.is_set():
        st = leer_estado_display()
        if st:
            def v(campo, default=0):
                val = st.get(campo)
                return val if val is not None else default

            os.system('clear' if os.name == 'posix' else 'cls')
            print("=" * 60)
            print(f" TELEMETRÍA - DEBUG LOCAL ({DISPLAY_HZ} Hz)")
            print("=" * 60)
            marcha_txt = NOMBRES_MARCHA.get(st.get('marcha'), str(st.get('marcha')))
            print(f" Marcha: {marcha_txt:<4} | RPM: {v('rpm'):<5} | TPS: {v('tps'):<5.1f}% | Bat: {v('bateria_v'):<4.1f}V")
            print(f" Temp Mot: {v('temp_refrigerante'):<4.1f}°C | Temp Aire: {v('temp_aire'):<4.1f}°C | AFR: {v('afr'):<4.2f}")
            print(f" Volante: {v('angulo'):<5.1f}° | Ay: {v('acel_lateral'):<5.2f}g | Ax: {v('acel_x'):<5.2f}g")
            print(f" Suspensión (dx1..dx3): {v('dx1')}, {v('dx2')}, {v('dx3')}")
            print(f" GPS: Lat {st.get('gps_lat')} | Lon {st.get('gps_lon')} | Vel {st.get('gps_vel_kmh')} km/h")
            print("=" * 60)
            print(" Presione Ctrl+C para detener.")
        time.sleep(DISPLAY_PERIODO_S)

# =============================================================
# LOOP PRINCIPAL
# =============================================================

def main():
    stop = threading.Event()
    cola_csv = queue.Queue(maxsize=1000)

    # 1. Configurar SPI
    spi = spidev.SpiDev()
    try:
        spi.open(SPI_BUS, SPI_DEVICE)
        spi.max_speed_hz = SPI_VELOCIDAD
        spi.mode = SPI_MODO
    except Exception as e:
        print(f"[ERROR] SPI init falló: {e}")
        return

    # 2. Configurar CAN
    try:
        bus_can = can.interface.Bus(channel=CAN_CANAL, bustype='socketcan', bitrate=CAN_BITRATE)
    except Exception as e:
        print(f"[ERROR] CAN init falló ({CAN_CANAL}): {e}")
        spi.close()
        return

    # 3. Iniciar Hilos
    t_ecu = threading.Thread(target=hilo_ecu, args=(stop,), daemon=True)
    t_gps = threading.Thread(target=hilo_gps, args=(bus_can, stop), daemon=True)
    t_can_tx = threading.Thread(target=hilo_can_tx, args=(bus_can, stop), daemon=True)
    t_csv = threading.Thread(target=hilo_csv, args=(cola_csv, stop), daemon=False)
    t_display = threading.Thread(target=hilo_consola, args=(stop,), daemon=True)

    t_ecu.start()
    t_gps.start()
    t_can_tx.start()
    t_csv.start()
    t_display.start()

    print(f"[MAIN] Loop principal a {1.0/CICLO_S:.0f} Hz...")
    t_inicio_pc = time.perf_counter()

    try:
        while True:
            t_ciclo = time.perf_counter()

            # Lectura SPI
            stm_datos = leer_stm32(spi)
            if stm_datos is not None:
                guardar_stm(stm_datos)
                if "marcha" in stm_datos and stm_datos["marcha"] is not None:
                    guardar_marcha(stm_datos["marcha"])

            # Consolidar datos
            stm_actual = leer_stm_sticky()
            ecu_actual = leer_ecu_sticky()
            gps_actual = leer_gps()

            tiempo_ms = int((time.perf_counter() - t_inicio_pc) * 1000)

            fila = {"tiempo_pc_ms": tiempo_ms}
            fila.update(stm_actual)
            fila.update(ecu_actual)
            fila.update(gps_actual)

            try:
                cola_csv.put_nowait(fila)
            except queue.Full:
                pass

            actualizar_display(fila)

            # Sincronización a 25 Hz
            dt = time.perf_counter() - t_ciclo
            resto = CICLO_S - dt
            if resto > 0:
                time.sleep(resto)

    except KeyboardInterrupt:
        print("\n[MAIN] Deteniendo sistema...")
        stop.set()
        t_csv.join(timeout=2.0)
        spi.close()
        bus_can.shutdown()
        print("[MAIN] Apagado limpio completado.")

if __name__ == "__main__":
    main()