#!/usr/bin/env python3
"""
=============================================================
  RECEPTOR SPI — Raspberry Pi 3  <-->  STM32
=============================================================

  Estructura del paquete (45 bytes, SIN checksum):
    [0]      -> 0xAA       sync byte
    [1..4]   -> angulo         float  [°]
    [5..8]   -> acel_lateral   float  [g]
    [9..12]  -> dx1            float  [mm]
    [13..16] -> dx2            float  [mm]
    [17..20] -> dx3            float  [mm]
    [21..24] -> dx4            float  [mm]
    [25..28] -> dx5            float  [mm]
    [29..32] -> presion1       float
    [33..36] -> presion2       float
    [37..40] -> cambio         uint32
    [41..44] -> tiempo_us      uint32 [µs]

  Instalación:
    pip install spidev

  Habilitar SPI:
    sudo raspi-config -> Interface Options -> SPI -> Enable

  Conexión (SPI0, CE0):
    STM32 PB3  (SCK)  -> RPi GPIO11 (pin 23)
    STM32 PB4  (MISO) -> RPi GPIO9  (pin 21)
    STM32 PB5  (MOSI) -> RPi GPIO10 (pin 19)
    STM32 PA15 (NSS)  -> RPi GPIO8  (pin 24)
    GND               -> GND
=============================================================
"""

import spidev
import struct
import csv
import time
import os
from datetime import datetime

# ── Configuración SPI ────────────────────────────────────────────
SPI_BUS      = 0
SPI_DEVICE   = 0
SPI_SPEED_HZ = 1_000_000   # 1 MHz — bajar a 500_000 si hay errores
SPI_MODE     = 0b00

# ── Paquete ──────────────────────────────────────────────────────
SYNC_BYTE    = 0xAA
PACKET_SIZE  = 45           # 1 sync + 9 floats×4 + 2 uint32×4 = 45
MAX_BUSQUEDA = PACKET_SIZE * 3

# 9 floats: angulo, acel, dx1..dx5, presion1, presion2
# 2 uint32: cambio, tiempo_us
STRUCT_FMT = "<9fII"

# ── Período ──────────────────────────────────────────────────────
PERIODO_S = 0.02

# ── CSV ──────────────────────────────────────────────────────────
CSV_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
CSV_FILE = os.path.join(CSV_DIR, f"sesion_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

HEADER = [
    "tiempo_pc_ms", "tiempo_stm_us",
    "angulo_deg", "acel_lateral_g",
    "dx1_mm", "dx2_mm", "dx3_mm", "dx4_mm", "dx5_mm",
    "presion1", "presion2",
    "cambio",
]

# ── ANSI ─────────────────────────────────────────────────────────
VERDE    = "\033[92m"
AMARILLO = "\033[93m"
ROJO     = "\033[91m"
CYAN     = "\033[96m"
GRIS     = "\033[90m"
RESET    = "\033[0m"
BOLD     = "\033[1m"
CLEAR    = "\033[2J\033[H"

MARCHAS = {0: "Neutro", 1: "1a", 2: "2a", 3: "3a", 4: "4a", 5: "5a", 6: "6a", -1: "???"}


# ============================================================
#                    FUNCIONES SPI
# ============================================================

def leer_byte(spi):
    return spi.xfer2([0x00])[0]


def buscar_sync(spi):
    """Lee hasta encontrar 0xAA. Devuelve bytes descartados, -1 si no encontró."""
    for descartados in range(MAX_BUSQUEDA):
        if leer_byte(spi) == SYNC_BYTE:
            return descartados
    return -1


def leer_paquete(spi):
    """
    Busca el sync y lee los 44 bytes restantes de un tirón.
    Devuelve (raw_bytes | None, bytes_descartados).
    """
    descartados = buscar_sync(spi)
    if descartados == -1:
        return None, MAX_BUSQUEDA

    resto = bytes(spi.xfer2([0x00] * (PACKET_SIZE - 1)))
    return bytes([SYNC_BYTE]) + resto, descartados


# ============================================================
#                    PARSEO
# ============================================================

def parsear(raw):
    """Parsea 45 bytes alineados. Devuelve dict o None."""
    if len(raw) != PACKET_SIZE or raw[0] != SYNC_BYTE:
        return None

    try:
        angulo, acel, dx1, dx2, dx3, dx4, dx5, pres1, pres2, cambio, tiempo_us = \
            struct.unpack_from(STRUCT_FMT, raw, 1)
    except struct.error as e:
        print(f"  {ROJO}[!] Error de unpack: {e}{RESET}")
        return None

    return {
        "angulo":    angulo,
        "acel":      acel,
        "dx1":       dx1,
        "dx2":       dx2,
        "dx3":       dx3,
        "dx4":       dx4,
        "dx5":       dx5,
        "presion1":  pres1,
        "presion2":  pres2,
        "cambio":    int(cambio),
        "tiempo_us": tiempo_us,
    }


# ============================================================
#                    DISPLAY
# ============================================================

def mostrar(d, t_pc_ms, n, n_err, n_desync):
    tasa_err = (n_err / n * 100) if n > 0 else 0.0
    c_err    = VERDE if tasa_err < 2 else (AMARILLO if tasa_err < 10 else ROJO)
    c_ds     = VERDE if n_desync == 0 else AMARILLO
    marcha   = MARCHAS.get(d["cambio"], "???")
    c_marcha = VERDE if d["cambio"] >= 0 else AMARILLO

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*56}{RESET}")
    print(f"{BOLD}{CYAN}   STM32  ->  Raspberry Pi  --  Receptor SPI{RESET}")
    print(f"{CYAN}{'='*56}{RESET}")
    print(f"\n  {BOLD}Muestra #{n:<6}{RESET}  "
          f"t_PC={t_pc_ms:>10.1f} ms  |  t_STM={d['tiempo_us']:>10} us")

    print(f"\n  {'─'*52}")
    print(f"  {'SEÑAL':<30}  {'VALOR':>10}  UNIDAD")
    print(f"  {'─'*52}")
    print(f"  {'Angulo volante':<30}  {d['angulo']:>10.2f}  deg")
    print(f"  {'Acel. lateral':<30}  {d['acel']:>10.3f}  g")
    print(f"  {'─'*52}")
    print(f"  {'Extensometro 1 (dx1)':<30}  {d['dx1']:>10.1f}  mm")
    print(f"  {'Extensometro 2 (dx2)':<30}  {d['dx2']:>10.1f}  mm")
    print(f"  {'Extensometro 3 (dx3)':<30}  {d['dx3']:>10.1f}  mm")
    print(f"  {'Extensometro 4 (dx4)':<30}  {d['dx4']:>10.1f}  mm")
    print(f"  {'Extensometro 5 (dx5)':<30}  {d['dx5']:>10.1f}  mm")
    print(f"  {'─'*52}")
    print(f"  {'Presion 1':<30}  {d['presion1']:>10.2f}")
    print(f"  {'Presion 2':<30}  {d['presion2']:>10.2f}")
    print(f"  {'─'*52}")
    print(f"  {'Marcha':<30}  {c_marcha}{marcha:>10}{RESET}")
    print(f"  {'─'*52}")
    print(f"\n  Paquetes erroneos:   {c_err}{n_err}  ({tasa_err:.1f}%){RESET}")
    print(f"  Bytes descartados:   {c_ds}{n_desync}{RESET}")
    print(f"\n  {GRIS}CSV -> {CSV_FILE}{RESET}")
    print(f"  {CYAN}Ctrl+C para detener{RESET}\n")


# ============================================================
#                         MAIN
# ============================================================

def main():
    os.makedirs(CSV_DIR, exist_ok=True)

    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz  = SPI_SPEED_HZ
    spi.mode          = SPI_MODE
    spi.bits_per_word = 8

    print(f"{VERDE}[OK]{RESET} SPI -- bus={SPI_BUS} dev={SPI_DEVICE} "
          f"vel={SPI_SPEED_HZ/1e3:.0f} kHz modo={SPI_MODE}")
    print(f"{VERDE}[OK]{RESET} CSV -> {CSV_FILE}")
    print("[..] Iniciando en 1 segundo...\n")
    time.sleep(1.0)

    csv_f  = open(CSV_FILE, "w", newline="")
    writer = csv.DictWriter(csv_f, fieldnames=HEADER)
    writer.writeheader()

    n        = 0
    n_err    = 0
    n_desync = 0
    t0       = time.monotonic()

    try:
        while True:
            t_ciclo = time.monotonic()
            t_pc_ms = (t_ciclo - t0) * 1000.0
            n += 1

            raw, descartados = leer_paquete(spi)
            n_desync += descartados

            if descartados > 0:
                print(f"\r  {AMARILLO}[resync] {descartados} bytes descartados "
                      f"(total: {n_desync}){RESET}", end="", flush=True)

            if raw is None:
                n_err += 1
                print(f"\r  {ROJO}[!] Sin sync en {MAX_BUSQUEDA} bytes{RESET}",
                      end="", flush=True)
                continue

            d = parsear(raw)
            if d is None:
                n_err += 1
                continue

            writer.writerow({
                "tiempo_pc_ms":   round(t_pc_ms, 2),
                "tiempo_stm_us":  d["tiempo_us"],
                "angulo_deg":     round(d["angulo"],   3),
                "acel_lateral_g": round(d["acel"],     4),
                "dx1_mm":         round(d["dx1"],      2),
                "dx2_mm":         round(d["dx2"],      2),
                "dx3_mm":         round(d["dx3"],      2),
                "dx4_mm":         round(d["dx4"],      2),
                "dx5_mm":         round(d["dx5"],      2),
                "presion1":       round(d["presion1"], 2),
                "presion2":       round(d["presion2"], 2),
                "cambio":         d["cambio"],
            })
            csv_f.flush()

            mostrar(d, t_pc_ms, n, n_err, n_desync)

            t_espera = PERIODO_S - (time.monotonic() - t_ciclo)
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print(f"\n\n{VERDE}[OK] Detenido por el usuario.{RESET}")

    finally:
        spi.close()
        csv_f.close()
        duracion  = time.monotonic() - t0
        aceptados = n - n_err
        tasa_ok   = (aceptados / n * 100) if n > 0 else 0.0
        print(f"\n{'─'*52}")
        print(f"  Duracion:            {duracion:.1f} s")
        print(f"  Muestras totales:    {n}")
        print(f"  Aceptados:           {aceptados}  ({tasa_ok:.1f}%)")
        print(f"  Errores de parseo:   {n_err}")
        print(f"  Bytes resincroniz.:  {n_desync}")
        print(f"  CSV -> {CSV_FILE}")
        print(f"{'─'*52}\n")


if __name__ == "__main__":
    main()
