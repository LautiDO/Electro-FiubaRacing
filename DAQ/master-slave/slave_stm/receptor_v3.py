#!/usr/bin/env python3
"""
=============================================================
  RECEPTOR SPI — Raspberry Pi  ←→  STM32
=============================================================
  Solicita 34 bytes cada 20 ms al STM32 via SPI,
  valida el paquete, muestra los datos en consola
  y los guarda en CSV.

  Estructura del paquete (34 bytes):
    [0]      → 0xAA  sync byte
    [1..4]   → angulo         float  [°]
    [5..8]   → acel_lateral   float  [g]
    [9..12]  → dx1            float  [mm]
    [13..16] → dx2            float  [mm]
    [17..20] → dx3            float  [mm]
    [21..24] → dx4            float  [mm]
    [25..28] → dx5            float  [mm]
    [29..32] → tiempo_us      uint32 [µs]
    [33]     → checksum XOR

  Dependencias:
    pip install spidev

  Habilitar SPI en la Raspberry:
    sudo raspi-config → Interface Options → SPI → Enable
=============================================================
"""

import spidev
import struct
import csv
import time
import os
from datetime import datetime


# ============================================================
#                     CONFIGURACIÓN
# ============================================================

SPI_BUS       = 0
SPI_DEVICE    = 0
SPI_SPEED_HZ  = 1_000_000
SPI_MODE      = 0b00

PERIODO_MS    = 20
PERIODO_S     = PERIODO_MS / 1000.0

SYNC_BYTE     = 0xAA
PACKET_SIZE   = 34

# Máximo de bytes que se leen buscando el sync antes de rendirse
# en un ciclo. Con 34 bytes de paquete, 68 es suficiente para
# encontrar el encabezado incluso en el peor caso de desfase.
MAX_BUSQUEDA  = PACKET_SIZE * 2

# ── Modo de validación ────────────────────────────────────────────
#  "estricto"  → sync byte + checksum correctos (usar en pista)
#  "sync"      → solo verifica 0xAA, ignora checksum
#  "ninguno"   → acepta todo (debug inicial)
MODO_VALIDACION = "sync"

# ── CSV ───────────────────────────────────────────────────────────
TIMESTAMP_SESION = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_DIR          = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sesiones")
CSV_FILENAME     = os.path.join(CSV_DIR, f"sesion_{TIMESTAMP_SESION}.csv")

CSV_HEADER = [
    "tiempo_pc_ms",
    "tiempo_stm_us",
    "angulo_deg",
    "acel_lateral_g",
    "dx1_mm",
    "dx2_mm",
    "dx3_mm",
    "dx4_mm",
    "dx5_mm",
    "checksum_ok",
]


# ============================================================
#               LECTURA SPI CON RESINCRONIZACIÓN
# ============================================================

def leer_byte(spi: spidev.SpiDev) -> int:
    """Lee un único byte por SPI."""
    return spi.xfer2([0x00])[0]


def leer_paquete_sincronizado(spi: spidev.SpiDev) -> tuple:
    """
    Busca el byte de sincronización 0xAA y luego lee los
    33 bytes restantes del paquete.

    Estrategia:
      1. Leer bytes de a uno hasta encontrar 0xAA.
      2. Una vez hallado, leer los 33 bytes restantes de un tirón.
      3. Armar el frame completo y parsearlo.

    Devuelve (raw_bytes, resincronizaciones):
      raw_bytes          → bytearray de 34 bytes, o None si no encontró sync
      resincronizaciones → cuántos bytes se descartaron buscando el sync
                           (0 = llegó alineado, >0 = había desfase)
    """
    descartados = 0

    # ── Buscar el sync byte ───────────────────────────────
    for _ in range(MAX_BUSQUEDA):
        b = leer_byte(spi)

        if b == SYNC_BYTE:
            # Encontró el encabezado — leer el resto del frame de un tirón
            resto = bytes(spi.xfer2([0x00] * (PACKET_SIZE - 1)))
            raw   = bytes([SYNC_BYTE]) + resto
            return raw, descartados

        descartados += 1

    # Si llegó acá, no encontró el sync en MAX_BUSQUEDA bytes
    return None, descartados


# ============================================================
#                   PARSEO DEL PAQUETE
# ============================================================

def calcular_checksum(raw: bytes) -> tuple:
    cs = 0
    for b in raw[:-1]:
        cs ^= b
    return cs, raw[-1]


def parsear_paquete(raw: bytes) -> tuple:
    """
    Parsea los 34 bytes ya alineados al sync byte.
    Devuelve (datos_dict, estado_str).
    """
    if len(raw) != PACKET_SIZE or raw[0] != SYNC_BYTE:
        return None, "frame_invalido"

    cs_calc, cs_rx = calcular_checksum(raw)
    checksum_ok    = (cs_calc == cs_rx)

    if MODO_VALIDACION == "estricto" and not checksum_ok:
        return None, f"checksum  esperado={cs_calc:#04x}  recibido={cs_rx:#04x}"

    try:
        angulo, acel, dx1, dx2, dx3, dx4, dx5, tiempo_us = struct.unpack_from(
            "<7fI", raw, 1
        )
    except struct.error as e:
        return None, f"unpack_error: {e}"

    estado = "ok" if checksum_ok else "ok_sin_cs"

    datos = {
        "angulo_deg":     angulo,
        "acel_lateral_g": acel,
        "dx1_mm":         dx1,
        "dx2_mm":         dx2,
        "dx3_mm":         dx3,
        "dx4_mm":         dx4,
        "dx5_mm":         dx5,
        "tiempo_stm_us":  tiempo_us,
        "_checksum_ok":   checksum_ok,
    }
    return datos, estado


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


def mostrar_consola(datos: dict, t_pc_ms: float, n_muestra: int,
                    n_errores: int, n_sin_cs: int, n_resync: int):

    total       = n_muestra
    tasa_error  = (n_errores / total * 100) if total > 0 else 0.0
    tasa_sin_cs = (n_sin_cs  / total * 100) if total > 0 else 0.0

    cs_indicador = (f"{VERDE}✓ CS OK{RESET}"
                    if datos["_checksum_ok"]
                    else f"{AMARILLO}⚠ CS falló — guardado igual{RESET}")

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*56}{RESET}")
    print(f"{BOLD}{CYAN}   STM32 → Raspberry Pi  |  SPI Data Logger{RESET}")
    print(f"{CYAN}  Modo: {BOLD}{MODO_VALIDACION.upper()}{RESET}{CYAN}   {cs_indicador}")
    print(f"{CYAN}{'='*56}{RESET}")

    print(f"\n  {BOLD}Muestra #{n_muestra:>6}{RESET}   "
          f"t_PC={t_pc_ms:>10.1f} ms   "
          f"t_STM={datos['tiempo_stm_us']:>10} µs")

    print(f"\n  {'─'*52}")
    print(f"  {'SENSOR':<24}  {'VALOR':>10}  UNIDAD")
    print(f"  {'─'*52}")
    print(f"  {'Ángulo volante':<24}  {datos['angulo_deg']:>10.2f}  °")
    print(f"  {'Acel. lateral':<24}  {datos['acel_lateral_g']:>10.3f}  g")
    print(f"  {'─'*52}")
    print(f"  {'Suspensión 1 (dx1)':<24}  {datos['dx1_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 2 (dx2)':<24}  {datos['dx2_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 3 (dx3)':<24}  {datos['dx3_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 4 (dx4)':<24}  {datos['dx4_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 5 (dx5)':<24}  {datos['dx5_mm']:>10.1f}  mm")
    print(f"  {'─'*52}")

    color_err   = VERDE if tasa_error  < 1.0 else (AMARILLO if tasa_error  < 5.0 else ROJO)
    color_cs    = VERDE if tasa_sin_cs < 5.0 else AMARILLO
    color_resync = VERDE if n_resync == 0 else AMARILLO

    print(f"\n  Descartados (sin sync): {color_err}{n_errores} ({tasa_error:.1f}%){RESET}")
    print(f"  CS falló (guardados):   {color_cs}{n_sin_cs} ({tasa_sin_cs:.1f}%){RESET}")
    print(f"  Resincronizaciones:     {color_resync}{n_resync}{RESET}"
          f"{GRIS}  (>0 = hubo desfase de bytes){RESET}")
    print(f"\n  {GRIS}CSV → {CSV_FILENAME}{RESET}")
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

    print(f"[OK] SPI  bus={SPI_BUS}  device={SPI_DEVICE}  "
          f"velocidad={SPI_SPEED_HZ/1e6:.1f} MHz  modo={SPI_MODE}")
    print(f"[OK] Modo validación : {MODO_VALIDACION.upper()}")
    print(f"[OK] Guardando en    : {CSV_FILENAME}")
    print(f"[..] Iniciando en 1 segundo...\n")
    time.sleep(1.0)

    csv_file   = open(CSV_FILENAME, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADER)
    csv_writer.writeheader()

    n_muestra   = 0
    n_errores   = 0
    n_sin_cs    = 0
    n_resync    = 0   # total acumulado de bytes descartados buscando sync
    t_inicio    = time.monotonic()

    try:
        while True:
            t_ciclo = time.monotonic()

            # ── Leer paquete con resincronización ─────────
            raw, descartados = leer_paquete_sincronizado(spi)

            if descartados > 0:
                n_resync += descartados
                print(f"\r  {AMARILLO}[RESYNC] Se descartaron {descartados} bytes "
                      f"buscando 0xAA  (total acum: {n_resync}){RESET}",
                      end="", flush=True)

            t_pc_ms    = (time.monotonic() - t_inicio) * 1000.0
            n_muestra += 1

            if raw is None:
                # No encontró el sync en MAX_BUSQUEDA bytes
                n_errores += 1
                print(f"\r  {ROJO}[!] Sin sync en {MAX_BUSQUEDA} bytes. "
                      f"Verificar conexión SPI.{RESET}",
                      end="", flush=True)
                continue

            # ── Parsear ───────────────────────────────────
            datos, estado = parsear_paquete(raw)

            if datos is None:
                n_errores += 1
                print(f"\r  {AMARILLO}[!] Frame inválido #{n_muestra}: {estado}{RESET}",
                      end="", flush=True)
            else:
                if not datos["_checksum_ok"]:
                    n_sin_cs += 1

                fila = {
                    "tiempo_pc_ms":   round(t_pc_ms, 3),
                    "tiempo_stm_us":  datos["tiempo_stm_us"],
                    "angulo_deg":     round(datos["angulo_deg"],     3),
                    "acel_lateral_g": round(datos["acel_lateral_g"], 4),
                    "dx1_mm":         round(datos["dx1_mm"],         2),
                    "dx2_mm":         round(datos["dx2_mm"],         2),
                    "dx3_mm":         round(datos["dx3_mm"],         2),
                    "dx4_mm":         round(datos["dx4_mm"],         2),
                    "dx5_mm":         round(datos["dx5_mm"],         2),
                    "checksum_ok":    1 if datos["_checksum_ok"] else 0,
                }
                csv_writer.writerow(fila)
                csv_file.flush()

                mostrar_consola(datos, t_pc_ms, n_muestra,
                                n_errores, n_sin_cs, n_resync)

            # ── Mantener período ──────────────────────────
            # Nota: si hubo resincronización, el ciclo tardó más
            # de 20 ms. En ese caso t_espera será negativo y no
            # se duerme — el próximo ciclo arranca de inmediato
            # para recuperar el ritmo.
            t_espera = PERIODO_S - (time.monotonic() - t_ciclo)
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print(f"\n\n{VERDE}[OK] Detenido por el usuario.{RESET}")

    finally:
        spi.close()
        csv_file.close()

        duracion  = time.monotonic() - t_inicio
        aceptados = n_muestra - n_errores
        tasa_ok   = (aceptados / n_muestra * 100) if n_muestra > 0 else 0

        print(f"\n{'─'*52}")
        print(f"  RESUMEN DE SESIÓN")
        print(f"{'─'*52}")
        print(f"  Duración:               {duracion:.1f} s")
        print(f"  Muestras totales:       {n_muestra}")
        print(f"  Aceptados:              {aceptados}  ({tasa_ok:.1f}%)")
        print(f"  Descartados (sin sync): {n_errores}")
        print(f"  CS falló (guardados):   {n_sin_cs}")
        print(f"  Bytes resincronizados:  {n_resync}")
        print(f"  CSV → {CSV_FILENAME}")
        print(f"{'─'*52}\n")


if __name__ == "__main__":
    main()
