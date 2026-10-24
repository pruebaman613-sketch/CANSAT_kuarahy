"""Interfaz visual CanSat: gráficos, contador y estado del sensado.

Formato serial aceptado:
    42,23.4,987.32,45.6
    Enviando: 42,23.4,987.32,45.6
"""

import re
import time
import threading
import queue
from collections import deque
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec


# -----------------------------------------------------------------------------
# CONFIGURACIÓN
# -----------------------------------------------------------------------------

MODO_DATOS = "serial"       # "serial" o "simulacion"
SERIAL_PORT = "COM9"
BAUD_RATE = 115200
RECONNECT_DELAY_S = 2.0
MAX_PUNTOS = 150
INTERVALO_MS = 300
ALTITUD_DE_DESPEGUE_M = 1.0

PATRON_PAQUETE = re.compile(
    r"(\d+)\s*,\s*"
    r"(-?\d+(?:\.\d+)?)\s*,\s*"
    r"(-?\d+(?:\.\d+)?)\s*,\s*"
    r"(-?\d+(?:\.\d+)?)"
)

COLOR_FONDO = "#F4F6FA"
COLOR_TEXTO = "#172033"
COLOR_SECUNDARIO = "#667085"
COLOR_PANEL = "#111827"
COLOR_BORDE = "#D8DEE9"
COLOR_VERDE = "#21A366"
COLOR_ROJO = "#E5484D"
COLOR_AZUL = "#2F80ED"
COLOR_NARANJA = "#F2994A"

tiempos = deque(maxlen=MAX_PUNTOS)
temperaturas = deque(maxlen=MAX_PUNTOS)
presiones = deque(maxlen=MAX_PUNTOS)
altitudes_relativas = deque(maxlen=MAX_PUNTOS)
altitudes = deque(maxlen=MAX_PUNTOS)
historico = []
cola_datos = queue.Queue()

altitud_inicial = None
inicio_contador_local = None
inicio_despegue_local = None
estado_conexion = {"conectado": False, "ultimo_error": None}


def convertir_linea(linea):
    """Convierte una línea del receptor en un dato válido."""
    match = PATRON_PAQUETE.search(linea)
    if match is None:
        return None
    try:
        pkt, temp, pres, alt = match.groups()
        dato = {
            "tiempo": datetime.now(),
            "pkt": int(pkt),
            "temperatura_C": float(temp),
            "presion_hPa": float(pres),
            "altitud_m": float(alt),
        }
        if dato["presion_hPa"] <= 0 or not all(
            np.isfinite(dato[nombre])
            for nombre in ("temperatura_C", "presion_hPa", "altitud_m")
        ):
            return None
        return dato
    except (ValueError, TypeError):
        return None


def hilo_lectura_serial():
    import serial

    ser = None
    while True:
        if ser is None:
            try:
                print(f"[Serial] Conectando a {SERIAL_PORT} @ {BAUD_RATE}...")
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                estado_conexion["conectado"] = True
                estado_conexion["ultimo_error"] = None
                time.sleep(2)
                ser.reset_input_buffer()
                print("[Serial] Conectado correctamente")
            except serial.SerialException as error:
                estado_conexion["conectado"] = False
                estado_conexion["ultimo_error"] = str(error)
                print(f"[Serial] No se pudo abrir {SERIAL_PORT}: {error}")
                time.sleep(RECONNECT_DELAY_S)
                continue

        try:
            linea = ser.readline().decode("utf-8", errors="ignore").strip()
            if not linea:
                continue
            dato = convertir_linea(linea)
            if dato is not None:
                cola_datos.put(dato)
        except serial.SerialException as error:
            print(f"[Serial] Conexión perdida: {error}")
            estado_conexion["conectado"] = False
            estado_conexion["ultimo_error"] = str(error)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(RECONNECT_DELAY_S)


def hilo_simulacion():
    paso = 0
    while True:
        presion = 1013.25 - paso * 0.15 + np.random.normal(0, 0.4)
        cola_datos.put({
            "tiempo": datetime.now(),
            "pkt": paso,
            "temperatura_C": 22 + 2.5 * np.sin(paso / 20),
            "presion_hPa": presion,
            "altitud_m": 44330 * (1 - (presion / 1013.25) ** (1 / 5.255)),
        })
        estado_conexion["conectado"] = True
        paso += 1
        time.sleep(INTERVALO_MS / 1000)


# -----------------------------------------------------------------------------
# INTERFAZ
# -----------------------------------------------------------------------------

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "axes.titleweight": "bold",
    "axes.labelcolor": COLOR_SECUNDARIO,
    "xtick.color": COLOR_SECUNDARIO,
    "ytick.color": COLOR_SECUNDARIO,
})

fig = plt.figure(figsize=(14, 9.5), facecolor=COLOR_FONDO)
layout = GridSpec(
    4, 2, figure=fig, width_ratios=(3.8, 1.35),
    height_ratios=(1, 1, 1, 1), wspace=0.16, hspace=0.58,
    left=0.06, right=0.97, bottom=0.08, top=0.87,
)

ax_temp = fig.add_subplot(layout[0, 0])
ax_pres = fig.add_subplot(layout[1, 0], sharex=ax_temp)
ax_alt_msl = fig.add_subplot(layout[2, 0], sharex=ax_temp)
ax_alt_rel = fig.add_subplot(layout[3, 0], sharex=ax_temp)

panel_layout = GridSpecFromSubplotSpec(
    6, 1, subplot_spec=layout[:, 1],
    height_ratios=(1.15, 0.72, 0.72, 0.72, 0.72, 0.95), hspace=0.16,
)
ax_timer = fig.add_subplot(panel_layout[0])
ax_temp_value = fig.add_subplot(panel_layout[1])
ax_pres_value = fig.add_subplot(panel_layout[2])
ax_alt_value = fig.add_subplot(panel_layout[3])
ax_alt_rel_value = fig.add_subplot(panel_layout[4])
ax_status = fig.add_subplot(panel_layout[5])

fig.suptitle(
    "Telemetría CanSat",
    x=0.06, y=0.965, ha="left", fontsize=20,
    color=COLOR_TEXTO, fontweight="bold",
)
texto_subtitulo = fig.text(
    0.06, 0.925, "Estación terrena · esperando datos del BMP280",
    ha="left", fontsize=10, color=COLOR_SECUNDARIO,
)


def preparar_grafico(axis, title, ylabel, color):
    axis.set_facecolor("white")
    axis.set_title(title, loc="left", fontsize=10, color=COLOR_TEXTO, pad=8)
    axis.set_ylabel(ylabel, fontsize=9)
    axis.grid(True, color="#E7EBF2", linewidth=0.8)
    for spine in axis.spines.values():
        spine.set_color(COLOR_BORDE)
    line, = axis.plot(
        [], [], color=color, linewidth=2.2,
        marker="o", markersize=3, markerfacecolor=color,
    )
    return line


linea_temp = preparar_grafico(ax_temp, "Temperatura BMP280", "°C", COLOR_ROJO)
linea_pres = preparar_grafico(ax_pres, "Presión atmosférica", "hPa", COLOR_AZUL)
linea_alt_msl = preparar_grafico(
    ax_alt_msl, "Altitud sobre el nivel del mar", "m", COLOR_VERDE
)
linea_alt_rel = preparar_grafico(
    ax_alt_rel, "Altitud del CanSat desde el inicio (0 m)", "m", "#8E5BD9"
)
ax_alt_rel.set_xlabel("Hora local de recepción", fontsize=9)
ax_alt_rel.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))


def preparar_tarjeta(axis, titulo, color):
    axis.set_facecolor("white")
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_color(COLOR_BORDE)
        spine.set_linewidth(1.0)
    axis.text(0.06, 0.76, titulo, transform=axis.transAxes,
              fontsize=9, color=COLOR_SECUNDARIO, fontweight="bold")
    return axis.text(0.06, 0.28, "--", transform=axis.transAxes,
                     fontsize=18, color=color, fontweight="bold")


for axis in (ax_timer, ax_status):
    axis.set_facecolor(COLOR_PANEL)
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_visible(False)

ax_timer.text(0.06, 0.88, "TIEMPO DE SENSADO", transform=ax_timer.transAxes,
              color="#AAB4C5", fontsize=8, fontweight="bold")
texto_timer = ax_timer.text(
    0.06, 0.64, "00:00:00", transform=ax_timer.transAxes,
    color="white", fontsize=20, fontweight="bold",
)
ax_timer.text(0.06, 0.42, "TIEMPO DESDE DESPEGUE (+1 m)",
              transform=ax_timer.transAxes, color="#AAB4C5",
              fontsize=8, fontweight="bold")
texto_despegue = ax_timer.text(
    0.06, 0.14, "00:00:00", transform=ax_timer.transAxes,
    color="#75BFFF", fontsize=20, fontweight="bold",
)

texto_temp = preparar_tarjeta(ax_temp_value, "TEMPERATURA", COLOR_ROJO)
texto_pres = preparar_tarjeta(ax_pres_value, "PRESIÓN", COLOR_AZUL)
texto_alt = preparar_tarjeta(ax_alt_value, "ALTITUD SOBRE NIVEL DEL MAR", COLOR_VERDE)
texto_alt_rel = preparar_tarjeta(
    ax_alt_rel_value, "ALTITUD DEL CANSAT (0 m)", "#8E5BD9"
)

texto_status = ax_status.text(
    0.06, 0.50, "No se sensó nada\nhasta el momento",
    transform=ax_status.transAxes, color="white", fontsize=10,
    va="center", fontweight="bold",
)


def formato_tiempo(segundos):
    segundos = max(0, int(segundos))
    horas, resto = divmod(segundos, 3600)
    minutos, segundos = divmod(resto, 60)
    return f"{horas:02d}:{minutos:02d}:{segundos:02d}"


def actualizar(_frame):
    global altitud_inicial, inicio_contador_local, inicio_despegue_local
    ultimo_dato = None

    while not cola_datos.empty():
        dato = cola_datos.get()
        if inicio_contador_local is None:
            inicio_contador_local = time.monotonic()
        if altitud_inicial is None:
            altitud_inicial = dato["altitud_m"]
            print(f"[Altitud] Referencia inicial: {altitud_inicial:.2f} m")

        altitud_relativa = dato["altitud_m"] - altitud_inicial
        if (
            inicio_despegue_local is None
            and altitud_relativa >= ALTITUD_DE_DESPEGUE_M
        ):
            inicio_despegue_local = time.monotonic()
            print(
                f"[Despegue] Altitud relativa alcanzó "
                f"{ALTITUD_DE_DESPEGUE_M:.1f} m"
            )
        tiempos.append(dato["tiempo"])
        temperaturas.append(dato["temperatura_C"])
        presiones.append(dato["presion_hPa"])
        altitudes.append(dato["altitud_m"])
        altitudes_relativas.append(altitud_relativa)
        historico.append({**dato, "altitud_relativa_m": altitud_relativa})
        ultimo_dato = dato

    if inicio_contador_local is None:
        texto_timer.set_text("00:00:00")
        texto_despegue.set_text("00:00:00")
        texto_status.set_text("No se sensó nada\nhasta el momento")
        texto_subtitulo.set_text(
            "Estación terrena · conectada, esperando telemetría"
            if estado_conexion["conectado"]
            else "Estación terrena · sin conexión al receptor"
        )
        return linea_temp, linea_pres, linea_alt_msl, linea_alt_rel

    segundos = time.monotonic() - inicio_contador_local
    texto_timer.set_text(formato_tiempo(segundos))
    if inicio_despegue_local is None:
        texto_despegue.set_text("00:00:00")
        estado_despegue = "Esperando altitud relativa de 1 m"
    else:
        tiempo_despegue = time.monotonic() - inicio_despegue_local
        texto_despegue.set_text(formato_tiempo(tiempo_despegue))
        estado_despegue = "Despegue detectado"
    texto_status.set_text(
        f"Recibiendo telemetría\nPaquete {ultimo_dato['pkt']}\n{estado_despegue}"
        if ultimo_dato is not None else "Recibiendo telemetría"
    )
    texto_temp.set_text(f"{temperaturas[-1]:.2f} °C")
    texto_pres.set_text(f"{presiones[-1]:.2f} hPa")
    texto_alt.set_text(f"{altitudes[-1]:.2f} m")
    texto_alt_rel.set_text(f"{altitudes_relativas[-1]:.2f} m")
    texto_subtitulo.set_text("Estación terrena · recibiendo telemetría")

    linea_temp.set_data(tiempos, temperaturas)
    linea_pres.set_data(tiempos, presiones)
    linea_alt_msl.set_data(tiempos, altitudes)
    linea_alt_rel.set_data(tiempos, altitudes_relativas)
    # Ajuste explícito de los ejes. Esto evita que el eje compartido deje
    # invisible la curva de temperatura cuando llegan pocos puntos.
    if tiempos:
        inicio = mdates.date2num(tiempos[0])
        fin = mdates.date2num(tiempos[-1])
        margen_x = max((fin - inicio) * 0.04, 1 / 86400)
        limite_x = (inicio - margen_x, fin + margen_x)

        for axis, values in (
            (ax_temp, temperaturas), (ax_pres, presiones),
            (ax_alt_msl, altitudes), (ax_alt_rel, altitudes_relativas),
        ):
            valores = np.asarray(values, dtype=float)
            minimo = float(np.min(valores))
            maximo = float(np.max(valores))
            margen_y = max((maximo - minimo) * 0.15, 0.05)
            axis.set_xlim(limite_x)
            axis.set_ylim(minimo - margen_y, maximo + margen_y)
    return linea_temp, linea_pres, linea_alt_msl, linea_alt_rel


def guardar_historico():
    if historico:
        pd.DataFrame(historico).to_csv("telemetria_historico.csv", index=False)
        print(f"[CSV] Guardados {len(historico)} registros")


if __name__ == "__main__":
    if MODO_DATOS == "serial":
        hilo = threading.Thread(target=hilo_lectura_serial, daemon=True)
    elif MODO_DATOS == "simulacion":
        hilo = threading.Thread(target=hilo_simulacion, daemon=True)
    else:
        raise ValueError('MODO_DATOS debe ser "serial" o "simulacion"')

    hilo.start()
    animacion = FuncAnimation(
        fig, actualizar, interval=INTERVALO_MS, cache_frame_data=False
    )
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        guardar_historico()
