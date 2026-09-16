import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import time
import csv

# ==================== CONFIG ====================
PORT = 'COM14'
BAUD = 115200
WINDOW_SIZE = 1500   # puntos visibles
FILENAME = 'trayectoria_xy.csv'

# ==================== SERIAL ====================
ser = serial.Serial(PORT, BAUD, timeout=1)

# ==================== VARIABLES ====================
xdata = deque(maxlen=WINDOW_SIZE)
ydata = deque(maxlen=WINDOW_SIZE)
grabando = False
datos_guardados = []

# ==================== FIGURA ====================
fig, ax = plt.subplots()
sc = ax.scatter([], [], s=15, color='royalblue', alpha=0.7)

ax.set_xlim(0, 0.22)   # ajusta según tu escala de sensor
ax.set_ylim(0, 0.20)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Trayectoria X vs Y en tiempo real")
ax.grid(True)
ax.set_aspect('equal', adjustable='box')

# ==================== FUNCIONES ====================
def update(frame):
    global grabando
    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if ',' in line:
                x, y = map(float, line.split(','))
                if grabando:
                    xdata.append(x)
                    ydata.append(y)
                    datos_guardados.append((x, y))
        except:
            pass

    if len(xdata) > 0:
        sc.set_offsets(np.c_[xdata, ydata])

    return sc,

def on_key(event):
    global grabando, datos_guardados

    if event.key.lower() == 'r':
        print("▶️ Grabación iniciada...")
        grabando = True
        datos_guardados = []

    elif event.key.lower() == 'f':
        if grabando:
            grabando = False
            print("⏹ Grabación detenida.")
            if datos_guardados:
                print(f"💾 Guardando {len(datos_guardados)} muestras en '{FILENAME}'...")
                with open(FILENAME, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["X [m]", "Y [m]"])
                    writer.writerows(datos_guardados)
                print("✅ Archivo guardado correctamente.")
            else:
                print("⚠️ No se registraron datos.")

# ==================== EVENTO DE TECLAS ====================
fig.canvas.mpl_connect('key_press_event', on_key)

# ==================== ANIMACIÓN ====================
ani = animation.FuncAnimation(
    fig, update, interval=50, blit=False, cache_frame_data=False
)

plt.show()
