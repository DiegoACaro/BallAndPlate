import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np

PORT = 'COM14'
BAUD = 115200
MODO = 2
#1 -> X,Y real time
#2 -> X,Y vs time
if(MODO == 1):
    import serial
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from collections import deque
    import numpy as np

    PORT = 'COM14'
    BAUD = 115200
    WINDOW_SIZE = 500

    ser = serial.Serial(PORT, BAUD)
    xdata, ydata = deque(maxlen=WINDOW_SIZE), deque(maxlen=WINDOW_SIZE)

    fig, ax = plt.subplots()
    sc = ax.scatter([], [], s=20, c='blue', alpha=0.7)
    ax.set_xlim(0, 0.22)
    ax.set_ylim(0, 0.20)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Posición en tiempo real (Scatter)")
    ax.set_aspect('equal', adjustable='box')
    maxpoints = 450
    def update(frame):
        while ser.in_waiting:
            try:
                line = ser.readline().decode().strip()
                if ',' in line:
                    x, y = map(float, line.split(','))
                    xdata.append(x)
                    ydata.append(y)

                            # Límite de puntos visibles (rastro)

            except:
                pass
        print(len(xdata))
        if len(xdata) > maxpoints:
            sc.set_offsets(np.c_[xdata, ydata])
        return sc,





if(MODO ==2):
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
    FILENAME = 'datos_xy.csv'

    # ==================== SERIAL ====================
    ser = serial.Serial(PORT, BAUD, timeout=1)

    # ==================== VARIABLES ====================
    tdata = deque(maxlen=WINDOW_SIZE)
    xdata = deque(maxlen=WINDOW_SIZE)
    ydata = deque(maxlen=WINDOW_SIZE)
    grabando = False
    start_time = None
    datos_guardados = []

    # ==================== FIGURA ====================
    fig, ax = plt.subplots()
    lineX, = ax.plot([], [], label='X (m)', color='blue')
    lineY, = ax.plot([], [], label='Y (m)', color='red')

    ax.set_xlim(0, 10)     # se ajustará dinámicamente
    ax.set_ylim(0, 0.25)
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Posición [m]")
    ax.set_title("X(t) y Y(t) en tiempo real")
    ax.legend()
    ax.grid(True)

# ==================== FUNCIONES ====================
def update(frame):
    global start_time, grabando
    VENTANA = 20
    while ser.in_waiting:
        try:
            line = ser.readline().decode().strip()
            if ',' in line:
                x, y = map(float, line.split(','))
                if grabando:
                    if start_time is None:
                        start_time = time.time()
                    t = time.time() - start_time
                    tdata.append(t)
                    xdata.append(x)
                    ydata.append(y)
                    datos_guardados.append((t, x, y))
        except:
            pass

    if len(tdata) > 0:
        ax.set_xlim(max(0, tdata[-1]-VENTANA), tdata[-1]+1)  # ventana móvil de 10s
        lineX.set_data(tdata, xdata)
        lineY.set_data(tdata, ydata)

    return lineX, lineY

def on_key(event):
    global grabando, datos_guardados, start_time

    if event.key.lower() == 'r':
        print("▶️ Grabación iniciada...")
        grabando = True
        datos_guardados = []
        start_time = None

    elif event.key.lower() == 'f':
        if grabando:
            grabando = False
            print("⏹ Grabación detenida.")
            if datos_guardados:
                print(f"💾 Guardando {len(datos_guardados)} muestras en '{FILENAME}'...")
                with open(FILENAME, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["Tiempo [s]", "X [m]", "Y [m]"])
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
