# import serial
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import numpy as np
# import time
# import csv

# # ==================== CONFIG ====================
# PORT = 'COM14'
# BAUD = 115200
# N_NEURONAS = 32
# N_EJES = 2
# VENTANA_TIEMPO = 30  # cantidad de muestras visibles en el heatmap
# FILENAME = 'actividad_neuronas_completa.csv'

# # ==================== SERIAL ====================
# ser = serial.Serial(PORT, BAUD, timeout=1)

# # ==================== VARIABLES ====================
# grabando = False
# start_time = None
# datos_guardados = []

# # 4 bloques: Aux0, AuxD0, Aux1, AuxD1 → total 128 neuronas
# N_BLOQUES = N_EJES * 2
# actividad_tiempo = np.zeros((VENTANA_TIEMPO, N_BLOQUES * N_NEURONAS))

# # ==================== FIGURA ====================
# fig, ax = plt.subplots(figsize=(10, 7))
# im = ax.imshow(
#     actividad_tiempo.T,
#     cmap='plasma',
#     interpolation='nearest',
#     aspect='auto',
#     origin='lower',
#     vmin=-1.0, vmax=1.0
# )

# ax.set_title("Respuesta temporal de las capas Auxiliares en ambos ejes")
# ax.set_xlabel("Tiempo")
# #ax.set_ylabel("Neurona (por bloque)")
# fig.colorbar(im, ax=ax, label='Activación Neuronal')

# # Etiquetas Y para cada bloque
# yticks = np.arange(0, N_BLOQUES * N_NEURONAS, N_NEURONAS)
# etiquetas = []
# etiqueta_eje = ["X", "Y"]
# for eje in range(N_EJES):
#     etiquetas.append(f"Aux Directa eje {etiqueta_eje[eje]}")
#     etiquetas.append(f"Aux Cruzada eje {etiqueta_eje[eje]}")
# ax.set_yticks(yticks)
# ax.set_yticklabels(etiquetas)

# # ==================== FUNCIONES ====================
# def update(frame):
#     global actividad_tiempo, grabando, start_time

#     nuevas = []  # almacenará [Aux0, AuxD0, Aux1, AuxD1]

#     for eje in range(N_EJES):
#         if not ser.in_waiting:
#             return [im]
#         line = ser.readline().decode(errors='ignore').strip()
#         if not line or ',' not in line:
#             return [im]
#         try:
#             valores = list(map(float, line.split(',')))
#         except ValueError:
#             return [im]

#         if len(valores) != N_NEURONAS * 2:
#             return [im]

#         Aux_vals = np.array(valores[0::2])
#         AuxD_vals = np.array(valores[1::2])

#         nuevas.append(Aux_vals)
#         nuevas.append(AuxD_vals)

#     # Concatenar todo: [Aux0, AuxD0, Aux1, AuxD1]
#     if len(nuevas) == N_BLOQUES:
#         fila = np.concatenate(nuevas)

#         # desplazar ventana temporal
#         actividad_tiempo = np.roll(actividad_tiempo, -1, axis=0)
#         actividad_tiempo[-1, :] = fila

#         # actualizar heatmap
#         im.set_data(actividad_tiempo.T)
#         im.set_clim(vmin=np.min(actividad_tiempo), vmax=np.max(actividad_tiempo))

#         # guardar si grabando
#         if grabando:
#             if start_time is None:
#                 start_time = time.time()
#             t = time.time() - start_time
#             datos_guardados.append((t,) + tuple(fila))

#     return [im]


# def on_key(event):
#     global grabando, datos_guardados, start_time
#     if event.key.lower() == 'r':
#         print("▶️ Grabación iniciada...")
#         grabando = True
#         datos_guardados = []
#         start_time = None

#     elif event.key.lower() == 'f':
#         if grabando:
#             grabando = False
#             print("⏹ Grabación detenida.")
#             if datos_guardados:
#                 print(f"💾 Guardando {len(datos_guardados)} muestras en '{FILENAME}'...")
#                 with open(FILENAME, 'w', newline='') as f:
#                     writer = csv.writer(f)
#                     encabezado = ["Tiempo [s]"] + [
#                         f"{label}_N{i}"
#                         for label in [f"Aux{e}" for e in range(N_EJES)] +
#                                       [f"AuxD{e}" for e in range(N_EJES)]
#                         for i in range(N_NEURONAS)
#                     ]
#                     writer.writerow(encabezado)
#                     writer.writerows(datos_guardados)
#                 print("✅ Archivo guardado correctamente.")
#             else:
#                 print("⚠️ No se registraron datos.")


# # ==================== EVENTO DE TECLAS ====================
# fig.canvas.mpl_connect('key_press_event', on_key)

# # ==================== ANIMACIÓN ====================
# ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
# plt.show()






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
FILENAME = 'datos_R.csv'

# ==================== SERIAL ====================
ser = serial.Serial(PORT, BAUD, timeout=1)

# ==================== VARIABLES ====================
tdata = deque(maxlen=WINDOW_SIZE)
R4x_data = deque(maxlen=WINDOW_SIZE)
R5x_data = deque(maxlen=WINDOW_SIZE)
R4y_data = deque(maxlen=WINDOW_SIZE)
R5y_data = deque(maxlen=WINDOW_SIZE)

grabando = False
start_time = None
datos_guardados = []

# ==================== FIGURA ====================
fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

lineR4x, = axs[0].plot([], [], label='R7 X', color='blue')
lineR5x, = axs[0].plot([], [], label='R8 X', color='red')

lineR4y, = axs[1].plot([], [], label='R7 Y', color='green')
lineR5y, = axs[1].plot([], [], label='R8 Y', color='orange')

for ax in axs:
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 0.25)
    ax.grid(True)
    ax.legend()

axs[0].set_ylabel("Valor X")
axs[1].set_ylabel("Valor Y")
axs[1].set_xlabel("Tiempo [s]")
axs[0].set_title("R7 y R8 en tiempo real - X")
axs[1].set_title("R7 y R8 en tiempo real - Y")

# ==================== FUNCIONES ====================
def update(frame):
    global start_time, grabando
    VENTANA = 20
    #while ser.in_waiting:
    try:
        
        line = ser.readline().decode(errors='ignore').strip()
        if line and ',' in line:
            R4x, R5x, R4y, R5y = map(float, line.split(','))
            if grabando:
                if start_time is None:
                    start_time = time.time()
                t = time.time() - start_time
                tdata.append(t)
                R4x_data.append(R4x)
                R5x_data.append(R4x)
                R4y_data.append(R4y)
                R5y_data.append(R5y)
                datos_guardados.append((t, R4x, R5x, R4y, R5y))
    except:
        pass

    if len(tdata) > 0:
        axs[0].set_xlim(max(0, tdata[-1]-VENTANA), tdata[-1]+1)
        axs[1].set_xlim(max(0, tdata[-1]-VENTANA), tdata[-1]+1)

        lineR4x.set_data(tdata, R4x_data)
        lineR5x.set_data(tdata, R5x_data)
        lineR4y.set_data(tdata, R4y_data)
        lineR5y.set_data(tdata, R5y_data)

        # Ajuste automático de límites
        axs[0].relim()
        axs[0].autoscale_view()
        axs[1].relim()
        axs[1].autoscale_view()

    return lineR4x, lineR5x, lineR4y, lineR5y

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
                    writer.writerow(["Tiempo [s]", "R4 X", "R5 X", "R4 Y", "R5 Y"])
                    writer.writerows(datos_guardados)
                print("✅ Archivo guardado correctamente.")
            else:
                print("⚠️ No se registraron datos.")

# ==================== EVENTO DE TECLAS ====================
fig.canvas.mpl_connect('key_press_event', on_key)

# ==================== ANIMACIÓN ====================
ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.show()
