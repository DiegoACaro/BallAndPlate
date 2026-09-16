import numpy as np

forma = 3
#1 circulo
#2 Infinito
#3cuadrado discreto
#4cuadrado continuo
#5roseta
#6 elipse

# --- Parámetros físicos ---
width_mm = 176
height_mm = 133
touch_min_X = 120
touch_max_X = 930
touch_min_Y = 160
touch_max_Y = 870

#CIRCULO
if forma == 1:


    # Conversión mm -> unidades del sensor
    scale_x = (touch_max_X - touch_min_X) / width_mm
    scale_y = (touch_max_Y - touch_min_Y) / height_mm

    # Centro del touch en unidades del sensor
    center_x = (touch_max_X + touch_min_X) / 2
    center_y = (touch_max_Y + touch_min_Y) / 2

    # ------------------ Trayectoria circular ------------------
    radio_mm = 100  # radio físico del círculo
    n_points = 500  # puntos de la trayectoria

    # Ángulos en radianes
    theta = np.linspace(0, 2*np.pi, n_points)

    # Coordenadas en mm (centro en 0,0)
    x_mm = radio_mm * np.cos(theta)
    y_mm = radio_mm * np.sin(theta)

    # Convertir a coordenadas del sensor
    x_touch = center_x + x_mm * scale_x
    y_touch = center_y + y_mm * scale_y

    # ------------------ Imprimir como código C++ ------------------
    print("float setpointX_array_elipse[N_POINTS_ELIPSE] = {", ", ".join(f"{x:.1f}" for x in x_touch), "};")
    print("float setpointY_array_elipse[N_POINTS_ELIPSE] = {", ", ".join(f"{y:.1f}" for y in y_touch), "};")

#INFINITO
elif forma == 2:

    # Conversión mm -> unidades del sensor
    scale_x = (touch_max_X - touch_min_X) / width_mm
    scale_y = (touch_max_Y - touch_min_Y) / height_mm

    # Centro del touch en unidades del sensor
    center_x = (touch_max_X + touch_min_X) / 2
    center_y = (touch_max_Y + touch_min_Y) / 2

    # ------------------ Trayectoria en forma de infinito ------------------
    a = 80  # “radio” de la figura en mm
    n_points = 500

    # Parámetro t
    t = np.linspace(0, 2*np.pi, n_points)

    # Ecuación paramétrica de la lemniscata (forma de ∞)
    x_mm = (a * np.sqrt(2) * np.cos(t)) / (np.sin(t)**2 + 1)
    #y_mm = (a * np.sqrt(2) * np.cos(t) * np.sin(t)) / (np.sin(t)**2 + 1)

    stretch_y = 2  # aumenta la altura (1.0 = igual, >1 = más alto)
    y_mm = stretch_y * (a * np.sqrt(2) * np.cos(t) * np.sin(t)) / (np.sin(t)**2 + 1)


    # Convertir a coordenadas del sensor
    x_touch = center_x + x_mm * scale_x
    y_touch = center_y + y_mm * scale_y

    # ------------------ Imprimir como código C++ ------------------
    print(f"#define N_POINTS_INFINITO {n_points}")
    print("float setpoint_infX[N_POINTS_INFINITO] = {", ", ".join(f"{x:.1f}" for x in x_touch), "};")
    print("float setpoint_infY[N_POINTS_INFINITO] = {", ", ".join(f"{y:.1f}" for y in y_touch), "};")

#CUADRADO DISCRETO
elif forma == 3:


    # --- Parámetros del cuadrado ---
    lado = 220          # tamaño del cuadrado
    N_POINTS = 2000     # total de puntos en el array

    Xmax = 800
    Ymax = 740
    Xmin = 270
    Ymin = 240

    # --- Cuatro vértices (en orden horario) ---
    x_vertices = [Xmin, Xmin, Xmax, Xmax]
    y_vertices = [Ymin, Ymax, Ymax, Ymin]

    # --- Repetir cada vértice el mismo número de veces ---
    points_per_vertex = N_POINTS // len(x_vertices)

    x_touch = np.repeat(x_vertices, points_per_vertex)
    y_touch = np.repeat(y_vertices, points_per_vertex)

    # Si N_POINTS no es múltiplo exacto de 4, ajusta longitud
    x_touch = np.pad(x_touch, (0, N_POINTS - len(x_touch)), mode='edge')
    y_touch = np.pad(y_touch, (0, N_POINTS - len(y_touch)), mode='edge')

    # --- Exportar como código C++ ---
    print(f"#define N_POINTS_CUADRADO {N_POINTS}")
    print("float setpointX_array2[N_POINTS_CUADRADO] = {", ", ".join(f"{x:.1f}" for x in x_touch), "};")
    print("float setpointY_array2[N_POINTS_CUADRADO] = {", ", ".join(f"{y:.1f}" for y in y_touch), "};")

#CUADRADO CONTINUO
elif forma == 4:

    # --- Parámetros del cuadrado ---
    lado = 140          # tamaño del cuadrado (en unidades del sensor)
    N_POINTS = 500     # puntos totales del recorrido

    Xmax = touch_max_X - lado
    Ymax = touch_max_Y - lado
    Xmin = touch_min_X + lado
    Ymin = touch_min_Y + lado

    # --- Cuatro vértices del cuadrado (en sentido horario) ---
    vertices = [
        (Xmin, Ymin),  # esquina inferior izquierda
        (Xmin, Ymax),  # esquina superior izquierda
        (Xmax, Ymax),  # esquina superior derecha
        (Xmax, Ymin),  # esquina inferior derecha
        (Xmin, Ymin)   # regreso al inicio (cierre del bucle)
    ]

    # --- Generar puntos a lo largo de los 4 lados ---
    points_per_side = N_POINTS // (len(vertices) - 1)
    x_touch = []
    y_touch = []

    for i in range(len(vertices) - 1):
        x0, y0 = vertices[i]
        x1, y1 = vertices[i + 1]
        x_side = np.linspace(x0, x1, points_per_side, endpoint=False)
        y_side = np.linspace(y0, y1, points_per_side, endpoint=False)
        x_touch.extend(x_side)
        y_touch.extend(y_side)

    # --- Ajuste final: cerrar el cuadrado ---
    x_touch.append(vertices[0][0])
    y_touch.append(vertices[0][1])

    # --- Convertir a arrays numpy ---
    x_touch = np.array(x_touch)
    y_touch = np.array(y_touch)

    # --- Asegurar longitud exacta ---
    if len(x_touch) > N_POINTS:
        x_touch = x_touch[:N_POINTS]
        y_touch = y_touch[:N_POINTS]

    # --- Exportar como código C++ ---
    print(f"#define N_POINTS_CUADRADO {N_POINTS}")
    print("float setpointX_array2[N_POINTS_CUADRADO] = {", ", ".join(f"{x:.1f}" for x in x_touch), "};")
    print("float setpointY_array2[N_POINTS_CUADRADO] = {", ", ".join(f"{y:.1f}" for y in y_touch), "};")

#ROSETA
elif forma == 5:


    # Conversión mm -> unidades del sensor
    scale_x = (touch_max_X - touch_min_X) / width_mm
    scale_y = (touch_max_Y - touch_min_Y) / height_mm

    # Centro del touch en unidades del sensor
    center_x = (touch_max_X + touch_min_X) / 2
    center_y = (touch_max_Y + touch_min_Y) / 2

    # ------------------ Curva de Lissajous ------------------
    N_POINTS = 500          # número de puntos
    A_mm = 30               # amplitud en mm (ajusta tamaño)
    B_mm = 30
    a = 2                   # frecuencia X
    b = 3                   # frecuencia Y
    delta = np.pi / 2       # desfase entre ejes

    t = np.linspace(0, 2*np.pi, N_POINTS)

    # Coordenadas en mm
    x_mm = A_mm * np.sin(a * t + delta)
    y_mm = B_mm * np.sin(b * t)

    # Convertir a coordenadas del sensor
    x_touch = center_x + x_mm * scale_x
    y_touch = center_y + y_mm * scale_y

    # ------------------ Imprimir como código C++ ------------------
    print("float setpointX_array[N_POINTS] = {", ", ".join(f"{x:.1f}" for x in x_touch), "};")
    print("float setpointY_array[N_POINTS] = {", ", ".join(f"{y:.1f}" for y in y_touch), "};")

#ELIPSE
elif forma == 6:


    # ------------------ Puntos dados ------------------
    p_top    = np.array([500, 240])
    p_right  = np.array([840, 485])
    p_bottom = np.array([490, 740])
    p_left   = np.array([220, 500])

    # ------------------ Centro y semiejes ------------------
    center = (p_top + p_bottom + p_left + p_right) / 4.0

    # eje mayor (horizontal aproximado)
    a = np.linalg.norm(p_right - p_left) / 2.0
    # eje menor (vertical aproximado)
    b = np.linalg.norm(p_top - p_bottom) / 2.0

    # Calcular ángulo de rotación de la elipse
    dx = p_right[0] - p_left[0]
    dy = p_right[1] - p_left[1]
    phi = np.arctan2(dy, dx)  # en radianes

    # ------------------ Generar puntos de la elipse ------------------
    n_points = 500
    theta = np.linspace(0, 2*np.pi, n_points)

    # Ecuación paramétrica de la elipse rotada
    x = center[0] + a*np.cos(theta)*np.cos(phi) - b*np.sin(theta)*np.sin(phi)
    y = center[1] + a*np.cos(theta)*np.sin(phi) + b*np.sin(theta)*np.cos(phi)

    # ------------------ Imprimir como código C++ ------------------
    print("float setpointX_array_elipse[N_POINTS_ELIPSE] = {", ", ".join(f"{xi:.1f}" for xi in x), "};")
    print("float setpointY_array_elipse[N_POINTS_ELIPSE] = {", ", ".join(f"{yi:.1f}" for yi in y), "};")
