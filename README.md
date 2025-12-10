<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&height=180&section=header&text=PhantomX%20Pincher%20X100%20%E2%80%A2%20ROS%202&fontSize=32&desc=Pr%C3%A1ctica%20de%20Laboratorio%20%E2%80%A2%20Rob%C3%B3tica%20Industrial&descSize=14&animation=fadeIn" width="100%" />
</div>

---

# 🤖 PhantomX Pincher X100 — ROS 2 Humble · RViz

> **Resumen:** Práctica de laboratorio del curso *Robótica * orientada a la creación de Joint Controllers con ROS 2 para manipular servomotores Dynamixel AX-12 del robot **PhantomX Pincher X100**, incluyendo el desarrollo de una interfaz gráfica completa para control manual, movimiento por coordenadas cartesianas y visualización en RViz.

---

## 🧾 Autores

- **Samuel David Sánchez Cárdenas** — Desarrollo, simulación y documentación.  
  [![GitHub samsanchezcar](https://img.shields.io/badge/GitHub-%40samsanchezcar-181717?style=for-the-badge&logo=github)](https://github.com/samsanchezcar)
- **Santiago Ávila Corredor** — Diseño de trayectorias, pruebas y documentación.  
  [![GitHub Santiago Ávila](https://img.shields.io/badge/GitHub-Search--Santiago%20%C3%81vila-181717?style=for-the-badge&logo=github)](https://github.com/search?q=Santiago+Avila)

---

## 📂 Estructura del repositorio

```text
ros2_ws/
    └── phantom_ws/
        ├── build/
        ├── install/
        ├── log/
        └── src/
            ├── phantomx_pincher_bringup/
            ├── phantomx_pincher_commander_cpp/
            ├── phantomx_pincher_demos/
            ├── phantomx_pincher_description/
            │   └── urdf/
            │       └── phantomx_pincher.urdf
            ├── phantomx_pincher_interfaces/
            ├── phantomx_pincher_moveit_config/
            └── pincher_control/
                └── pincher_control/
                    ├── __init__.py
                    ├── control_servo.py
                    ├── LAB5_P1.py
                    ├── terminal_control.py
                    ├── terminal_subscriber.py
                    └── toolbox.py
sources/
    ├── DH.png
    └── Matlab_vis.gif
DH.mlx
```

---

## 📋 Tabla de Contenidos

1. [Introducción](#introducción)  
2. [Objetivos](#objetivos)  
3. [Cinemática Directa - Parámetros DH](#cinemática-directa---parámetros-dh)  
4. [Movimiento Home y Posición Objetivo](#movimiento-home-y-posición-objetivo)  
5. [Control por Terminal - Publisher](#control-por-terminal---publisher)  
6. [Suscriptor de Estados Articulares](#suscriptor-de-estados-articulares)  
7. [Visualización con Robotics Toolbox](#visualización-con-robotics-toolbox)  
8. [Interfaz Gráfica de Usuario (GUI)](#interfaz-gráfica-de-usuario-gui)  
9. [Conclusiones](#conclusiones)  
10. [Referencias](#referencias)  

---

## 📖 Introducción

La integración de robots manipuladores con sistemas operativos robóticos (ROS) representa un pilar fundamental en la robótica moderna. Esta práctica se centra en el robot **PhantomX Pincher X100**, un manipulador de 4 grados de libertad equipado con servomotores **Dynamixel AX-12**, ampliamente utilizado en entornos académicos y de investigación.

El laboratorio abarca desde el análisis cinemático del manipulador hasta la implementación de una interfaz gráfica completa que permite:

- Control articular mediante sliders y valores numéricos.
- Movimiento en el espacio cartesiano (X, Y, Z) mediante cinemática inversa.
- Visualización en tiempo real con RViz y Robotics Toolbox.
- Comunicación bidireccional mediante tópicos y servicios de ROS 2.

La práctica integra conocimientos de cinemática, programación en Python, comunicación con hardware mediante el SDK de Dynamixel, y el ecosistema de ROS 2 Humble para crear un sistema de control robusto y modular.

---

## 🎯 Objetivos

- Crear todos los **Joint Controllers** con ROS para manipular servomotores **Dynamixel AX-12** del robot PhantomX Pincher.
- Manipular los **tópicos de estado y comando** para todos los Joint Controllers del robot PhantomX Pincher.
- Manipular los **servicios** para todos los Joint Controllers del robot PhantomX Pincher.
- Conectar el robot PhantomX Pincher con **Python** usando **ROS 2**.

---

## 📐 Cinemática Directa - Parámetros DH

El análisis cinemático del PhantomX Pincher X100 se realizó utilizando la convención de **Denavit-Hartenberg (DH)**. Este proceso involucró la identificación de los sistemas coordenados de cada articulación y el cálculo de los parámetros correspondientes.

### Ubicación de Ejes Coordenados

<div align="center">
  <img src="./sources/DH.png" alt="Ejes coordenados DH" width="600" />
  <p><em>Asignación de sistemas coordenados según la convención Denavit-Hartenberg para el PhantomX Pincher.</em></p>
</div>

### Parámetros DH del Manipulador

| Articulación | θ (rad) | d (m) | a (m) | α (rad) |
|:------------:|:-------:|:-----:|:-----:|:-------:|
| 1 | q₁ | 0.044 | 0 | π/2 |
| 2 | q₂ + π/2 | 0 | 0.1075 | 0 |
| 3 | q₃ | 0 | 0.1075 | 0 |
| 4 | q₄ | 0 | 0.0753 | 0 |

### Implementación en MATLAB

El archivo `DH.mlx` contiene la implementación del modelo cinemático directo utilizando el Robotics Toolbox de Peter Corke para MATLAB. Esta implementación permite:

- Calcular la posición y orientación del efector final dado un conjunto de ángulos articulares.
- Visualizar el robot en diferentes configuraciones.
- Validar los parámetros DH obtenidos analíticamente.

<div align="center">
  <img src="./sources/Matlab_vis.gif" alt="Visualización MATLAB" width="500" />
  <p><em>Visualización del modelo cinemático en MATLAB usando el Robotics Toolbox.</em></p>
</div>

---

## 🏠 Movimiento Home y Posición Objetivo

El script `LAB5_P1.py` implementa una secuencia de movimientos que lleva el robot desde la posición **Home** (todos los ángulos en 0°) hasta una configuración objetivo definida, moviendo cada articulación de forma secuencial.

### Descripción Funcional

El nodo `SecuenciaLab5` utiliza el controlador de motores `PincherController` para:

1. Inicializar la comunicación con los servomotores.
2. Llevar el robot a la posición Home.
3. Calcular las posiciones objetivo en valores Dynamixel.
4. Mover secuencialmente cada articulación (waist → shoulder → elbow → wrist).

**Ubicación del archivo:** `ros2_ws/phantom_ws/src/pincher_control/pincher_control/LAB5_P1.py`

### Diagrama de Flujo

```mermaid
flowchart TD
    subgraph INIT["Inicialización"]
        A0[START]
        A1[Inicializar rclpy]
        A2[Crear nodo SecuenciaLab5]
        A3[Crear PincherController]
        A4[Iniciar hilo de spin para ROS2]
    end

    subgraph HOME["Ir a Home"]
        B0[Llamar home_all_motors_sec]
        B1[Esperar 3 segundos]
    end

    subgraph CONFIG["Configurar Objetivo"]
        C0[Definir ángulos objetivo en radianes]
        C1["q_waist = 0.5 rad"]
        C2["q_shoulder = -0.3 rad"]
        C3["q_elbow = 0.6 rad"]
        C4["q_wrist = 0.6 rad"]
        C5[Convertir radianes a valores Dynamixel]
    end

    subgraph MOVE["Movimiento Secuencial"]
        D0[Mover WAIST al objetivo]
        D1[Esperar 2 segundos]
        D2[Mover SHOULDER al objetivo]
        D3[Esperar 2 segundos]
        D4[Mover ELBOW al objetivo]
        D5[Esperar 2 segundos]
        D6[Mover WRIST al objetivo]
        D7[Esperar 2 segundos]
        D8[Secuencia terminada]
    end

    subgraph LOOP["Bucle Principal"]
        E0{¿Continuar?}
        E1[Ejecutar secuencia nuevamente]
    end

    A0 --> A1 --> A2 --> A3 --> A4 --> B0
    B0 --> B1 --> C0
    C0 --> C1 --> C2 --> C3 --> C4 --> C5
    C5 --> D0 --> D1 --> D2 --> D3 --> D4 --> D5 --> D6 --> D7 --> D8
    D8 --> E0
    E0 -->|Sí| E1 --> B0
    E0 -->|No / Ctrl+C| F0[Cerrar controlador]
    F0 --> F1[Shutdown ROS2]
    F1 --> F2[END]
```

### Video de Implementación

<div align="center">

<!-- TODO: Reemplazar VIDEO_ID por el ID real de YouTube -->
[![Movimiento Home a Objetivo](https://img.youtube.com/vi/VIDEO_ID_HOME/0.jpg)](https://youtu.be/VIDEO_ID_HOME)

**Demostración del movimiento secuencial desde Home hasta la posición objetivo.**

</div>

---

## ⌨️ Control por Terminal - Publisher

El script `terminal_control.py` permite controlar el robot PhantomX Pincher desde la línea de comandos, publicando posiciones articulares directamente a los motores.

### Descripción Funcional

Este nodo ofrece dos modos de operación:

1. **Control Individual:** Especificar un motor y su ángulo objetivo en grados.
2. **Configuraciones Predefinidas:** Seleccionar entre 5 poses preconfiguradas (c1-c5).

**Ubicación del archivo:** `ros2_ws/phantom_ws/src/pincher_control/pincher_control/terminal_control.py`

### Configuraciones Predefinidas

| Config | Waist | Shoulder | Elbow | Wrist | Gripper | Descripción |
|:------:|:-----:|:--------:|:-----:|:-----:|:-------:|:-----------:|
| c1 | 0° | 0° | 0° | 0° | 0° | HOME |
| c2 | 25° | 25° | 20° | -20° | 0° | Alcance Medio |
| c3 | -35° | 35° | -30° | 30° | 0° | Lateral |
| c4 | 85° | -20° | 55° | 25° | 0° | Elevada |
| c5 | 80° | -35° | 55° | -45° | 0° | Extendida |

### Diagrama de Flujo

```mermaid
flowchart TD
    subgraph INIT["Inicialización"]
        A0[START]
        A1[Inicializar rclpy]
        A2[Crear PincherController]
        A3[Iniciar hilo de spin]
        A4[Mostrar información inicial]
    end

    subgraph MAIN["Bucle Principal"]
        B0[Esperar entrada del usuario]
        B1{¿Comando vacío?}
        B2{¿Comando = quit?}
        B3{¿Comando = help?}
        B4{¿Es configuración c1-c5?}
        B5{¿Formato válido: ID ángulo?}
    end

    subgraph PRESET["Configuración Predefinida"]
        C0[Obtener ángulos de la configuración]
        C1[Enviar configuración completa]
        C2[Para cada motor: convertir y enviar]
    end

    subgraph SINGLE["Control Individual"]
        D0[Parsear ID y ángulo]
        D1{¿ID válido?}
        D2{¿Ángulo en límites?}
        D3[Convertir grados a radianes]
        D4[Aplicar signo del motor]
        D5[Convertir a valor Dynamixel]
        D6[Enviar comando al motor]
        D7[Mostrar confirmación]
    end

    subgraph ERROR["Manejo de Errores"]
        E0[Mostrar error de formato]
        E1[Mostrar error de ID]
        E2[Mostrar error de límites]
    end

    A0 --> A1 --> A2 --> A3 --> A4 --> B0
    B0 --> B1
    B1 -->|Sí| B0
    B1 -->|No| B2
    B2 -->|Sí| F0[Cerrar y salir]
    B2 -->|No| B3
    B3 -->|Sí| G0[Mostrar ayuda] --> B0
    B3 -->|No| B4
    B4 -->|Sí| C0 --> C1 --> C2 --> B0
    B4 -->|No| B5
    B5 -->|No| E0 --> B0
    B5 -->|Sí| D0 --> D1
    D1 -->|No| E1 --> B0
    D1 -->|Sí| D2
    D2 -->|No| E2 --> B0
    D2 -->|Sí| D3 --> D4 --> D5 --> D6 --> D7 --> B0

    F0 --> F1[Cerrar controlador]
    F1 --> F2[Shutdown ROS2]
    F2 --> F3[END]
```

### Video de Implementación

<div align="center">

<!-- TODO: Reemplazar VIDEO_ID por el ID real de YouTube -->
[![Control por Terminal](https://img.youtube.com/vi/VIDEO_ID_TERMINAL/0.jpg)](https://youtu.be/VIDEO_ID_TERMINAL)

**Demostración del control por terminal con comandos individuales y configuraciones predefinidas.**

</div>

---

## 👁️ Suscriptor de Estados Articulares

El script `terminal_subscriber.py` implementa un nodo suscriptor que muestra en tiempo real los estados de las articulaciones del robot.

### Descripción Funcional

Este nodo se suscribe al tópico `/joint_states` y convierte las posiciones de radianes a grados para una visualización más intuitiva.

**Ubicación del archivo:** `ros2_ws/phantom_ws/src/pincher_control/pincher_control/terminal_subscriber.py`

### Diagrama de Flujo

```mermaid
flowchart TD
    subgraph INIT["Inicialización"]
        A0[START]
        A1[Inicializar rclpy]
        A2[Crear nodo JointStatePrinter]
        A3[Crear suscripción a /joint_states]
        A4[Inicializar diccionario de ángulos]
    end

    subgraph CALLBACK["Callback de Joint States"]
        B0[Recibir mensaje JointState]
        B1[Para cada articulación en el mensaje]
        B2{¿Nombre en diccionario?}
        B3[Convertir radianes a grados]
        B4[Actualizar valor en diccionario]
        B5[Formatear cadena de salida]
        B6[Imprimir estados actuales]
    end

    subgraph SPIN["Bucle de Spin"]
        C0[rclpy.spin - esperar mensajes]
        C1{¿Interrupción?}
    end

    A0 --> A1 --> A2 --> A3 --> A4 --> C0
    C0 --> B0
    B0 --> B1 --> B2
    B2 -->|Sí| B3 --> B4 --> B1
    B2 -->|No| B1
    B1 -->|Completado| B5 --> B6 --> C0
    C0 --> C1
    C1 -->|Sí| D0[Destruir nodo]
    D0 --> D1[Shutdown ROS2]
    D1 --> D2[END]
    C1 -->|No| C0
```

### Video de Implementación

<div align="center">

<!-- TODO: Reemplazar VIDEO_ID por el ID real de YouTube -->
[![Suscriptor de Estados](https://img.youtube.com/vi/VIDEO_ID_SUB/0.jpg)](https://youtu.be/VIDEO_ID_SUB)

**Visualización en tiempo real de los estados articulares mediante el suscriptor.**

</div>

---

## 🔧 Visualización con Robotics Toolbox

El script `toolbox.py` proporciona una visualización 3D en tiempo real del robot utilizando el **Robotics Toolbox de Peter Corke** para Python.

### Descripción Funcional

Este nodo combina ROS 2 con matplotlib para mostrar el modelo cinemático del robot actualizándose en tiempo real según los datos del tópico `/joint_states`.

**Ubicación del archivo:** `ros2_ws/phantom_ws/src/pincher_control/pincher_control/toolbox.py`

### Diagrama de Flujo

```mermaid
flowchart TD
    subgraph INIT["Inicialización"]
        A0[START]
        A1[Inicializar rclpy]
        A2[Construir modelo DH del robot]
        A3[Crear nodo PincherVisualizer]
        A4[Suscribirse a /joint_states]
        A5[Configurar matplotlib interactivo]
        A6[Crear timer de actualización 10Hz]
    end

    subgraph MODEL["Modelo DH"]
        M0["L1 = 44.0 mm"]
        M1["L2 = 107.5 mm"]
        M2["L3 = 107.5 mm"]
        M3["L4 = 75.3 mm"]
        M4[Crear links RevoluteDH]
        M5[Configurar herramienta TCP]
    end

    subgraph CALLBACK["Callback Joint States"]
        B0[Recibir mensaje JointState]
        B1{¿Tiene >= 4 posiciones?}
        B2[Actualizar current_q con primeras 4]
    end

    subgraph VIS["Actualización Visual"]
        C0[Timer dispara update_visualization]
        C1{¿Primera vez?}
        C2[Crear figura con robot.plot]
        C3[Actualizar robot.q]
        C4[Llamar fig.step]
        C5[plt.pause para refrescar]
    end

    subgraph SPIN["Bucle Principal"]
        D0[rclpy.spin]
        D1{¿Interrupción?}
    end

    A0 --> A1 --> A2
    A2 --> M0 --> M1 --> M2 --> M3 --> M4 --> M5
    M5 --> A3 --> A4 --> A5 --> A6 --> D0

    D0 --> B0
    B0 --> B1
    B1 -->|No| D0
    B1 -->|Sí| B2 --> D0

    A6 -.->|Timer| C0
    C0 --> C1
    C1 -->|Sí| C2 --> C5
    C1 -->|No| C3 --> C4 --> C5
    C5 --> D0

    D0 --> D1
    D1 -->|No| D0
    D1 -->|Sí| E0[Destruir nodo]
    E0 --> E1[Cerrar figuras]
    E1 --> E2[Shutdown ROS2]
    E2 --> E3[END]
```

### Video de Implementación

<div align="center">

<!-- TODO: Reemplazar VIDEO_ID por el ID real de YouTube -->
[![Visualización Toolbox](https://img.youtube.com/vi/VIDEO_ID_TOOLBOX/0.jpg)](https://youtu.be/VIDEO_ID_TOOLBOX)

**Visualización 3D en tiempo real con Robotics Toolbox de Peter Corke.**

</div>

---

## 🖥️ Interfaz Gráfica de Usuario (GUI)

El script `control_servo.py` implementa una interfaz gráfica completa utilizando **PyQt5** que integra todas las funcionalidades de control del robot.

### Descripción Funcional

La GUI ofrece múltiples páginas de control:

1. **Panel Principal:** Vista general del estado del robot y control de velocidad.
2. **Control Manual:** Sliders para cada articulación.
3. **Valores Fijos:** Entrada numérica directa para cada motor.
4. **Ángulos Predefinidos:** 5 poses preconfiguradas con un clic.
5. **Control XYZ:** Movimiento en coordenadas cartesianas con cinemática inversa.
6. **Visualización:** Lanzadores para RViz y Robotics Toolbox.
7. **Información:** Datos del proyecto y autores.

**Ubicación del archivo:** `ros2_ws/phantom_ws/src/pincher_control/pincher_control/control_servo.py`

### Captura de Pantalla de la GUI

<div align="center">

<!-- TODO: Agregar captura de pantalla de la GUI -->
<img src="./sources/gui_screenshot.png" alt="Interfaz Gráfica" width="800" />
<p><em>Interfaz gráfica moderna para el control del PhantomX Pincher X100.</em></p>

</div>

### Diagrama de Flujo - Sistema Principal

```mermaid
flowchart TD
    subgraph INIT["Inicialización del Sistema"]
        A0[START]
        A1[Inicializar rclpy]
        A2[Crear PincherController]
        A3[Iniciar hilo de spin ROS2]
        A4[Crear aplicación PyQt5]
        A5[Aplicar stylesheet moderno]
        A6[Crear ModernPincherGUI]
        A7[Mostrar ventana principal]
    end

    subgraph CONTROLLER["PincherController - Nodo ROS2"]
        B0[Abrir puerto serial]
        B1[Configurar baudrate 1M]
        B2[Inicializar PacketHandler]
        B3[Crear publisher /joint_states]
        B4[Crear timer 10Hz para publicar]
        B5[Habilitar torque en motores]
        B6[Configurar velocidad inicial]
    end

    subgraph GUI["Interfaz Gráfica"]
        C0[Crear sidebar con menú]
        C1[Crear páginas del stack]
        C2[Conectar señales y slots]
        C3[Iniciar timer XYZ 200ms]
    end

    subgraph PAGES["Páginas de la GUI"]
        P0[Dashboard - Estado general]
        P1[Control Manual - Sliders]
        P2[Valores Fijos - Entradas numéricas]
        P3[Ángulos Predefinidos - 5 poses]
        P4[Control XYZ - Cinemática inversa]
        P5[Visualización - RViz/Toolbox]
        P6[Información - Autores]
    end

    A0 --> A1 --> A2 --> B0
    B0 --> B1 --> B2 --> B3 --> B4 --> B5 --> B6
    B6 --> A3 --> A4 --> A5 --> A6
    A6 --> C0 --> C1 --> C2 --> C3
    C1 --> P0 & P1 & P2 & P3 & P4 & P5 & P6
    C3 --> A7 --> D0[Event loop PyQt5]
```

### Diagrama de Flujo - Control XYZ (Cinemática Inversa)

```mermaid
flowchart TD
    subgraph INPUT["Entrada de Usuario"]
        A0[Usuario ingresa X, Y, Z]
        A1[Clic en MOVER A POSICIÓN XYZ]
    end

    subgraph VALIDATE["Validación de Alcance"]
        B0[Calcular r = sqrt de x² + y²]
        B1{¿r en rango válido?}
        B2{¿z en rango válido?}
        B3[Mostrar error de alcance radial]
        B4[Mostrar error de alcance Z]
    end

    subgraph IK["Cinemática Inversa"]
        C0[Construir transformación objetivo T_target]
        C1[Obtener q_current como semilla]
        C2[Definir semillas adicionales]
        C3[Para cada semilla]
        C4[Ejecutar ikine_LM]
        C5{¿Solución exitosa?}
        C6[Calcular error de posición]
        C7{¿Error < mejor_error?}
        C8[Guardar como mejor solución]
        C9{¿Error < 0.003m?}
    end

    subgraph MOVE["Movimiento"]
        D0{¿Se encontró solución?}
        D1[Mostrar error IK]
        D2[Llamar move_to_joint_angles]
        D3[Para cada motor 1-4]
        D4[Aplicar signo del motor]
        D5[Saturar a límites articulares]
        D6[Convertir a valor Dynamixel]
        D7[Enviar comando al motor]
        D8[Actualizar estado GUI]
    end

    A0 --> A1 --> B0 --> B1
    B1 -->|No| B3 --> END1[FIN - Error]
    B1 -->|Sí| B2
    B2 -->|No| B4 --> END1
    B2 -->|Sí| C0 --> C1 --> C2 --> C3 --> C4 --> C5
    C5 -->|No| C3
    C5 -->|Sí| C6 --> C7
    C7 -->|Sí| C8 --> C9
    C7 -->|No| C3
    C9 -->|Sí| D0
    C9 -->|No| C3
    C3 -->|Todas probadas| D0
    D0 -->|No| D1 --> END2[FIN - Sin solución]
    D0 -->|Sí| D2 --> D3 --> D4 --> D5 --> D6 --> D7 --> D3
    D3 -->|Completado| D8 --> END3[FIN - Éxito]
```

### Diagrama de Flujo - Publicación de Joint States

```mermaid
flowchart TD
    subgraph TIMER["Timer 10Hz"]
        A0[Timer dispara publish_joint_states]
    end

    subgraph CREATE["Crear Mensaje"]
        B0[Crear JointState]
        B1[Crear Header con timestamp]
        B2[Asignar frame_id = base_link]
        B3[Asignar nombres de articulaciones]
        B4[Asignar posiciones actuales]
    end

    subgraph PUBLISH["Publicar"]
        C0[joint_state_pub.publish]
        C1[Mensaje disponible en /joint_states]
    end

    A0 --> B0 --> B1 --> B2 --> B3 --> B4 --> C0 --> C1
```

### Video de Implementación

<div align="center">

<!-- TODO: Reemplazar VIDEO_ID por el ID real de YouTube -->
[![GUI Control Completo](https://img.youtube.com/vi/VIDEO_ID_GUI/0.jpg)](https://youtu.be/VIDEO_ID_GUI)

**Demostración completa de la interfaz gráfica con todas sus funcionalidades.**

</div>

---

## 🎓 Conclusiones

1. **Integración ROS 2 - Hardware:** La comunicación exitosa entre ROS 2 Humble y los servomotores Dynamixel AX-12 mediante el SDK de Dynamixel demuestra la viabilidad de crear sistemas de control robótico modulares y escalables.

2. **Arquitectura Pub/Sub:** El patrón publicador-suscriptor de ROS 2 facilitó la separación de responsabilidades entre el control de motores (`PincherController`), la visualización (`toolbox.py`, RViz) y la interfaz de usuario (GUI), permitiendo que cada componente opere de forma independiente.

3. **Cinemática Inversa:** La implementación del algoritmo de Levenberg-Marquardt para la cinemática inversa, junto con múltiples semillas de inicialización, permitió alcanzar posiciones cartesianas con errores menores a 3 mm en la mayoría de los casos dentro del espacio de trabajo alcanzable.

4. **Interfaz de Usuario:** El desarrollo de una GUI moderna con PyQt5 demostró que es posible crear herramientas de control intuitivas que abstraen la complejidad del sistema ROS 2 subyacente, facilitando la operación por usuarios no expertos.

5. **Visualización en Tiempo Real:** La integración con RViz y el Robotics Toolbox de Peter Corke proporcionó retroalimentación visual inmediata del estado del robot, crucial para la validación de movimientos y la detección de errores de configuración.

6. **Modularidad del Código:** La estructura del paquete `pincher_control` con scripts independientes para cada funcionalidad (terminal_control, terminal_subscriber, toolbox, control_servo) facilita el mantenimiento, pruebas y extensión futura del sistema.

7. **Parámetros DH:** La correcta identificación de los parámetros Denavit-Hartenberg y su implementación tanto en MATLAB como en Python fue fundamental para la coherencia entre el modelo simulado y el robot físico.

---

## 📚 Referencias

1. **Laboratorio No. 05 - Pincher Phantom X100 - ROS Humble - RViz.** Universidad Nacional de Colombia, 2025.

2. ROBOTIS. *DYNAMIXEL SDK Manual.* Documentación oficial para comunicación con servomotores Dynamixel.

3. Corke, P. *Robotics, Vision and Control: Fundamental Algorithms in MATLAB.* Springer, 2017.

4. Open Robotics. *ROS 2 Humble Documentation.* https://docs.ros.org/en/humble/

5. Trossen Robotics. *PhantomX Pincher Robot Arm Assembly Guide.*

6. Qt Company. *PyQt5 Documentation.* https://www.riverbankcomputing.com/static/Docs/PyQt5/

---

<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&height=100&section=footer&text=Universidad%20Nacional%20de%20Colombia&fontSize=20&animation=fadeIn" width="100%" />
</div>
