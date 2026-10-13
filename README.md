# Follow Ball with VFF (Virtual Force Field) in ROS 2

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%2F%20Iron-blue.svg)](https://docs.ros.org/)
[![C++](https://img.shields.io/badge/Language-C%2B%2B17-00599C.svg)](https://isocpp.org/)
[![Build Tool: colcon](https://img.shields.io/badge/Build-colcon-orange.svg)](https://colcon.readthedocs.io/)
[![Platform: Kobuki / Gazebo](https://img.shields.io/badge/Platform-Kobuki%20%2F%20Simulation-brightgreen.svg)](https://turtlebot.github.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Aplicación reactiva para **ROS 2** diseñada para la detección, seguimiento y alcance de una pelota de color característico esquivando obstáculos estáticos y dinámicos mediante el algoritmo de campos de fuerzas potenciales **VFF (Virtual Force Field)**.

El sistema desacopla la percepción visual y sensorial mediante nodos independientes que publican vectores atractivos y repulsivos, combinándolos en un nodo central de control reactivo basado en una máquina de estados finitos (FSM) con soporte completo para simulación y robot real (**Kobuki**).

---

## 📌 Tabla de Contenidos
- [Estructura del Paquete](#-estructura-del-paquete)
- [Fundamento Teórico: Algoritmo VFF](#-fundamento-teórico-algoritmo-vff)
- [Arquitectura Software y Nodos](#-arquitectura-software-y-nodos)
  - [Filtro de Color HSV (`hsv_filter`)](#1-filtro-hsv-vector-atractivo-hsv_filter)
  - [Detector de Obstáculos (`obstacle_detector`)](#2-detector-de-obstáculos-vector-repulsivo-obstacle_detector)
  - [Nodo de Control FSM (`control_node`)](#3-nodo-de-control-y-navegación-control_node)
- [Gestión de Parámetros y Remapeos](#-gestión-de-parámetros-y-remapeos)
- [Compilación y Construcción](#-compilación-y-construcción)
- [Guía de Ejecución y Launchers](#-guía-de-ejecución-y-launchers)
- [Autores](#-autores)

---

## 📂 Estructura del Paquete

```text
follow_ball/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── params.yaml              # Configuración base / genérica
│   ├── params.sim.yaml          # Parámetros para entorno de simulación (Gazebo)
│   └── params_kobuki.yaml       # Parámetros calibrados para la plataforma real (Kobuki)
├── include/follow_ball/
│   ├── control_node.hpp         # Declaración del nodo VFF y FSM de seguimiento
│   ├── hsv_filter.hpp           # Procesamiento de imagen OpenCV -> Vector atractivo
│   └── obstacle_detector.hpp    # Procesamiento LiDAR/LaserScan -> Vector repulsivo
├── launch/
│   ├── simulation.launch.py     # Lanzador unificado para entorno de simulación
│   ├── kobuki.launch.py         # Lanzador unificado para robot real Kobuki
│   ├── hsv.filter.launch.py     # Lanzador modular de HSV (real)
│   ├── hsv.filter.sim.launch.py # Lanzador modular de HSV (sim)
│   ├── obstacle.detector.launch.py     # Lanzador modular de obstáculos (real)
│   └── obstacle.detector.sim.launch.py # Lanzador modular de obstáculos (sim)
└── src/
    ├── follow_ball/
    │   ├── control_node.cpp
    │   ├── hsv_filter.cpp
    │   └── obstacle_detector.cpp
    ├── control_node_main.cpp
    ├── hsv_filter_main.cpp
    └── obstacle_detector_main.cpp
```

---

## 📐 Fundamento Teórico: Algoritmo VFF

El algoritmo de navegación reactiva calcula en cada ciclo de control un vector resultante $\vec{V}_{\text{resultante}}$ como combinación lineal del objetivo visual y la evasión de colisiones:

$$\vec{V}_{\text{resultante}} = \alpha \cdot \vec{V}_{\text{atractivo}} + \beta \cdot \vec{V}_{\text{repulsivo}}$$

```
                       ▲ V_atractivo (Pelota)
                       │
                       │
       V_repulsivo     │      V_resultante
        (Obstáculo) ◄──┼─────────► (Comando de velocidad)
                       │
                       │
```

- **Vector Atractivo ($\vec{V}_{\text{atractivo}}$):** Apunta hacia la posición relativa estimada de la pelota calculada por el nodo `hsv_filter`.
- **Vector Repulsivo ($\vec{V}_{\text{repulsivo}}$):** Suma vectorial ponderada en sentido opuesto a los obstáculos más cercanos detectados por el LiDAR (`obstacle_detector`).
- **Comandos de tracción:** El vector resultante se mapea a velocidades lineales ($v_x$) y angulares ($\omega_z$) enviadas sobre el topic `/cmd_vel`.

---

## ⚙️ Arquitectura Software y Nodos

```
  ┌────────────────────────┐
  │   Cámara (RGB)         │
  └───────────┬────────────┘
              │ sensor_msgs/msg/Image
              ▼
  ┌────────────────────────┐                   Vector Atractivo
  │      hsv_filter        ├─────────────────► (geometry_msgs/Vector3) ──┐
  └────────────────────────┘                                             │
                                                                         ▼
  ┌────────────────────────┐                                   ┌──────────────────┐               ┌────────────┐
  │   LiDAR (LaserScan)    │                                   │   control_node   │  /cmd_vel     │            │
  └───────────┬────────────┘                                   │  (FSM + VFF)     ├──────────────►│ Base Móvil │
              │ sensor_msgs/msg/LaserScan                      └──────────────────┘ (Twist)       │  (Kobuki)  │
              ▼                                                          ▲                        └────────────┘
  ┌────────────────────────┐                   Vector Repulsivo          │
  │   obstacle_detector    ├─────────────────► (geometry_msgs/Vector3) ──┘
  └────────────────────────┘
```

### 1. Filtro HSV: Vector Atractivo (`hsv_filter`)
- Recibe el flujo de vídeo de la cámara en formato `sensor_msgs/msg/Image`.
- Aplica umbralización por espacio de color HSV para segmentar la pelota.
- Calcula el centroide del objeto y la distancia estimada en función del área/profundidad.
- Publica el vector atractivo tridimensional normalizado (`geometry_msgs/msg/Vector3`).

### 2. Detector de Obstáculos: Vector Repulsivo (`obstacle_detector`)
- Procesa barridos de telemetría de distancias `sensor_msgs/msg/LaserScan`.
- Evalúa los obstáculos dentro de un umbral de seguridad configurable ($d_{\text{min}}$ y $d_{\text{max}}$).
- Modela fuerzas inversamente proporcionales a la distancia de impacto y publica el vector repulsivo resultante (`geometry_msgs/msg/Vector3`).

### 3. Nodo de Control y Navegación (`control_node`)
El nodo opera mediante una **Máquina de Estados Finitos (FSM)**:

```
          ┌──────────────────────────────────────────────────┐
          │                                                  │
          ▼                                                  │
 ┌───────────────────┐        Pelota detectada      ┌───────────────────┐
 │   BÚSQUEDA        ├─────────────────────────────►│    SEGUIMIENTO    │
 │ (Spin / Giro)     │                              │    (VFF Activo)   │
 └───────────────────┘◄─────────────────────────────┴───────────────────┘
                              Pelota perdida
```

- **Estado 1: BÚSQUEDA (Pelota no percibida):**
  - El robot ejecuta un giro continuo sobre su propio eje.
  - Recuerda la última posición o dirección conocida de la pelota para optimizar la re-adquisición visual en lugar de realizar giros aleatorios.
- **Estado 2: SEGUIMIENTO (Pelota percibida):**
  - Fusión de vectores mediante VFF.
  - Generación de comandos de velocidad continua hacia el objetivo esquivando obstáculos activos.

---

## ⚙️ Gestión de Parámetros y Remapeos

Los nodos emplean nombres de *topics* genéricos (`/image_raw`, `/scan`, `/attractive_vector`, `/repulsive_vector`, `/cmd_vel`) que se vinculan mediante **remapeos en los launchers**.

Todos los parámetros se gestionan desde los ficheros dentro de `config/`:

| Parámetro | Descripción | Fichero de Configuración |
| :--- | :--- | :--- |
| `hsv.h_min`, `hsv.h_max` | Rango de matiz (Hue) de la pelota | `params.yaml`, `params.sim.yaml` |
| `hsv.s_min`, `hsv.s_max` | Rango de saturación | `params.yaml`, `params.sim.yaml` |
| `hsv.v_min`, `hsv.v_max` | Rango de brillo (Value) | `params.yaml`, `params.sim.yaml` |
| `vff.alpha` | Peso / ganancia de atracción | `params_kobuki.yaml` / `params.sim.yaml` |
| `vff.beta` | Peso / ganancia de repulsión | `params_kobuki.yaml` / `params.sim.yaml` |
| `motion.max_linear_vel` | Límite superior de velocidad lineal ($m/s$) | `params_kobuki.yaml` / `params.sim.yaml` |
| `motion.search_angular_vel` | Velocidad de rotación durante la búsqueda ($rad/s$) | `params_kobuki.yaml` / `params.sim.yaml` |

---

## 🛠️ Compilación y Construcción

1. Configura el espacio de trabajo de ROS 2:
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   ```

2. Sitúate en la raíz del workspace y compila el paquete:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select follow_ball
   ```

3. Carga el entorno resultante:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 Guía de Ejecución y Launchers

El paquete incluye *launchers* modulares para pruebas individuales y lanzadores orquestados para ejecución completa:

### 1. Ejecución Completa en Simulación (Gazebo)
Inicia los nodos de percepción HSV, detección de obstáculos y el nodo de control cargando `config/params.sim.yaml` y aplicando los remapeos propios del simulador:
```bash
ros2 launch follow_ball simulation.launch.py
```

### 2. Ejecución Completa en Robot Real (Kobuki)
Lanza toda la arquitectura adaptada a los tópicos y calibración de la base Kobuki y la cámara física (`config/params_kobuki.yaml`):
```bash
ros2 launch follow_ball kobuki.launch.py
```

### 3. Lanzamiento Modular de Subnodos
Para depuración aislada de subsistemas:

* **Filtro HSV únicamente:**
  ```bash
  # En simulación
  ros2 launch follow_ball hsv.filter.sim.launch.py

  # En robot real
  ros2 launch follow_ball hsv.filter.launch.py
  ```

* **Detector de obstáculos únicamente:**
  ```bash
  # En simulación
  ros2 launch follow_ball obstacle.detector.sim.launch.py

  # En robot real
  ros2 launch follow_ball obstacle.detector.launch.py
  ```

---

## 👥 Autores

Desarrollado para la práctica de seguimiento de pelota reactivo mediante VFF en **ROS 2**.
- **Autor / Grupo:** [@usuario](https://github.com/usuario)
