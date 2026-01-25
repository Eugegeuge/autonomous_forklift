# Autonomous Forklift - Sistema de Carretilla Autónoma

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Simulator](https://img.shields.io/badge/Simulator-MVSim-green)
![Nav2](https://img.shields.io/badge/Navigation-Nav2-purple)
![Status](https://img.shields.io/badge/Status-Functional-brightgreen)

Sistema completo de carretilla autónoma para logística de almacén utilizando **ROS 2 Humble**, simulación con **MVSim** y navegación con **Nav2**.

---

## 📋 Índice

- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Instalación](#-instalación)
- [Uso](#-uso)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Fases de la Misión](#-fases-de-la-misión)
- [Sistema de Agarre](#-sistema-de-agarre-del-pallet)
- [Editor de Grafos](#-editor-de-grafos)
- [Equipo](#-equipo)

---

## 🏗 Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                        INTERFACE NODE                           │
│                    (GUI Control de Misión)                      │
└─────────────────────────┬───────────────────────────────────────┘
                          │ Topics ROS 2
          ┌───────────────┼───────────────┬───────────────┐
          ▼               ▼               ▼               ▼
    /navegacion    /navigation_goal   /agarre      /deposicion
          │               │               │               │
          ▼               ▼               ▼               ▼
┌─────────────────┐ ┌─────────────┐ ┌─────────────────────────┐
│ WAYPOINT        │ │   NAV2      │ │    LIFT CONTROLLER      │
│ FOLLOWER        │◄┤  (AMCL +    │ │  (Control Elevador)     │
│ (Navegación     │ │  Planner)   │ │                         │
│  por Grafos)    │ │             │ │  - Enganche pallet      │
└────────┬────────┘ └─────────────┘ │  - Suelta pallet        │
         │                          │  - Comunicación MVSim   │
         ▼                          └─────────────────────────┘
    /cmd_vel
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                          MVSIM                                  │
│              (Simulador 2.5D del Almacén)                       │
│                                                                 │
│   🚜 Forklift    📦 Pallets    🏭 Estanterías                  │
└─────────────────────────────────────────────────────────────────┘
```

### Componentes Principales

| Componente | Script | Función |
|------------|--------|---------|
| **Interface Node** | `interface_node.py` | GUI de control de misiones con Tkinter |
| **Waypoint Follower** | `waypoint_follower.py` | Navegación por grafos con algoritmo BFS |
| **Lift Controller** | `lift_controller.py` | Control del elevador y manipulación de pallets |
| **Graph Visualizer** | `graph_visualizer.py` | Visualización del grafo en RViz |
| **Graph Editor** | `graph_editor.py` | Editor web para modificar el grafo de navegación |

---

## 🚀 Instalación

### Prerrequisitos

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- MVSim: `sudo apt install ros-humble-mvsim`
- Nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- SLAM Toolbox: `sudo apt install ros-humble-slam-toolbox`

### Compilación

```bash
cd ~/ros2_ws/autonomous_forklift
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 Uso

### 1. Lanzar el Sistema Completo

```bash
ros2 launch autonomous_forklift system.launch.py
```

Esto inicia:
- **MVSim**: Simulador con el almacén y el forklift
- **Nav2**: Stack de navegación (AMCL + Planner)
- **RViz2**: Visualización
- **Waypoint Follower**: Navegación por grafos
- **Interface Node**: Interfaz gráfica de control
- **Lift Controller**: Control del elevador

### 2. Crear/Editar Mapas (SLAM)

```bash
ros2 launch autonomous_forklift mapping.launch.py
```

Para guardar el mapa:
```bash
ros2 run autonomous_forklift save_map.py
```

### 3. Editor de Grafos

```bash
ros2 run autonomous_forklift graph_editor.py
```
Luego abre el navegador en: `http://localhost:8000`

### 4. Control Manual (Teleoperación)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 📁 Estructura del Proyecto

```
autonomous_forklift/
├── src/
│   ├── autonomous_forklift/          # Paquete principal
│   │   ├── config/
│   │   │   ├── nav2_params.yaml      # Parámetros Nav2
│   │   │   ├── slam_params.yaml      # Parámetros SLAM
│   │   │   └── warehouse_graph.geojson # Grafo de navegación
│   │   ├── launch/
│   │   │   ├── system.launch.py      # Launch principal
│   │   │   ├── navigation.launch.py  # Nav2 + Simulación
│   │   │   ├── simulation.launch.py  # Solo MVSim
│   │   │   └── mapping.launch.py     # SLAM Toolbox
│   │   ├── maps/
│   │   │   └── mundo_map.yaml/pgm    # Mapa del almacén
│   │   ├── mvsim_models/
│   │   │   ├── forklift.world.xml    # Mundo MVSim
│   │   │   └── forklift.vehicle.xml  # Definición del robot
│   │   ├── scripts/
│   │   │   ├── interface_node.py     # GUI de control
│   │   │   ├── waypoint_follower.py  # Navegación por grafos
│   │   │   ├── lift_controller.py    # Control elevador
│   │   │   ├── graph_visualizer.py   # Markers RViz
│   │   │   ├── graph_editor.py       # Editor web
│   │   │   └── save_map.py           # Guardar mapas
│   │   ├── rviz/
│   │   │   └── rviz_copiaDEFINITIVA.rviz
│   │   └── worlds/
│   │       └── mundo.xml             # Mundo principal
│   └── smart_warehouse/              # Modelos adicionales
└── README.md
```

---

## 🔄 Fases de la Misión

El sistema ejecuta misiones de transporte de pallets en **10 fases secuenciales**:

| Fase | Nombre | Descripción |
|------|--------|-------------|
| 1 | **NAV → ORIGEN** | Navegación autónoma hacia la estantería de origen |
| 2 | **APROXIMACIÓN** | Movimiento ciego (0.5 m/s × 900ms) para acercarse al pallet |
| 3 | **RECOGIENDO** | Activa elevador, engancha pallet. Marca estantería como VACÍA |
| 4 | **RETROCESO** | Navegación reversa al nodo anterior del grafo |
| 5 | **NAV → DESTINO** | Navegación hacia la estantería de destino |
| 6 | **APROXIMACIÓN** | Movimiento ciego hacia posición de descarga |
| 7 | **DEPOSITANDO** | Suelta pallet. Marca estantería como OCUPADA |
| 8 | **RETROCESO** | Navegación reversa al nodo anterior |
| 9 | **NAV → HOME** | Regreso a posición inicial |
| 10 | **REPOSO** | Misión completada |

### Control de Emergencia

El botón **PARADA DE EMERGENCIA** funciona como toggle:
- **1ª pulsación**: Detiene el robot inmediatamente y pausa la misión
- **2ª pulsación**: Reanuda la misión desde la fase pausada

---

## 🦾 Sistema de Agarre del Pallet

### Funcionamiento

El control del elevador se implementa en `lift_controller.py`:

1. **Enganche** (topic `/agarre`):
   - Busca el pallet más cercano (< 3m)
   - Lo "engancha" y comienza a seguir al forklift
   - Actualiza posición a 50Hz mediante comunicación con MVSim

2. **Suelta** (topic `/deposicion`):
   - Libera el pallet en su posición actual
   - Deja de actualizarlo

### Parámetros

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `GRASP_OFFSET_X` | 0.8 m | Distancia pallet-robot |
| `MAX_GRASP_DISTANCE` | 3.0 m | Distancia máxima de enganche |

---

## 🗺 Editor de Grafos

El grafo de navegación se almacena en formato GeoJSON en `config/warehouse_graph.geojson`.

### Uso del Editor Web

```bash
ros2 run autonomous_forklift graph_editor.py
# Abrir http://localhost:8000
```

### Funcionalidades
- **Añadir nodos**: Click en el mapa
- **Conectar nodos**: Seleccionar dos nodos y crear arista
- **Nombrar nodos**: Asignar nombres (HOME, ESTANTERIA_1, etc.)
- **Exportar**: Guardar cambios al archivo GeoJSON

---

## 🤖 Modelo del Robot

### Especificaciones

| Parámetro | Valor |
|-----------|-------|
| Tipo de tracción | Diferencial |
| Masa | 200 kg |
| Diámetro ruedas | 0.40 m |
| Par motor máximo | 200 Nm |

### Sensores

| Sensor | FOV | Alcance | Topic |
|--------|-----|---------|-------|
| LIDAR 360° | 360° | 25 m | `/scan` |
| LIDAR Frontal | 120° | 5 m | `/front_scan` |
| LIDAR Trasero | 120° | 5 m | `/rear_scan` |
| IMU | - | - | `/imu` |

---

## 🔧 Topics ROS 2 Principales

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Comandos de velocidad |
| `/scan` | `LaserScan` | LIDAR principal |
| `/navegacion` | `String` | ON/OFF navegación |
| `/navigation_goal` | `String` | Destino de navegación |
| `/agarre` | `String` | Comando de enganche |
| `/deposicion` | `String` | Comando de suelta |
| `/navigation_status` | `String` | Estado (REACHED, MOVING) |

---

## 👥 Equipo

| Nombre | Rol |
|--------|-----|
| Hugo Sevilla Martínez | Desarrollo |
| Juan Diego Serrato Tovar | Desarrollo |
| Hugo López Pastor | Desarrollo |
| Pablo Molina Pérez | Desarrollo |

---

## 📄 Licencia

Proyecto académico - Universidad de Alicante

---

## 🎬 Demo

```bash
# Terminal 1: Lanzar sistema
ros2 launch autonomous_forklift system.launch.py

# La interfaz gráfica aparecerá automáticamente
# Seleccionar origen y destino, pulsar "INICIAR TAREA"
```
