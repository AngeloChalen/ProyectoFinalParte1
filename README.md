
# 🤖 Planificación de Trayectorias LPA* para Unitree Go2 en ROS 2

Este repositorio contiene la implementación de un sistema de navegación autónoma para el robot cuadrúpedo **Unitree Go2**. El proyecto abarca desde la simulación en entornos industriales (Gazebo) y generación de mapas (SLAM), hasta la planificación de rutas óptimas utilizando el algoritmo **LPA* (Lifelong Planning A*)**.

## 📋 Tabla de Contenidos
1. [Requisitos e Instalación](#1-requisitos-e-instalación)
2. [Paso 1: Simulación en Gazebo](#2-paso-1-simulación-en-gazebo)
3. [Paso 2: Mapeo (SLAM)](#3-paso-2-mapeo-slam)
4. [Paso 3: Guardado y Edición del Mapa](#4-paso-3-guardado-y-edición-del-mapa)
5. [Paso 4: Planificación de Rutas (LPA*)](#5-paso-4-planificación-de-rutas-lpa)
6. [Estructura del Proyecto](#6-estructura-del-proyecto)



## 1. Requisitos e Instalación

### Dependencias del Sistema
Este proyecto fue desarrollado en **Ubuntu 22.04** con **ROS 2 Humble**. Es necesario instalar las siguientes librerías de navegación y descripción:

```bash
sudo apt update
sudo apt install ros-humble-nav2-map-server ros-humble-joint-state-publisher ros-humble-xacro ros-humble-rviz2

```

### Configuración del Workspace

Clona este repositorio dentro de la carpeta `src` de tu espacio de trabajo:

```bash
cd ~/go2_ws/src
git clone [https://github.com/AngeloChalen/ProyectoFinalParte1.git](https://github.com/AngeloChalen/ProyectoFinalParte1.git)

```

### Compilación

Para que ROS reconozca los paquetes y los mapas nuevos, compila el entorno:

```bash
cd ~/go2_ws
colcon build --symlink-install
source install/setup.bash

```

---

## 2. Paso 1: Simulación en Gazebo

Iniciamos el entorno de simulación industrial ("Factory"). Este comando carga el modelo del Unitree Go2 equipado con un **LiDAR 2D**.

```bash
ros2 launch go2_config gazebo.launch.py world:=factory

```

> **Verificación:** Deberías ver el entorno de la fábrica y el robot con el láser girando en la parte superior.

---

## 3. Paso 2: Mapeo (SLAM)

Para generar el mapa de ocupación, utilizamos SLAM. Abre una **nueva terminal** y ejecuta:

```bash
cd ~/go2_ws
source install/setup.bash
ros2 launch go2_config slam.launch.py use_sim_time:=true

```

### Teleoperación

Para explorar la fábrica y completar el mapa, mueve el robot usando el teclado en otra terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

* **Teclas:** `i` (adelante), `j` (izquierda), `l` (derecha), `,` (atrás), `k` (parar).

---

## 4. Paso 3: Guardado y Edición del Mapa

### Guardar el Mapa

Una vez que el mapa en RViz se vea completo, guárdalo ejecutando:

```bash
cd ~/go2_ws/mapeos/
ros2 run nav2_map_server map_saver_cli -f mapa_fabrica

```

Esto generará `mapa_fabrica.pgm` y `mapa_fabrica.yaml`.

### Edición con GIMP (Mejora del Mapa)

Los mapas de SLAM suelen tener "ruido". Para limpiarlo:

1. Abre `mapa_fabrica.pgm` con **GIMP**.
2. Usa el pincel con color **Blanco** para limpiar zonas libres (borrar manchas grises).
3. Usa el color **Negro** para cerrar paredes o definir obstáculos.
4. Exporta sobreescribiendo el archivo original (formato RAW o ASCII).

---

## 5. Paso 4: Planificación de Rutas (LPA*)


### Terminal 1: Servidor del Mapa

Carga el mapa estático que editamos previamente.

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/$USER/go2_ws/mapeos/mapa_fabrica.yaml

```

*(Espera a que diga "Waiting on external lifecycle transitions...")*

### Terminal 2: Activación y Robot

Activa el mapa y carga la descripción visual del robot.

```bash
# Activar el mapa
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
#Debe salir Transitioning successfull

# Cargar modelo del robot
cd ~/go2_ws
colcon build --packages-select go2_description
source install/setup.bash
ros2 launch go2_description description.launch.py &
ros2 run joint_state_publisher joint_state_publisher

```

### Terminal 3: Nodo Planificador LPA*

Ejecuta el script de inteligencia artificial que calcula la ruta.

```bash
cd ~/go2_ws
source install/setup.bash
python3 src/unitree-go2-ros2/lpa_planner_node.py
```

### Terminal 4: Visualización (RViz)

Abre RViz con la configuración lista para visualizar el Path.

```bash
ros2 run rviz2 rviz2 -d ~/go2_ws/src/go2_config/rviz/go2_lpa.rviz

```


## Configuracion RVIZ2
### 1️⃣ Configuración Global (Global Options)
Esto es lo primero que debes revisar en el panel izquierdo (Displays):
* Busca Global Options.
* Fixed Frame: Escribe o selecciona map.
### 2️⃣ Agregar el Mapa (Factory2)
Para ver las paredes negras y el suelo gris:
1. Haz clic en el botón Add (abajo a la izquierda).
2. Busca la pestaña By Topic.
3. Busca `map` y selecciona Map.
4. Clic en OK.
5. (Importante) Si no ves el mapa, despliega las opciones de "Map" en el panel izquierdo y busca Durability Policy. Cámbialo a `Transient Local`.
### 3️⃣ Agregar el Robot (Unitree Go2)
Para ver al perro gris en 3D (y no un cubo o nada):
1. Clic en Add.
2. Pestaña **By Display Type** -> Selecciona RobotModel.
3. Clic en OK.
4. En el panel izquierdo, dentro de las opciones de RobotModel:
* * **Description Topic:** Asegúrate de que esté seleccionado `robot_description`.
### 4️⃣ Agregar la Ruta (La Línea Verde)
Para cumplir con el punto de "Path Visible" de la rúbrica:
1. Clic en Add.
2. Pestaña By Topic.
3. Busca `/plan` y selecciona Path.
4. Clic en OK.

### 🎯 Prueba de Planificación

1. En RViz, selecciona la herramienta **"2D Goal Pose"** (flecha verde superior).
2. Haz clic en un punto libre del mapa.
3. El algoritmo calculará la trayectoria y dibujará una línea (Path) conectando al robot con el objetivo.

---



# PARTE 2 PROYECTO FINAL


## 🚀 Proyecto de Navegación Autónoma: Unitree Go2 en Entorno Industrial
Este repositorio contiene la solución técnica para la navegación autónoma del robot cuadrúpedo **Unitree Go2** en un entorno de fábrica simulado en **ROS 2 Humble**.

### 🧠 Descripción del Sistema
El sistema se basa en una arquitectura de planificación y control desacoplada, optimizada para entornos con obstáculos densos:

1. **Planificador Global (LPA*):** Implementa el algoritmo Lifelong Planning A* en el nodo `lpa_planner_node.py`.
  * **Capa de Inflación:** Se configuró un margen de seguridad de 6 celdas para evitar colisiones físicas en Gazebo basándose exclusivamente en el mapa lógico `.pgm`.
  * **Evidencia CSV (Pure Pursuit):** Genera automáticamente un archivo con los waypoints de la ruta en la carpeta `~/go2_ws`.

2. **Controlador Local (Pure Pursuit):** Ejecutado en el nodo `pure_pursuit_node.py`, encargado del seguimiento de trayectoria con alta fidelidad.
   * **Métrica de Precisión:** álculo de distancia euclidiana directa desde el punto de origen (`2D Pose Estimate`) para garantizar exactitud de 10m o 15m.
   * **Monitoreo:** Incluye un cronómetro de tiempo real y publicación de rastro visual en el tópico `/drive_path`.

### 🛠️ Requisitos de Software
* Ubuntu 22.04 con ROS 2 Humble.
* Simulador Gazebo Classic.
* Visualizador RViz2.

## 🚦 Guía de Ejecución Rápida
Siga este orden en terminales independientes para inicializar el sistema:
1. Terminal 1:Inicializar Simulación Gazebo
   ```bash
   cd ~/go2_ws
   colcon build
   source install/setup.bash
   ros2 launch go2_config gazebo.launch.py world:=factory
   ```
2. Terminal 2: Cargar Mapa de Navegación y el modelo del robot
   ```bash
   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/$USER/go2_ws/mapeos/mapa_fabrica.yaml
   ```
   Terminal 3:
   ```
   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activate
   #debe salir Transitioning successfull

   #luego 
   # Cargar modelo del robot
   cd ~/go2_ws
   colcon build --packages-select go2_description
   source install/setup.bash
   # Cargar modelo del robot
   ros2 launch go2_description description.launch.py &
   ros2 run joint_state_publisher joint_state_publisher

   
   ```
4. Ejecutar Planificador de trayectoria y control
   ### Terminal 4: Planificación Global
   ```bash
   cd ~/go2_ws
   source install/setup.bash
   python3 src/unitree-go2-ros2/lpa_planner_node.py

   ```
   ### Terminal 5: Control y Métricas
   ```bash
   cd ~/go2_ws
   source install/setup.bash
   python3 src/unitree-go2-ros2/pure_pursuit_node.py
   ```

5. terminal 6 : Visualización y Pruebas en Rviz
   ```bash
   ros2 run rviz2 rviz2 -d ~/go2_ws/src/go2_config/rviz/go2_lpa.rviz
   ```
## Configuracion RVIZ2
### 1️⃣ Configuración Global (Global Options)
Esto es lo primero que debes revisar en el panel izquierdo (Displays):
* Busca Global Options.
* Fixed Frame: Escribe o selecciona map.
### 2️⃣ Agregar el Mapa (Factory2)
Para ver las paredes negras y el suelo gris:
1. Haz clic en el botón Add (abajo a la izquierda).
2. Busca la pestaña By Topic.
3. Busca `map` y selecciona Map.
4. Clic en OK.
5. (Importante) Si no ves el mapa, despliega las opciones de "Map" en el panel izquierdo y busca Durability Policy. Cámbialo a `Transient Local`.
### 3️⃣ Agregar el Robot (Unitree Go2)
Para ver al perro gris en 3D (y no un cubo o nada):
1. Clic en Add.
2. Pestaña **By Display Type** -> Selecciona RobotModel.
3. Clic en OK.
4. En el panel izquierdo, dentro de las opciones de RobotModel:
* * **Description Topic:** Asegúrate de que esté seleccionado `robot_description`.
### 4️⃣ Agregar la Ruta (La Línea Verde)
Para cumplir con el punto de "Path Visible" de la rúbrica:
1. Clic en Add.
2. Pestaña By Topic.
3. Busca `/plan` y selecciona Path.
4. Clic en OK.

### 🎯 Prueba de Planificación

1. En RViz, selecciona la herramienta **"2D Goal Pose"** (flecha verde superior).
2. Haz clic en un punto libre del mapa.
3. El algoritmo calculará la trayectoria y dibujará una línea (Path) conectando al robot con el objetivo.


### Demostración de movimiento de CUADRUPEDO mapeo
Haga clic en la imagen para ver el video completo en YouTube:
[![Video de Simulación Go2](https://img.youtube.com/vi/MYDZ2EgiQFA/0.jpg)]([https://youtu.be/dFn0udzOmLs](https://youtu.be/sOy21uL0iJc))


### Demostración de movimiento de CUADRUPEDO planificacion + Controlador
Haga clic en la imagen para ver el video completo en YouTube:
[![Video de Simulación Go2](https://img.youtube.com/vi/MYDZ2EgiQFA/0.jpg)](https://youtu.be/dFn0udzOmLs)


---

**Autor:** Angelo Chalen

**Institución:** ESPOL - Ingeniería en Electrónica y Automatización

```

```
