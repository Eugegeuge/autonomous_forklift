#!/bin/bash

echo "--- INICIANDO ROBOT ---"

# 1. Limpieza previa
echo "Matando procesos antiguos..."
killall -9 urg_node2_node 2>/dev/null
killall -9 kobuki_ros_node 2>/dev/null

# 2. Base Kobuki
echo "Lanzando Kobuki..."
ros2 launch kobuki kobuki.launch.py &
PID_KOBUKI=$!
sleep 3

# 3. Lidar (Hokuyo)
echo "Lanzando Lidar (IP: 192.168.0.11)..."
# Le ponemos nombre fijo 'lidar_node' para encontrarlo fácil
ros2 run urg_node2 urg_node2_node --ros-args -r __node:=lidar_node -p ip_address:='192.168.0.11' -p frame_id:='laser' &
PID_LIDAR=$!

echo "Esperando al Lidar..."
sleep 5

# 4. Activar Lidar (Lifecycle)
echo "Configurando Lidar..."
ros2 lifecycle set /lidar_node configure
sleep 2
echo "Activando Lidar..."
ros2 lifecycle set /lidar_node activate

# 5. Transformada (TF)
echo "Publicando TF..."
ros2 run tf2_ros static_transform_publisher 0 0 0 1 0 0 0 base_link laser &
PID_TF=$!

echo "--- TODO LISTO ---"
echo "Monitoriza en otro terminal: ros2 topic echo /scan"
echo "Pulsa Ctrl+C para salir."

wait $PID_KOBUKI $PID_LIDAR $PID_TF
