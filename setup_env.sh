#!/bin/bash
echo "Instalando dependencias del Proyecto Safety Cobot..."

# 1. Actualizar
sudo apt update

# 2. Instalar herramientas base de ROS y Control
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-moveit-servo \
  ros-noetic-trac-ik-kinematics-plugin \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-joint-state-controller \
  ros-noetic-effort-controllers \
  ros-noetic-position-controllers \
  ros-noetic-universal-robots \
  git

# 3. Instalar dependencias del código fuente
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "¡Entorno listo! Ahora ejecuta 'catkin_make'"

