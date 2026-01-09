#!/usr/bin/env python3
import rospy
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

class SafetyFilter:
    def __init__(self):
        rospy.init_node('safety_filter_node', anonymous=True)

        # 1. SUSCRIPTORES Y PUBLICADORES
        # Escuchamos a la camara bruta
        self.sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback)
        # Publicamos la nube filtrada (LIMPIA)
        self.pub = rospy.Publisher("/camera/depth/points_filtered", PointCloud2, queue_size=10)

        # 2. PARAMETROS DE LA ZONA DE SEGURIDAD (Caja Virtual)
        # Todo lo que este fuera de estas medidas se borra
        self.z_min = 0.05  # Altura minima (para borrar el suelo)
        self.z_max = 2   # Altura maxima (para borrar el techo/ruido lejano)
        
        rospy.loginfo("--- FILTRO DE SEGURIDAD ACTIVADO ---")
        rospy.loginfo(f"Eliminando suelo (< {self.z_min}m) y techo (> {self.z_max}m)")

    def callback(self, ros_cloud):
        # Esta funcion se ejecuta cada vez que llega una imagen de la camara
        
        # 1. Convertir mensaje de ROS a lista de puntos (X, Y, Z)
        # Es lento en Python puro, pero sirve para empezar
        gen = pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True)
        
        filtered_points = []
        
        # 2. FILTRADO (La Lógica)
        for p in gen:
            x, y, z = p
            
            # Condicion: Solo guardar si esta dentro de la altura deseada
            # (Aqui es donde evitamos que vea el suelo como obstaculo)
            if self.z_min < z < self.z_max:
                # Opcional: Aqui podrias agregar mas filtros (ej. distancia X/Y)
                filtered_points.append([x, y, z])

        # 3. Re-empaquetar y Publicar
        if len(filtered_points) > 0:
            header = ros_cloud.header
            # Crear la nube nueva
            cloud_out = pc2.create_cloud_xyz32(header, filtered_points)
            self.pub.publish(cloud_out)

if __name__ == '__main__':
    try:
        sf = SafetyFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass