#!/usr/bin/env python3
import sys
import rospy
import math
import moveit_commander
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class SafetyMonitor:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('safety_monitor_node', anonymous=True)
        
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.sub = rospy.Subscriber("/camera/depth/points_filtered", PointCloud2, self.callback)

        # --- CONFIGURACIÓN DE ZONAS (AJUSTA ESTOS VALORES) ---
        # ZONA ROJA: El alcance fisico del brazo. 
        # Si entra aqui -> STOP INMEDIATO.
        # Un UR5 estirado mide ~0.85m. Ponemos 0.60m como zona de trabajo activa.
        self.RADIO_ROJO = 0.60  

        # ZONA AMARILLA: Tu "metro" de advertencia.
        # Si entra aqui -> VELOCIDAD LENTA.
        # Definimos que va desde el borde rojo hasta 1.0m
        self.RADIO_AMARILLO = 1.0 

        # Estado actual
        self.estado_actual = "VERDE"
        self.last_speed = 1.0

        rospy.loginfo("--- MONITOR DE SEGURIDAD (LOGICA PAPER) ---")
        rospy.loginfo(f"🛑 ZONA ROJA (Parada): 0m - {self.RADIO_ROJO}m")
        rospy.loginfo(f"⚠️ ZONA AMARILLA (Lento): {self.RADIO_ROJO}m - {self.RADIO_AMARILLO}m")
        rospy.loginfo(f"✅ ZONA VERDE (Rapido): > {self.RADIO_AMARILLO}m")

    def cambiar_velocidad(self, factor):
        # MoveIt necesita "re-escalar" la velocidad.
        # Nota: Esto afecta al SIGUIENTE movimiento planificado.
        if self.last_speed != factor:
            self.move_group.set_max_velocity_scaling_factor(factor)
            self.move_group.set_max_acceleration_scaling_factor(factor)
            self.last_speed = factor
            rospy.loginfo(f"⚙️ Velocidad ajustada al {factor*100}%")

    def callback(self, ros_cloud):
        # --- PROCESAMIENTO TURBO ---
        # Saltamos puntos para leer mas rapido (Step 40 = leemos 1 de cada 40 puntos)
        step = 40 
        gen = pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True)
        
        distancia_minima = 99.9
        
        # Buscamos el punto mas cercano
        count = 0
        for p in gen:
            count += 1
            if count % step != 0: continue 
            
            x, y, z = p
            # Distancia desde la base del robot (0,0,0)
            dist = math.sqrt(x**2 + y**2 + z**2)
            
            if dist < distancia_minima:
                distancia_minima = dist

        # Si no detectamos nada o la distancia es gigante, salimos
        if distancia_minima > 5.0: return

        # --- LOGICA DE ZONAS ---
        
        # 1. CASO ROJO: INTRUSIÓN CRÍTICA
        if distancia_minima < self.RADIO_ROJO:
            # Imprimir alerta siempre en rojo para que lo veas
            sys.stdout.write(f"\r🛑 PELIGRO: {distancia_minima:.3f}m | PARADA DE EMERGENCIA      ")
            sys.stdout.flush()
            
            # Orden de parada continua mientras este en la zona
            self.move_group.stop()
            self.estado_actual = "ROJO"

        # 2. CASO AMARILLO: ADVERTENCIA
        elif distancia_minima < self.RADIO_AMARILLO:
            sys.stdout.write(f"\r⚠️ ALERTA: {distancia_minima:.3f}m | Reduciendo Velocidad...   ")
            sys.stdout.flush()
            
            if self.estado_actual != "AMARILLO":
                # Bajamos al 10% de velocidad
                self.cambiar_velocidad(0.1) 
                self.estado_actual = "AMARILLO"
                
                # TRUCO: Si el robot iba rapido, hacemos un stop breve para que
                # el siguiente movimiento ya salga lento.
                self.move_group.stop() 

        # 3. CASO VERDE: LIBRE
        else:
            sys.stdout.write(f"\r✅ LIBRE: {distancia_minima:.3f}m | Velocidad Normal          ")
            sys.stdout.flush()
            
            if self.estado_actual != "VERDE":
                # Volvemos al 100%
                self.cambiar_velocidad(1.0)
                self.estado_actual = "VERDE"

if __name__ == '__main__':
    try:
        monitor = SafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass