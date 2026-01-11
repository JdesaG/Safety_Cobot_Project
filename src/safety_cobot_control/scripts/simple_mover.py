#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

class SimpleMover:
    def __init__(self):
        # 1. Inicializar MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('simple_mover', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # 2. CONFIGURACIÓN DE VELOCIDAD MÁXIMA (Modo Turbo)
        # Lo ponemos al 100% (1.0) para que sea obvio cuando el sistema de seguridad lo frene.
        self.move_group.set_max_velocity_scaling_factor(1.0)
        self.move_group.set_max_acceleration_scaling_factor(1.0)

        # 3. Protección del Suelo
        rospy.sleep(1) # Esperar a que conecte
        self.add_floor()
        
        rospy.loginfo("--- ROBOT LISTO: MODO VELOCIDAD ALTA (100%) ---")

    def add_floor(self):
        # Evita que el robot intente pasar por debajo del piso
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.z = -0.02
        self.scene.add_box("suelo_virtual", p, (3.0, 3.0, 0.01))

    def mover_motores(self, base, hombro, codo, m1, m2, m3):
        # Mueve las articulaciones a angulos exactos (radianes)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = base
        joint_goal[1] = hombro
        joint_goal[2] = codo
        joint_goal[3] = m1
        joint_goal[4] = m2
        joint_goal[5] = m3

        # go(wait=True) bloquea el codigo hasta que el robot termina de moverse
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def ciclo_infinito(self):
        rospy.loginfo("Iniciando rutina de movimiento rápido...")
        
        while not rospy.is_shutdown():
            # POSICION 1: Izquierda
            rospy.loginfo(">> Moviendo a IZQUIERDA")
            self.mover_motores(1.5, -1.0, 1.5, -1.5, -1.5, 0)
            
            # POSICION 2: Centro (Vertical)
            rospy.loginfo(">> Moviendo a CENTRO")
            self.mover_motores(0, -1.57, 0, -1.57, 0, 0)

            # POSICION 3: Derecha
            rospy.loginfo(">> Moviendo a DERECHA")
            self.mover_motores(-1.5, -1.0, 1.5, -1.5, -1.5, 0)

if __name__ == '__main__':
    try:
        mover = SimpleMover()
        mover.ciclo_infinito()
    except rospy.ROSInterruptException:
        pass