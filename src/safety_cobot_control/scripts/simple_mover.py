#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

class SimpleMover:
    def __init__(self):
        # 1. Inicializar
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('simple_mover', anonymous=True)

        # 2. Actores Principales
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface() # Interfaz para añadir obstaculos
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # 3. Configuracion de Seguridad
        self.move_group.set_max_velocity_scaling_factor(0.4)
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10) # Intentar mas veces si falla

        # 4. SOLUCION AL CHOQUE: AÑADIR EL SUELO
        rospy.sleep(2) # Esperar a que la escena conecte
        self.add_floor()

        rospy.loginfo("--- ROBOT LISTO Y SUELO PROTEGIDO ---")

    def add_floor(self):
        # Creamos una caja gigante justo debajo del robot para representar el suelo
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.z = -0.02  # Ligeramente debajo de 0 para no chocar con la base
        
        # Añadimos una caja de 3x3 metros
        # (Nombre, Pose, Dimensiones)
        self.scene.add_box("suelo_virtual", p, (3.0, 3.0, 0.01))
        rospy.loginfo(">> Suelo Virtual añadido a MoveIt")

    def ir_a_pose(self, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        self.move_group.set_pose_target(pose_goal)
        # Planificar primero para ver si es posible
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return plan

    def ir_a_home(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -1.57 # Hombro Arriba
        joint_goal[2] = 0.0
        joint_goal[3] = -1.57
        joint_goal[4] = 0.0
        joint_goal[5] = 0.0
        
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

    def ciclo_infinito(self):
        self.ir_a_home()
        while not rospy.is_shutdown():
            # Movimiento 1
            rospy.loginfo(">> Izquierda (Alto)")
            self.ir_a_pose(0.4, 0.3, 0.5) 
            rospy.sleep(0.5)

            # Movimiento 2
            rospy.loginfo(">> Derecha (Alto)")
            self.ir_a_pose(0.4, -0.3, 0.5)
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        mover = SimpleMover()
        mover.ciclo_infinito()
    except rospy.ROSInterruptException:
        pass