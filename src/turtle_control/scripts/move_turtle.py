#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

def mover_dos_tortugas():
    rospy.init_node('mover_dos_tortugas', anonymous=True)

    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz

    start_time = rospy.get_time()

    estado = "avanzando"
    tiempo_avance = 2.0
    velocidad_angular = 2 * math.pi / 10.0  # ≈ 0.628 rad/s
    # No necesitaremos tiempo_circulo porque será infinito

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed = current_time - start_time

        cmd1 = Twist()
        cmd2 = Twist()

        if estado == "avanzando" and elapsed < tiempo_avance:
            cmd1.linear.x = 1.0
            cmd2.linear.x = 1.0

        elif estado == "avanzando" and elapsed >= tiempo_avance:
            estado = "circulo"

        if estado == "circulo":
            # Giro infinito
            cmd1.linear.x = 1.0
            cmd1.angular.z = -velocidad_angular

            cmd2.linear.x = 1.0
            cmd2.angular.z = velocidad_angular

        pub1.publish(cmd1)
        pub2.publish(cmd2)

        rate.sleep()

if __name__ == '__main__':
    try:
        mover_dos_tortugas()
    except rospy.ROSInterruptException:
        pass
