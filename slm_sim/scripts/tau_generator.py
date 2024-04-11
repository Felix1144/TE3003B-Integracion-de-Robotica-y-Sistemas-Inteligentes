#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32

def random_publisher():
    #"Crea un publicador para el mensaje Float32 en el topico"
    pub = rospy.Publisher('/tau',Float32,queue_size=10)
    while not rospy.is_shutdown():
        #Genera un valor aleatorio entre 4 y 8 o entre -4 y -8
        random_value = random.uniform(4,8) if random.choice([True,False]) else random.uniform(-8,-4)
        rospy.loginfo("Publicado: %.2f",random_value) #Muestra el valor en la terminal
        pub.publish(random_value) #Publicar valor en el topico /tau
        rospy.sleep(0.5) #Esperar 2 segundos
        cero = 0.0
        pub.publish(cero)
        rospy.sleep(40)

        rate.sleep()

if __name__ == '__main__':
    #Inicializa el nodo
    rospy.init_node('tau_generator')
    #Estable la tasa de plublicacion
    rate = rospy.Rate(10) 
    try:
        random_publisher()
    except rospy.ROSInterruptException:
        pass


