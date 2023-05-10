#!/usr/bin/env python3

######################### Marco de importaciones ##############################
import numpy as np
import matplotlib.pyplot as plt
import rospy
import threading
import os

from geometry_msgs.msg import Twist
from matplotlib import animation
from std_msgs.msg import Float32MultiArray, Float32

################################ Comentarios ##################################
# El control se realiza sobre el robot del equipo azul
# Los datos dados por los tópicos de posición del balón y del robot estan en milímetros
# Las dimensiones de la cancha son x: [-6000,6000], y: [-4500,4500] mm
# los datos del tópico de velocidad del robot estan en m/s y rad/s

############################# Variables Globales ##############################
global Ball_pos
global x_robot, y_robot, z_robot

##################### Asignación de variables Globales ########################
Ball_pos = 0
x_robot = []
y_robot = []
z_robot = []

################################# Funciones ###################################

def callback_BallPosition(data):
	global Ball_pos
	Ball_pos = data.data # lista [x,y] balon



def callback_RobotPosition(data):	
	global x_robot, y_robot, z_robot
	
	x_robot.append(data.linear.x)
	y_robot.append(data.linear.y)
	z_robot.append(data.angular.z)


def obtener_posicion_objetivo():
	global Ball_pos, x_robot, y_robot

	x_arco_iz = -6000
	y_arco_iz = 0

	x_arco_der = 6000
	y_arco_der = 0


	x_balon = Ball_pos[0]
	y_balon = Ball_pos[1]



	if ((x_arco_iz-x_balon)**2 + (y_arco_iz-y_balon)**2)**(1/2) < ((x_arco_der-x_balon)**2 + (y_arco_der-y_balon)**2)**(1/2):
		x_arco = x_arco_iz
		y_arco = y_arco_iz
		condicion = 1
	else:
		x_arco = x_arco_der
		y_arco = y_arco_der
		condicion = 0

	m = (y_arco-y_balon)/(x_arco-x_balon)
	theta = np.arctan(m)
	r = 60
	x_obj1 = r*np.cos(theta)+x_balon
	y_obj1 = r*np.sin(theta)+y_balon

	x_obj2 = r*np.cos(theta+np.pi)+x_balon
	y_obj2 = r*np.sin(theta+np.pi)+y_balon

	if ((x_obj1-x_arco)**2 + (y_obj1-y_arco)**2)**(1/2) > ((x_obj2-x_arco)**2 + (y_obj2-y_arco)**2)**(1/2):
		x_obj = x_obj1
		y_obj = y_obj1
	else:
		x_obj = x_obj2
		y_obj = y_obj2

	return x_obj, y_obj, theta+np.pi*condicion


def set_graph():
	Dynamic = animation.FuncAnimation(plt.figure(1), animar, 1000000)
	plt.show()

def animar(i):
	global x_robot, y_robot
	plt.cla()
	plt.plot(x_robot, y_robot, label = 'Robot')
	plt.axis([-6000,6000,-4500,4500])
	plt.title('Posición')
	plt.ylabel('y [mm]')
	plt.xlabel('x [mm]')
	plt.legend()


def controlador_orientacion(objetivos, pub, Vel):
	global z_robot

	z_obj = objetivos[2]
	alpha = z_obj-z_robot[-1]
	kz = 0.2

	while abs(alpha)>1*np.pi/180:
		alpha = z_obj-z_robot[-1]
		Vel.angular.z = kz*alpha
		pub.publish(Vel)
	print('Orientación alcanzada')
	Vel.angular.z = 0
	pub.publish(Vel)





def controlador_posicion(objetivo, pub, Vel):
	global x_robot, y_robot

	x_obj = objetivo[0]
	y_obj = objetivo[1]
	
	rho_x = x_obj-x_robot[-1]
	rho_y = y_obj-y_robot[-1]
	rho = (rho_x**2 + rho_y**2)**(0.5)
	rho_list = [rho]

	kx = 0.0002
	ky = 0.0002

	while rho > 50:

		rho_x = x_obj-x_robot[-1]
		rho_y = y_obj-y_robot[-1]
		rho = (rho_x**2 + rho_y**2)**(0.5)
		rho_list.append(rho)
		Vel.linear.x = -kx*rho_x
		Vel.linear.y = ky*rho_y
		pub.publish(Vel)
	print('Posición alcanzada')
	Vel.linear.y = 0
	Vel.linear.x = 0
	pub.publish(Vel)







def principal():
	global Ball_pos, x_robot, y_robot

	rospy.init_node('Generar_movimiento', anonymous = True)

	pub = rospy.Publisher('robot_move_vel', Twist, queue_size = 10)
	rospy.Subscriber('ball_Position', Float32MultiArray, callback_BallPosition)
	rospy.Subscriber('robot_Position', Twist, callback_RobotPosition)

	while Ball_pos == 0 or len(x_robot)==0:
		pass

	Vel = Twist()
	grapher = threading.Thread(target = set_graph)
	grapher.start()

	controlador_orientacion([0,0,np.pi/2], pub, Vel)
	objetivo = obtener_posicion_objetivo()
	controlador_posicion(objetivo, pub, Vel)
	controlador_orientacion(objetivo, pub, Vel)
	BASE_PATH = str(os.path.abspath('')) + '/src/taller3/results/'
	plt.gcf()
	plt.savefig(BASE_PATH+'trayectoria_pelota.png')
	


def prueba():

	x_arco_iz = -5
	y_arco_iz = 0

	x_arco_der = 5
	y_arco_der = 0


	x_balon = -1
	y_balon = -2

	if ((x_arco_iz-x_balon)**2 + (y_arco_iz-y_balon)**2)**(1/2) < ((x_arco_der-x_balon)**2 + (y_arco_der-y_balon)**2)**(1/2):
		x_arco = x_arco_iz
		y_arco = y_arco_iz
	else:
		x_arco = x_arco_der
		y_arco = y_arco_der

	m = (y_arco-y_balon)/(x_arco-x_balon)
	theta = np.arctan(m)
	r = 1
	x_obj1 = r*np.cos(theta)+x_balon
	y_obj1 = r*np.sin(theta)+y_balon

	x_obj2 = r*np.cos(theta+np.pi)+x_balon
	y_obj2 = r*np.sin(theta+np.pi)+y_balon

	if ((x_obj1-x_arco)**2 + (y_obj1-y_arco)**2)**(1/2) > ((x_obj2-x_arco)**2 + (y_obj2-y_arco)**2)**(1/2):
		x_obj = x_obj1
		y_obj = y_obj1
	else:
		x_obj = x_obj2
		y_obj = y_obj2

	up = np.linspace(-5,5,100)
	plt.figure()
	plt.plot(up,np.ones((100,1))*5)
	plt.plot(up,np.ones((100,1))*-5)
	plt.plot(np.ones((100,1))*5,up)
	plt.plot(np.ones((100,1))*-5,up)
	plt.scatter(x_arco,y_arco, label= 'arco')
	plt.scatter(x_balon,y_balon, label = 'balon')
	plt.scatter(x_obj,y_obj, label = 'obj')
	plt.legend()
	plt.show()
	print(theta*180/np.pi)
######################### Main ##############################

if __name__ == '__main__':
	principal()
	#prueba()