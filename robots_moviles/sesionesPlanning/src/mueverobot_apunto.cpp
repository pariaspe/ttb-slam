#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "sesionesPlanning/apunto.h"
#include <tf/tf.h>
#include <iostream>

using namespace std;

geometry_msgs::Point punto_actual;
int orientacionActual = 0;
double angulo = 0;
ros::Publisher mueve_robot;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	punto_actual.x = msg->pose.pose.position.x;
	punto_actual.y = msg->pose.pose.position.y;
	
	tf::Quaternion q(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
			 msg->pose.pose.orientation.z,
			 msg->pose.pose.orientation.w);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	angulo = yaw;
	angulo = (360*angulo)/(2*3.1415);
	//cout << "El ángulo actual es "<< angulo << endl;
	
	//Actualizo aquí el ángulo
}

void avanzar(float velocidad){
	geometry_msgs::Twist msg;
    msg.linear.x = 0.1;
    ROS_INFO("Enviando %f como velocidad en x", msg.linear.x);
    mueve_robot.publish(msg);
}

void avanza(float distancia){
	geometry_msgs::Twist msg;
    msg.linear.x = 0.2;
    ROS_INFO("Enviando %f como velocidad en x", msg.linear.x);
    mueve_robot.publish(msg);
    
    ros::Rate loop_rate(10);
    
    double piX = punto_actual.x;
    double piY = punto_actual.y;
    
    double pfX = piX;
    double pfY = piY;
    
	
	while (ros::ok() && (sqrt(pow((piX - pfX),2) + pow((piY-pfY),2)) < distancia))
  {
	
	pfX = punto_actual.x;
	pfY = punto_actual.y;
	cout << "He recorrido "<< sqrt(pow((piX - pfX),2) + pow((piY-pfY),2))<< endl; 
	mueve_robot.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
   
  }
    
}

bool distanciaAlcanzada(geometry_msgs::Point distanciaInicial, float distanciaPedida){
	cout << "He recorrido "<< abs(punto_actual.x - distanciaInicial.x) << " metros" << endl;
	return ( abs(punto_actual.x - distanciaInicial.x) > distanciaPedida);
}

bool giraRobot(double anguloGiro){
	geometry_msgs::Twist msg;
	//a la derecha negativo, a la izquierda positivo
	
    double angulo_actual = angulo;
	if (angulo_actual < 0){
		angulo_actual = 360 - abs(angulo_actual);
	}
	
	double angulo_final_giro = angulo_actual + anguloGiro;
	cout << "angulo final giro original " << angulo_final_giro << endl;
	if ((angulo_final_giro > 360) || (angulo_final_giro < 0)){
		angulo_final_giro = abs(360 - abs(angulo_final_giro));
	}
	cout << "angulo final giro modificado " << angulo_final_giro << endl;
	
	ros::Rate loop_rate(10);
	
	int signo = 1;
	if (anguloGiro < 0) {
		signo = -1;
	} 
    msg.angular.z = 0.2*signo;
    
    ROS_INFO("Enviando %f como velocidad en z", msg.angular.z);
 
    while (ros::ok() && abs(angulo_actual - angulo_final_giro) > 0.7)
  {
	angulo_actual = angulo;
	if (angulo_actual < 0){
		angulo_actual = 360 - abs(angulo_actual);
	}
    
    cout << "Angulo actual es " << angulo << " y tiene que alcanzar " << angulo_final_giro << ". El ángulo actual sin signo es " << angulo_actual<< endl;
		mueve_robot.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
   
  }
    
}


bool ponteEnAngulo(double anguloGiro){
	geometry_msgs::Twist msg;
	//a la derecha negativo, a la izquierda positivo
	
    double angulo_actual = angulo;
	if (angulo_actual < 0){
		angulo_actual = 360 - abs(angulo_actual);
	}
	
	double angulo_final_giro = anguloGiro;
	
	ros::Rate loop_rate(10);
	
	int signo = 1;

	if ((angulo_actual > angulo_final_giro) && ((angulo_actual - angulo_final_giro) < 180)){
		signo = -1;
	}
    msg.angular.z = 0.2*signo;
    
    ROS_INFO("Enviando %f como velocidad en z", msg.angular.z);
 
    while (ros::ok() && abs(angulo_actual - angulo_final_giro) > 0.7)
  {
	angulo_actual = angulo;
	if (angulo_actual < 0){
		angulo_actual = 360 - abs(angulo_actual);
	}
	if (angulo_actual > 359 && angulo_actual < 360.1){
		angulo_actual = 0;
	}
    
    cout << "Angulo actual es " << angulo << " y tiene que alcanzar " << angulo_final_giro << ". El ángulo actual sin signo es " << angulo_actual<< endl;
	mueve_robot.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
   
  }
    
}

bool moverEnDireccion(sesionesPlanning::apunto::Request  &req,
         sesionesPlanning::apunto::Response &res)
{
	cout << "Me han enviado una petición de dirección" << endl;
	//suponemos que el robot en el inicio está mirando abajo (dirección 2, el cero es la derecha)
  geometry_msgs::Twist msg;
  switch (req.direccion){
	  case 0:
		//cout << "Este es el caso 0" << endl;
		//cout << "Voy a girar 90 grados" << endl;
		ponteEnAngulo(90);
		avanza(1);
	  break;
	  case 1:
		ponteEnAngulo(45);
		avanza(1.4);
	  break;
	  case 2:
		ponteEnAngulo(0);
		avanza(1);
	  break;
	  case 3:
		ponteEnAngulo(315);
		avanza(1.4);
	  break;
	  case 4:
		ponteEnAngulo(270);
		avanza(1);
	  break;
	  case 5:
		ponteEnAngulo(225);
		avanza(1.4);
	  break;
	  case 6:
		ponteEnAngulo(180);
		avanza(1);
	  break;
	  case 7:
		ponteEnAngulo(135);
		avanza(1.4);
	  break;
	  default:
	  break;
  }

  
  
  //msg.linear.x = 0.1;
  //ROS_INFO("Enviando %f como velocidad en x", msg.linear.x);
  //mueve_robot.publish(msg);
  
  cout << "Voy a enviar un 77" << endl;
  cout << "Y el ángulo es " << angulo << endl;
  
  res.resultado = 77;
  
  return true;
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mueveRobot_apunto");

  ros::NodeHandle n;

  mueve_robot = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
  ros::Subscriber odom_subs = n.subscribe("odom", 1000, odomCallback);
  ros::ServiceServer service = n.advertiseService("mover_en_direccion", moverEnDireccion);

  ros::Rate loop_rate(10);

  
  bool distanciaPedida = false;
  float distancia = 0;
  geometry_msgs::Point puntoDeInicio;
  while (ros::ok())
  {
    

    ros::spinOnce();

    loop_rate.sleep();
   
  }


  return 0;
}

