/// @author Sakir Furkan Yondem
/// @date April 7, 2022
/// In diesem Skript wird turtle1 über Konsole manuel gesteuert. 
/// Turtle2 faehrt in die richtung von turtle1 mit einer bestimmten Distanz.  
/// wenn turtle1 stoppt, stoppt turtle2 nach der Distanz. 


#include <cstdlib> // Use for the absolute value method abs()
#include <iostream> // Enables command line input and output
#include "turtlesim/Spawn.h"
#include "ros/ros.h" // Necessary header files for ROS
#include "geometry_msgs/Twist.h" // Twist messages (linear & angular velocity)
#include "geometry_msgs/Pose2D.h" // x, y position and theta orientation
#include "turtlesim/Pose.h" // x, y, theta, linear & angular velocity
 
// Für die Eingabe der Zahlen aus der Tastatur
using namespace std;
 
geometry_msgs::Twist turtle1, turtle2; // Linear and angular velocity in m/s 
geometry_msgs::Pose2D current_turtle1, current_turtle2; // Current x, y, and theta
geometry_msgs::Pose2D desired_turtle1, desired_turtle2; // Desired x, y, and theta

const double K_l = 0.5; // Für die linere Geschwindigkeit der Roboter
 
const double K_a = 0.5; // Für die Winkelsgeschwindigkeit der Roboter
 
const double distanceTolerance = 0.1; 
 
const double angleTolerance = 0.1;

// Die Kordinaten des Roboters"turtle1" werden für den gezielten Weg  manuel eingegeben.
// Die Anfangswete für die Roboter "turtle1 und turtle2" eingegeben.  

void Anfangswerte() {
  float goal_x;
  cout << "X_Pose:";
  cin >> goal_x;
  const double GOAL_x = goal_x;
  float goal_y;
  cout << "Y_Pose:";
  cin >> goal_y;
  const double GOAL_y = goal_y;

  desired_turtle1.x = GOAL_x;
  desired_turtle1.y = GOAL_y;
  desired_turtle2.x = current_turtle1.x;
  desired_turtle2.y = current_turtle1.y;

  turtle1.linear.x = 0.0;
  turtle1.linear.y = 0.0;
  turtle1.linear.z = 0.0;
  turtle1.angular.x = 0.0;
  turtle1.angular.y = 0.0;
  turtle1.angular.z = 0.0;

  turtle2.linear.x = 0.0;
  turtle2.linear.y = 0.0;
  turtle2.linear.z = 0.0;
  turtle2.angular.x = 0.0;
  turtle2.angular.y = 0.0;
  turtle2.angular.z = 0.0;
}

// Der gezielten Weg des Roboter"turtle1" berechnet.
double getDistanceToWaypoint_turtle1() {
  return sqrt(pow(desired_turtle1.x - current_turtle1.x, 2) + pow(
    desired_turtle1.y - current_turtle1.y, 2));
}

// Der gezielten Weg des Roboter"turtle2" berechnet.
// Der gewünschte Weg von Turtle2 sollte der aktuelle Standort von Turtle1 sein
double getDistanceToWaypoint_turtle2() {
  return sqrt(pow(current_turtle1.x - current_turtle2.x, 2) + pow(
    current_turtle1.y - current_turtle2.y, 2));
}
 
// Der Radianswert wird berechnet.  
double getHeadingError_turtle1() {
 
  double deltaX_turtle1 = desired_turtle1.x - current_turtle1.x;
  double deltaY_turtle1 = desired_turtle1.y - current_turtle1.y;
  double waypointHeading_turtle1 = atan2(deltaY_turtle1, deltaX_turtle1);
  double headingError_turtle1 = waypointHeading_turtle1 - current_turtle1.theta;   
   
  // Ob die Winkelfehler im Bereich von PI sind. 
  if (headingError_turtle1 > M_PI) {
    headingError_turtle1 = headingError_turtle1 - (2 * M_PI);
  } 
  if (headingError_turtle1 < -M_PI) {
    headingError_turtle1 = headingError_turtle1 + (2 * M_PI);
  } 
   
  return headingError_turtle1;
}

double getHeadingError_turtle2() {
 
  double deltaX_turtle2 = current_turtle1.x - current_turtle2.x;
  double deltaY_turtle2 = current_turtle1.y - current_turtle2.y;
  double waypointHeading_turtle2 = atan2(deltaY_turtle2, deltaX_turtle2);
  double headingError_turtle2 = waypointHeading_turtle2 - current_turtle2.theta;   
   
  // Die Einschaerenkung der Richtungsfehler zwischen dem Wert von PI.
  if (headingError_turtle2 > M_PI) {
    headingError_turtle2 = headingError_turtle2 - (2 * M_PI);
  } 
  if (headingError_turtle2 < -M_PI) {
    headingError_turtle2 = headingError_turtle2 + (2 * M_PI);
  } 
   
  return headingError_turtle2;
}

// Wenn die Roboter das Ziel noch nicht erreicht haben, stellen Sie den Velocity-Wert ein.
// Andernfalls stoppen die Roboter.
void setVelocity() {
 
  double distanceToWaypoint_turtle1 = getDistanceToWaypoint_turtle1();
  double headingError_turtle1 = getHeadingError_turtle1();
  double distanceToWaypoint_turtle2 = getDistanceToWaypoint_turtle2();
  double headingError_turtle2 = getHeadingError_turtle2();
 
  // Die Kontrollierung der Distaz von turtle1 zwischen der aktuellen Pose und dem Toleranzwert.
  if ((abs(distanceToWaypoint_turtle1) > distanceTolerance)) {
    
    // Einstellung der Richtung von turtle1
    if (abs(headingError_turtle1) > angleTolerance) {
      turtle1.linear.x = 0.0;
      turtle1.angular.z = K_a  * headingError_turtle1;
    }
    // Wenn der Winkel von turtle1 gleich der Winkel des gezielten Wegs 
    // Geht turtle1 zum gewünschten Punkt.
    else {
      turtle1.linear.x = K_l * distanceToWaypoint_turtle1;
      turtle1.angular.z = 0.0;    
    }
  }
  else {
    cout << "Turtle1 kommt im Punt an!" << endl << endl;
    turtle1.linear.x = 0.0;
    turtle1.angular.z = 0.0; 
  }
    // Die Kontrollierung der Distaz zwischen der aktuellen Pose von turtle2 und dem Toleranzwert.
    if ((abs(distanceToWaypoint_turtle2) > (distanceTolerance+1.0))) {
    
    // Einstellung der Richtung von turtle2
    if (abs(headingError_turtle2) > angleTolerance) {
      turtle2.linear.x = 0.0;
      turtle2.angular.z = (K_a/0.1)  * headingError_turtle2;
    }

    // Wenn der Winkel von turtle1 gleich der Winkel des gezielten Wegs 
    // Geht turtle2 zum gewünschten Punkt.
    else {
      turtle2.linear.x = (K_l+0.2) * distanceToWaypoint_turtle2;
      turtle2.angular.z = 0.0;    
    }
  }
  else {
    cout << "Turtle2 kommt im Punt an!" << endl << endl;
    turtle2.linear.x = 0.0;
    turtle2.angular.z = 0.0; 
  }
}

// Das Update der Pose von turtle1 
void updatePose_turtle1(const turtlesim::PoseConstPtr &currentPose_turtle1) {
  current_turtle1.x = currentPose_turtle1->x;
  current_turtle1.y = currentPose_turtle1->y;
  current_turtle1.theta = currentPose_turtle1->theta;
}

// Das Update der Pose von turtle2
void updatePose_turtle2(const turtlesim::PoseConstPtr &currentPose_turtle2) {
  current_turtle2.x = currentPose_turtle2->x;
  current_turtle2.y = currentPose_turtle2->y;
  current_turtle2.theta = currentPose_turtle2->theta;
}

int main(int argc, char **argv) {
 
  Anfangswerte();  
 
  // Initiate ROS
  ros::init(argc, argv, "praktikumversuch_1_2");
     
  // Der Ansatz von turtle2
  ros::NodeHandle node;
  ros::ServiceClient client = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  srv.request.x= 4.5;
  srv.request.y= 4.5;
  client.call(srv);
  
  // Subscriber für die Pose der Roboter
  ros::Subscriber currentPoseSub_turtle1 =
    node.subscribe("turtle1/pose", 0, updatePose_turtle1);
  ros::Subscriber currentPoseSub_turtle2 =
    node.subscribe("turtle2/pose", 0, updatePose_turtle2);
     
  // Publish für die linere Geschwindigkeit und die Winkelsgeschwindigkeit.
  ros::Publisher velocityPub_turtle1 =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);
  ros::Publisher velocityPub_turtle2 =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 0);

  // In diesem Fall möchte ich 10 Zyklen pro Sekunde 
  ros::Rate loop_rate(10); 
 
  while (ros::ok()) {
 
    ros::spinOnce();

    setVelocity();
 
    // Publish für Geschwindigkeit der Roboter
    velocityPub_turtle1.publish(turtle1);
    velocityPub_turtle2.publish(turtle2);

    // Ausgabe auf der Konsole
    cout << "Current_x_turtle1 = " << current_turtle1.x << endl
         << "Current_x_turtle2 = " << current_turtle2.x << endl
         << "Desired_x_turtle1 = " << desired_turtle1.x <<  endl
         << "Distance to Goal für Turtle1 = " << getDistanceToWaypoint_turtle1() << " m" << endl
         << "Linere Geschwindigkeit von Turtle1 (x) = " << turtle1.linear.x << " m/s" << endl
         << endl;   
 
    // 10Hz
    loop_rate.sleep();
  }
 
  return 0;
}
