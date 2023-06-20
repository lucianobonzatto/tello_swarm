#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#define POUSADO 0
#define TAKEOFF 1
#define LAND 2
#define VOANDO 3
#define ACIONA_PID_1 4
#define PARA_PID 6
#define FIXO 8
#define LINE 9
#define ACIONA_PID_2 10

using namespace std;

geometry_msgs::Point gotoPose;

ros::Publisher pose_pub;

int state;
int pid_flag_ID1, pid_flag_ID8;
int running;

void pidFlag_ID1_Callback(const std_msgs::Bool::ConstPtr& msg){
  pid_flag_ID1 = (int) msg->data;
}
void pidFlag_ID8_Callback(const std_msgs::Bool::ConstPtr& msg){
  pid_flag_ID8 = (int) msg->data;
}

void keyboardThread(){
  char teste[1];

  while(running){
    std::cin >> teste;
    switch(state){
      case POUSADO:
        if(teste[0] == 'l')
          state = LAND;
        else if(teste[0] == 's')
          state = TAKEOFF;
        else
          cout << "\t transição invalida para esse stado" << endl;
        break;
        
      case TAKEOFF:
        cout << "\t transição invalida para esse stado" << endl;
        break;
        
      case LAND:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case VOANDO:
        if(teste[0] == 'l')
          state = LAND;
        else if(teste[0] == 'p')
          state = ACIONA_PID_1;
        else if(teste[0] == 't')
          state = ACIONA_PID_2;
        else
          cout << "\t transição invalida para esse stado" << endl;
        break;

      case ACIONA_PID_1:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case ACIONA_PID_2:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case PARA_PID:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case LINE:
        if(teste[0] == 'l')
          state = LAND;
        else
          cout << "\t transição invalida para esse stado" << endl;
        break;

      case FIXO:
        if(teste[0] == 'l')
          state = LAND;
        else
          cout << "\t transição invalida para esse stado" << endl;
        break;
    }
  }
}

void includePoints(std::vector<geometry_msgs::Pose>* line_ID1, float x, float y, float z){
  geometry_msgs::Pose point;
  point.position.x = x;
  point.position.y = y;
  point.position.z = z;
  line_ID1->push_back(point);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_two_control");
  ros::NodeHandle nh("");
  ros::Rate loop_rate(10);

  ros::Publisher takeoff_pub_ID1 = nh.advertise<std_msgs::Empty>("/tello_ID1/takeoff", 1);
//  ros::Publisher takeoff_pub_ID1 = nh.advertise<std_msgs::Empty>("/tello_ID1/manual_takeoff", 1);
  ros::Publisher land_pub_ID1 = nh.advertise<std_msgs::Empty>("/tello_ID1/land", 1);
  ros::Publisher goto_pub_ID1 = nh.advertise<geometry_msgs::Pose>("tello_ID1/pid/goto", 1);
  ros::Publisher pid_start_pub_ID1 = nh.advertise<std_msgs::Bool>("tello_ID1/pid/start", 1);
  ros::Subscriber pid_flag_sub_ID1 = nh.subscribe<std_msgs::Bool>("tello_ID1/pid/flag", 1, &pidFlag_ID1_Callback);

  ros::Publisher takeoff_pub_ID8 = nh.advertise<std_msgs::Empty>("/tello_ID8/takeoff", 1);
//  ros::Publisher takeoff_pub_ID8 = nh.advertise<std_msgs::Empty>("/tello_ID8/manual_takeoff", 1);
  ros::Publisher land_pub_ID8 = nh.advertise<std_msgs::Empty>("/tello_ID8/land", 1);
  ros::Publisher goto_pub_ID8 = nh.advertise<geometry_msgs::Pose>("tello_ID8/pid/goto", 1);
  ros::Publisher pid_start_pub_ID8 = nh.advertise<std_msgs::Bool>("tello_ID8/pid/start", 1);
  ros::Subscriber pid_flag_sub_ID8 = nh.subscribe<std_msgs::Bool>("tello_ID8/pid/flag", 1, &pidFlag_ID8_Callback);

  int state_square = 0, flag_square = 0;

  state = 0;
  running = 1;
  pid_flag_ID1 = 0;
  pid_flag_ID8 = 0;
  std::thread keyboardRead(keyboardThread);


  std_msgs::Empty emptyMsg;
  std_msgs::Bool boolMsg;
  std::vector<geometry_msgs::Pose> line_ID1;
  std::vector<geometry_msgs::Pose> line_ID8;

  includePoints(&line_ID1, 0.2, 0.00, 1.7);
  includePoints(&line_ID1, 0.6, 0.00, 1.7);
  includePoints(&line_ID1, 0.2, 0.00, 1.7);
  //includePoints(&line_ID1, 0.2, 0.00, 1.0);

  includePoints(&line_ID8, -0.2, 0.40, 1.7);
  includePoints(&line_ID8, -0.6, 0.40, 1.7);
  includePoints(&line_ID8, -0.2, 0.40, 1.7);
  //includePoints(&line_ID8, -0.2, 0.40, 1.0);

  while(ros::ok()){
    ros::spinOnce();

    cout << "==============================" << endl;
    cout << "estado atual: " << state << endl;

    switch(state){
      case POUSADO:
        break;
        
      case TAKEOFF:
        cout << "TAKEOFF START" << endl;
        takeoff_pub_ID1.publish(emptyMsg);
        takeoff_pub_ID8.publish(emptyMsg);
        sleep(5);
        state = VOANDO;
        cout << "TAKEOFF END" << endl;
        break;
        
      case LAND:
        cout << "LAND START" << endl;
        boolMsg.data = 0;
        pid_start_pub_ID1.publish(boolMsg);
        pid_start_pub_ID8.publish(boolMsg);

        land_pub_ID1.publish(emptyMsg);
        land_pub_ID8.publish(emptyMsg);
        sleep(5);
        state = POUSADO;
        cout << "LAND END" << endl;
        break;

      case VOANDO:
        break;

      case ACIONA_PID_1:
        cout << "ACIONA_PID_1 START" << endl;
        boolMsg.data = 1;
        pid_start_pub_ID1.publish(boolMsg);
        pid_start_pub_ID8.publish(boolMsg);
        state = FIXO;
        cout << "ACIONA_PID_1 END" << endl;
        break;

      case ACIONA_PID_2:
        cout << "ACIONA_PID_2 START" << endl;
        boolMsg.data = 1;
        pid_start_pub_ID1.publish(boolMsg);
        pid_start_pub_ID8.publish(boolMsg);
        state = LINE;
        cout << "ACIONA_PID_2 END" << endl;
        break;

      case PARA_PID:
        cout << "PARA_PID START" << endl;
        boolMsg.data = 0;
        pid_start_pub_ID1.publish(boolMsg);
        pid_start_pub_ID8.publish(boolMsg);
        state = VOANDO;
        cout << "PARA_PID END" << endl;
        break;

      case LINE:
        cout << "LINE START" << endl;
        //state = PARA_PID;
        cout << "ponto " << state_square << " flag: " << pid_flag_ID1;
        cout << "\n\tID1 x: " << line_ID1[state_square].position.x << " y: " << line_ID1[state_square].position.y << " z: " << line_ID1[state_square].position.z << endl;
        cout << "\n\tID8 x: " << line_ID8[state_square].position.x << " y: " << line_ID8[state_square].position.y << " z: " << line_ID8[state_square].position.z << endl;
        if(flag_square == 0){
          //publica o proximo destino
          goto_pub_ID1.publish(line_ID1[state_square]);
          goto_pub_ID8.publish(line_ID8[state_square]);
          //garante que o pid foi atualizado
          if(pid_flag_ID1 == 0 && pid_flag_ID8 == 0)
            flag_square = 1;
        }
        else{
          //verifica se chegou
          if(pid_flag_ID1 == 1 && pid_flag_ID8 == 1){
            flag_square = 0;
            state_square++;
            //verifica se terminou a rotina
            if(state_square >= line_ID1.size()){
              state_square = 0;
              state = PARA_PID;
            }
          }
        }
        cout << "LINE END" << endl;
        break;

      case FIXO:
        cout << "FIXO START" << endl;
        //state = PARA_PID;
        cout << "FIXO END" << endl;
        break;
    }
    loop_rate.sleep();
  }
  running = 0;
}