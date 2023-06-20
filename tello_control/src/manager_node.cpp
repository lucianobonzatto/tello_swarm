#include <thread>
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
#define ACIONA_PID_2 5
#define PARA_PID 6
#define SQUARE 7
#define FIXO 8
#define TRIANGLE 9
#define ACIONA_PID_3 10

using namespace std;

geometry_msgs::Point gotoPose;

ros::Publisher pose_pub;

int state;
int pid_flag;
int running;

void pidFlagCallback(const std_msgs::Bool::ConstPtr& msg){
  pid_flag = (int) msg->data;
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
        else if(teste[0] == 'q')
          state = ACIONA_PID_2;
        else if(teste[0] == 't')
          state = ACIONA_PID_3;
        else
          cout << "\t transição invalida para esse stado" << endl;
        break;

      case ACIONA_PID_1:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case ACIONA_PID_2:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case ACIONA_PID_3:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case PARA_PID:
        cout << "\t transição invalida para esse stado" << endl;
        break;

      case SQUARE:
        if(teste[0] == 'l')
          state = LAND;
        else
          cout << "\t transição invalida para esse stado" << endl;
        break;

      case TRIANGLE:
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "manager_control");
  ros::NodeHandle nh(""), nh_param("~");
  ros::Rate loop_rate(10);

  ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/tello_ID1/takeoff", 1);
  ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/tello_ID1/land", 1);
  ros::Publisher goto_pub = nh.advertise<geometry_msgs::Pose>("tello_ID1/pid/goto", 1);
  ros::Publisher pid_start_pub = nh.advertise<std_msgs::Bool>("tello_ID1/pid/start", 1);
  ros::Subscriber pid_flag_sub = nh.subscribe<std_msgs::Bool>("tello_ID1/pid/flag", 1, &pidFlagCallback);

  int state_square = 0, flag_square = 0;

  state = 0;
  running = 1;
  pid_flag = 0;
  std::thread keyboardRead(keyboardThread);


  std_msgs::Empty emptyMsg;
  std_msgs::Bool boolMsg;

  geometry_msgs::Pose ponto0;
  geometry_msgs::Pose ponto1;
  geometry_msgs::Pose ponto2;
  geometry_msgs::Pose ponto3;
  geometry_msgs::Pose ponto4;
  geometry_msgs::Pose ponto5;
  geometry_msgs::Pose ponto6;

  ponto0.position.x = 0.0;
  ponto0.position.y = 0.0;
  ponto0.position.z = 2.0;

  ponto1.position.x = 0.45;
  ponto1.position.y = 0.45;
  ponto1.position.z = 2.0;

  ponto2.position.x = -0.45;
  ponto2.position.y = 0.45;
  ponto2.position.z = 2.0;

  ponto3.position.x = -0.45;
  ponto3.position.y = -0.15;
  ponto3.position.z = 2.0;
 
  ponto4.position.x = 0.45;
  ponto4.position.y = -0.15;
  ponto4.position.z = 2.0;

  ponto5.position.x = 0.45;
  ponto5.position.y = 0.45;
  ponto5.position.z = 2.0;

  ponto6.position.x = 0.0;
  ponto6.position.y = 0.0;
  ponto6.position.z = 2.0;

  geometry_msgs::Pose ponto0T;
  geometry_msgs::Pose ponto1T;
  geometry_msgs::Pose ponto2T;
  geometry_msgs::Pose ponto3T;

  ponto0T.position.x = 0.0;
  ponto0T.position.y = 0.35;
  ponto0T.position.z = 2.0;

  ponto1T.position.x = -0.7;
  ponto1T.position.y = 0.35;
  ponto1T.position.z = 2.0;

  ponto2T.position.x = 0.0;
  ponto2T.position.y = 0.35;
  ponto2T.position.z = 2.0;
 
  ponto3T.position.x = 0.0;
  ponto3T.position.y = 0.35;
  ponto3T.position.z = 1.0;


  while(ros::ok()){
    ros::spinOnce();

    cout << "========================" << endl;
    cout << "\t stado atual: " << state << endl;

    switch(state){
      case POUSADO:
        break;
        
      case TAKEOFF:
        cout << "TAKEOFF START" << endl;
        takeoff_pub.publish(emptyMsg);
        sleep(5);
        state = VOANDO;
        cout << "TAKEOFF END" << endl;
        break;
        
      case LAND:
        cout << "LAND START" << endl;
        boolMsg.data = 0;
        pid_start_pub.publish(boolMsg);
        land_pub.publish(emptyMsg);
        sleep(5);
        state = POUSADO;
        cout << "LAND END" << endl;
        break;

      case VOANDO:
        break;

      case ACIONA_PID_1:
        cout << "ACIONA_PID_1 START" << endl;
        boolMsg.data = 1;
        pid_start_pub.publish(boolMsg);
        state = FIXO;
        cout << "ACIONA_PID_1 END" << endl;
        break;

      case ACIONA_PID_2:
        cout << "ACIONA_PID_2 START" << endl;
        boolMsg.data = 1;
        pid_start_pub.publish(boolMsg);
        state = SQUARE;
        cout << "ACIONA_PID_2 END" << endl;
        break;

      case ACIONA_PID_3:
        cout << "ACIONA_PID_3 START" << endl;
        boolMsg.data = 1;
        pid_start_pub.publish(boolMsg);
        state = TRIANGLE;
        cout << "ACIONA_PID_3 END" << endl;
        break;

      case PARA_PID:
        cout << "PARA_PID START" << endl;
        boolMsg.data = 0;
        pid_start_pub.publish(boolMsg);
        state = VOANDO;
        cout << "PARA_PID END" << endl;
        break;

      case SQUARE:
        cout << "SQUARE START" << endl;
        //state = PARA_PID;
        switch(state_square)
        {
        case 0:
          cout << "ponto 1 " << pid_flag << endl;
          if(flag_square == 0){
            //goto ponto 0
            goto_pub.publish(ponto0);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 1;
            }
          }
          break;
        case 1:
          cout << "ponto 2 " << pid_flag << endl;
          if(flag_square == 0){
            //goto ponto 1
            goto_pub.publish(ponto1);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 2;
            }
          }
          break;
        case 2:
          cout << "ponto 3 " << pid_flag  << endl;
          if(flag_square == 0){
            //goto ponto 2
            goto_pub.publish(ponto2);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 3;
            }
            
          }
          break;
        case 3:
          cout << "ponto 4 " << pid_flag  << endl;
          if(flag_square == 0){
            //goto ponto 3
            goto_pub.publish(ponto3);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 4;
            }
            
          }
          break;
        case 4:
          cout << "ponto 5 " << pid_flag  << endl;
          if(flag_square == 0){
            //goto ponto 4
            goto_pub.publish(ponto4);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 5;
            }
            
          }
          break;
        case 5:
          cout << "ponto 6 " << pid_flag  << endl;
          if(flag_square == 0){
            //goto ponto 5
            goto_pub.publish(ponto5);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 6;
            }
            
          }
          break;
        case 6:
          cout << "ponto 7 " << pid_flag  << endl;
          if(flag_square == 0){
            //goto ponto 6
            goto_pub.publish(ponto6);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = -1;
            }
            
          }
          break;
        default:
          flag_square = 0;
          state_square = 0;
          state = PARA_PID;
          break;
        }

        cout << "SQUARE END" << endl;
        break;

      case TRIANGLE:
        cout << "TRIANGLE START" << endl;
        //state = PARA_PID;
        switch(state_square)
        {
        case 0:
          cout << "ponto 0 " << pid_flag << endl;
          if(flag_square == 0){
            //goto ponto 0
            goto_pub.publish(ponto0T);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 1;
            }
          }
          break;
        case 1:
          cout << "ponto 1 " << pid_flag << endl;
          if(flag_square == 0){
            //goto ponto 1
            goto_pub.publish(ponto1T);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 2;
            }
          }
          break;
        case 2:
          cout << "ponto 2 " << pid_flag  << endl;

          if(flag_square == 0){
            //goto ponto 2
            goto_pub.publish(ponto2T);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = 3;
            }
            
          }
          break;
        case 3:
          cout << "ponto 3 " << pid_flag  << endl;
          if(flag_square == 0){
            //goto ponto 3
            goto_pub.publish(ponto3T);
            if(pid_flag == 0)
              flag_square = 1;
          }
          else{
            //verifica se chegou
            if(pid_flag == 1){
              flag_square = 0;
              state_square = -1;
            }
            
          }
          break;
        default:
          flag_square = 0;
          state_square = 0;
          state = PARA_PID;
          break;
        }

        cout << "TRIANGLE END" << endl;
        break;

      case FIXO:
        cout << "FIXO START" << endl;
        //state = PARA_PID;
        geometry_msgs::Pose ponto_fixo;

        ponto_fixo.position.x = 0.0;
        ponto_fixo.position.y = -0.15;
        ponto_fixo.position.z = 1.0;
        goto_pub.publish(ponto_fixo);
        cout << "FIXO END" << endl;
        break;
    }
    loop_rate.sleep();
  }

  running = 0;
}