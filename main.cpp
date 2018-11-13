
#include <stdlib.h>
#include <stdint.h>
#include <ros/ros.h>
#include "snake_control.h"

extern RobotSpec spec;
extern joy_handler::JoySelectedData joystick;


//#define SETTING_MODE  // パラメータ，サーボIDリストを設定したいとき，コメントアウトして実効

#ifndef SETTING_MODE
// 通常

void timerCallback(const ros::TimerEvent& event)
{
  //ROS_INFO("[SnakeControl] time1");

  static int mode = 0;

  // if (joystick.button_ps) {
  //   mode = 0;    //ストップモード
  //   ROS_INFO("stop mode");
  //   //ros::Duration(0.5).sleep();
  // }

  if (joystick.button_select and joystick.button_triangle) {
    mode=1;    //横うねりモード
    ROS_INFO("lateral undulation");
    //ros::Duration(0.5).sleep();
  }
  if(mode==1){//横うねりモード

    SnakeControl::OperateMoveLateralUndulation(joystick);

  }

  if (joystick.button_select and joystick.button_circle) {
    mode=2;    //サイドワインディングモード
    ROS_INFO("sidewinding");
    //ros::Duration(0.5).sleep();
  }
  if(mode==2){  //サイドワインディングモード  

    SnakeControl::OperateMoveSidewinding(joystick);

  }

  if (joystick.button_select and joystick.button_cross) {
    mode=3;//ラテラルローリングモード
    ROS_INFO("lateral rolling");
  }
  if(mode==3){//ラテラルローリングモード
  
    SnakeControl::OperateMoveLateralRolling(joystick);

  }

  if (joystick.button_select and joystick.button_square) {
    mode=4;//曲螺旋捻転モード
    ROS_INFO("helical curve");
    //ros::Duration(0.5).sleep();
  }
  if(mode==4){//曲螺旋捻転モード

    SnakeControl::OperateMoveHelicalCurve(joystick);
   // SnakeControlRequest::RequestCOPDataAll(); 
 //   ros::Duration(0.5).sleep();	
  }
  
  if (joystick.button_select and joystick.closs_key_up) {
    mode=5;//直線モード
    ROS_INFO("straight");
    //ros::Duration(0.5).sleep();
  }
  if(mode==5){//直線モード

    SnakeControl::OperateMoveStraight(joystick);
    //   SnakeControlRequest::RequestCOPDataAll(); 

  }
  
  // if (joystick.button_select and joystick.closs_key_left) {
  //   mode=6;//テストモード
  //   ROS_INFO("test");
  //   //ros::Duration(0.5).sleep();
  // }
  // if(mode==6){//テストモード

  //   SnakeControl::OperateMoveTest(joystick);

  // }
  // if (joystick.button_select and joystick.closs_key_left) {
  //    mode = 0;    //ストップモード
  //    ROS_INFO("stop mode");
  //    ros::Duration(0.5).sleep();
  // }

   if (joystick.button_select and joystick.closs_key_left) {
    ROS_INFO("[SnakeControl] request cop sensor data");  
    SnakeControlRequest::RequestCOPDataAll();   
    //SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0); 
    ros::Duration(0.5).sleep();
   }

   if (joystick.button_select and joystick.closs_key_right) {
    ROS_INFO("[SnakeControl] change PID gain");  
    SnakeControl::RequestJointSetPIDGainAll(12.0, 0.0, 0.0); 
    //    SnakeControl::RequestJointSetPIDGain(19, 1.0, 0.0, 0.0);
    ros::Duration(0.5).sleep();
   }
   /*  
  if (joystick.button_select and joystick.closs_key_down) {
    mode=7;//常螺旋byTakemori
    ROS_INFO("helical rolling");
    //ros::Duration(0.5).sleep();
  }
  if(mode==7){//常螺旋byTakemori

    SnakeControl::OperateMoveHelicalRolling2(joystick);

    }*/
  
  
  ////////////////////////////////////////////////
  //全モード共通
  
  if (joystick.button_select and joystick.button_start) {//サーボオフ 
    ROS_INFO("[SnakeControl] Servo OFF");
    SnakeControlRequest::RequestJointFreeAll();
    ros::Duration(0.5).sleep();
  }else if (joystick.button_start) {//サーボオン 
    ROS_INFO("[SnakeControl] Servo ON");
    SnakeControl::RequestJointSetPIDGainAll(10.0, 0.0, 0.0); 
    SnakeControl::RequestJointSetPIDGain(19, 1.0, 0.0, 0.0);
    ros::Duration(0.5).sleep();
    SnakeControlRequest::RequestJointActivateAll();
    ros::Duration(0.5).sleep();
  }

  if (joystick.button_ps) {//エラーを消す
    ROS_INFO("[SnakeControl] Clear joint error");
    SnakeControlRequest::RequestJointClearErrorAll();   
    ros::Duration(0.5).sleep();
    //ROS_INFO("[SnakeControl] request cop sensor data");
    //SnakeControlRequest::RequestCOPData(0);   
    //SnakeControlRequest::RequestCOPDataAll();   
    //ros::Duration(0.5).sleep();
    //ROS_INFO("[SnakeControl] Ping All");
    //SnakeControlRequest::RequestJointPingAll();   
    //ros::Duration(0.5).sleep();
  }
 

  if ( joystick.button_r3 or joystick.button_l3 ) {//ノード再起動
    pid_t pid;
    char path_name[64];
    char process_name[64];
    char arg_name1[64];
    char arg_name2[64];
    strcpy( path_name, "/opt/ros/indigo/bin/" );
    strcpy( process_name, "rosrun" );
    strcpy( arg_name1, "snake_control" );
    strcpy( arg_name2, "snake_control" );
    pid = fork();

    switch( pid ){
    case -1:
      perror( "fork failed" );
      exit(1);
    case 0: // 子プロセス
      chdir( path_name );
      execl( process_name, process_name, arg_name1, arg_name2, (char *)NULL);
      //execl( process_name, process_name, (char *)NULL );
      break;
    default: // 親プロセス
      break;
    }
    ros::Duration(0.5).sleep();

    //system("/opt/ros/indigo/bin/rosrun snake_control snake_control");
    //exit(0);
  }



  static long long unsigned int i;
  i++;

  /*
  if ( (i % 5) == 0) {
    //ROS_INFO("[SnakeControl] request joint angle");
    SnakeControlRequest::RequestJointReadPositionAll();
  }else if ( (i % 5) == 1) {

  }else if ( (i % 5) == 2) {
    SnakeControlRequest::RequestCOPDataAll();   
  }else if ( (i % 5) == 3) {

  }else if ( (i % 5) == 4) {
    SnakeControlRequest::RequestIMURollPitchYaw(0);
  }



  }*/


  if ( (i % 10) == 0) {
    //ROS_INFO("[SnakeControl] request joint angle");
    SnakeControlRequest::RequestJointReadPositionAll();
  }else if ( (i % 10) == 1) {

  }else if ( (i % 10) == 2) {
    SnakeControlRequest::RequestCOPDataAll();   
  }else if ( (i % 10) == 3) {

  }else if ( (i % 10) == 4) {
    SnakeControlRequest::RequestIMURollPitchYaw(0);
  }else if ( (i % 10) == 5) {

  }else if ( (i % 10) == 6) {
       
  }else if ( (i % 10) == 7) {

  }else if ( (i % 10) == 8) {
    
  }else if ( (i % 10) == 9) {
    
  }



}

//センサー用タイマー(20msecに一回呼ばれる)
void timerCallback2(const ros::TimerEvent& event)
{
  // //ROS_INFO("[SnakeControl] time2");

  // static long long unsigned int i;
  // i++;

  // if ( (i % 5) == 0) {
  //   //ROS_INFO("[SnakeControl] request joint angle");
  //   SnakeControlRequest::RequestJointReadPositionAll();
  // }else if ( (i % 5) == 1) {

  // }else if ( (i % 5) == 2) {
  //   SnakeControlRequest::RequestCOPDataAll();   
  // }else if ( (i % 5) == 3) {

  // }else if ( (i % 5) == 4) {
  //   SnakeControlRequest::RequestIMURollPitchYaw(0);
  // }
}

int main(int argc, char **argv) {

   ros::init(argc, argv, "snake_control_node");
  ROS_INFO("Initialized : snake_control_node");

  SnakeControl::Initialize();

  //ros::NodeHandle node_handle("~");
  ros::NodeHandle node_handle;


  //    node_handle.param("loop_rate", loop_rate_, 1.0);
  node_handle.param("loop_rate", SnakeControl::loop_rate_, 50.0);// [Hz]
  //  node_handle.param("loop_rate", SnakeControl::loop_rate_, 20.0);// [Hz]

  //    sub_joy_selected_data_ = node_handle.subscribe("joy_selected_data", 1, SnakeControl::CallBackOfJoySelectedData);
  ros::Subscriber sub_joy_selected_data_ = node_handle.subscribe("joy_selected_data", 1, &SnakeControl::CallBackOfJoySelectedData);
  SnakeControl::sampling_time_ = 1.0/SnakeControl::loop_rate_;  // [s] サンプリングタイム loop_rateの逆

  //ros::Rate rate(SnakeControl::loop_rate());


  // コマンド送信用タイマーを作る Durationの単位は[sec]
  // ROSシステムとの通信のためのノードハンドルを宣言
  ros::NodeHandle nh;
  //ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback); //100msecでやる;
  ros::Timer timer = nh.createTimer(ros::Duration(SnakeControl::sampling_time_), timerCallback);// [s] サンプリングタイムでタイマー呼び出し

  //センサー用タイマー
  ros::NodeHandle nh2;
  //ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback); //100msecでやる;
  ros::Timer timer2 = nh2.createTimer(ros::Duration(0.02), timerCallback2);// [s] サンプリングタイムでタイマー呼び出し

  // コールバックをずっと待つ(while())
  ros::spin();

  /*
  while(ros::ok()) {
    SnakeControl::RequestJointReadPositionVelosityCurrentAll();
//    SnakeControl::RequestJointReadPositionAll();
//    SnakeControl::RequestIMURollPitchYaw(0);
    ros::spinOnce();

    //ROS_INFO("hogehoge");

    rate.sleep();
  }
  */
}

// パラメータ，サーボIDリストの設定
#else
int main(int argc, char **argv) {
  ros::init(argc, argv, "snake_control_node");
  SnakeControl::Initialize();
  ROS_INFO("Initialized : snake_control_node");
  ros::Rate rate(0.1);
  while(ros::ok()) {
    ROS_INFO("Write Servo ID list to All Slave MCU");
    SnakeControl::RequestJointSetIDList();
    SnakeControl::RequestJointReadIDList();

    ROS_INFO("Ping All Servo");
    SnakeControl::RequestJointPingAll();

//    ROS_INFO("Write LockParameter to Servo");
//    SnakeControl::RequestJointSetLockParameterAll(5, 50, 10);

//    ROS_INFO("Write PID Gain to Servo");
//    SnakeControl::RequestJointSetPIDGainAll(100.0, 100.0, 100.0);  // KONDOの場合 [%]
//    SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0);  // Dynamixelの場合 そのまま入る

    rate.sleep();
  }
}
#endif
