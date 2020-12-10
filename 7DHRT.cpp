
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "sim_snake_class.h"

using namespace std;

float shift = 0;

float Helical_motor_angle[32];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_snake_robot_master");

  Sim_snake_IO sim_io;

  for(int i=0; i<JOINT_NUM; i++){
    sim_io.target_motor_angle[i] = 0.0;
    sim_io.current_motor_angle[i] = 0.0;
  }

  ros::NodeHandle node;
  sim_io.Set_publisher(node);

  ros::Subscriber sub_current_position = node.subscribe("current_motor_angle", 100, &Sim_snake_IO::Sub_current_motor_angle, &sim_io);
  ros::Subscriber sub_time = node.subscribe("simulationTime", 1, &Sim_snake_IO::Sub_time, &sim_io);
  ros::Subscriber sub_motor_angle = node.subscribe("joint_target_position", 100,  &Sim_snake_IO::Sub_motor_angle, &sim_io);
  ros::Subscriber sub_force_sensor = node.subscribe("force_data", 1, &Sim_snake_IO::Sub_force_data, &sim_io);
  ros::Subscriber sub_imu_data = node.subscribe("imu_data_vrep", 1, &Sim_snake_IO::Sub_imu_data, &sim_io);
  ros::Subscriber sub_imu_rpy = node.subscribe("sim_imu_data", 1, &Sim_snake_IO::Sub_imu_roll_pitch_yaw, &sim_io);

  ros::Rate loop_rate(100);

  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {

    sim_io.Pub_motor_angle();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
