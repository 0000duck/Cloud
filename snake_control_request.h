/**
 * @file snake_control_request.h
 * @brief ヘビ型ロボットに動作要求するクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/08/31
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_SNAKE_CONTROL_REQUEST_H_
#define SNAKE_CONTROL_SRC_SNAKE_CONTROL_REQUEST_H_

#include <vector>

#include <ros/ros.h>
#include <snake_msgs/SnakeJointCommand.h>
#include <snake_msgs/SnakeJointData.h>
#include <snake_msgs/SnakeIMUCommand.h>
#include <snake_msgs/SnakeCOPCommand.h>

class SnakeControlRequest {
 public:
  static void Initialize() {
    ros::NodeHandle node_handle;
    pub_joint_command_ = node_handle.advertise<snake_msgs::SnakeJointCommand>("joint_command", 100);
    pub_joint_target_position_ = node_handle.advertise<snake_msgs::SnakeJointData>("joint_target_position", 100);
    pub_imu_command_ = node_handle.advertise<snake_msgs::SnakeIMUCommand>("imu_command", 20);
    pub_cop_command_ = node_handle.advertise<snake_msgs::SnakeCOPCommand>("cop_command", 20);
  }

  //--- Joint -----------------------//
  // Slaveに記憶させるサーボのIDリスト関係
  static void RequestJointSetIDList();
  static void RequestJointReadIDList();
  static void RequestJointPingAll();
  // 関節への動作要求
  static void RequestJointClearErrorAll();
  static void RequestJointActivate(uint8_t joint_index);
  static void RequestJointActivateAll();
  static void RequestJointFree(uint8_t joint_index);
  static void RequestJointFreeAll();
  static void RequestJointHold(uint8_t joint_index);
  static void RequestJointHoldAll();
  static void RequestJointReset(uint8_t joint_index);
  static void RequestJointResetAll();
  // 関節の情報の読み込み
  static void RequestJointReadPositionAll();
  static void RequestJointReadVelosityAll();
  static void RequestJointReadCurrentAll();
  static void RequestJointReadVoltageAll();
  static void RequestJointReadMotorTemperatureAll();
  static void RequestJointReadPositionVelosityAll();
  static void RequestJointReadPositionCurrentAll();
  static void RequestJointReadPositionVelosityCurrentAll();
  // 関節へのパラメータ書き込み
  static void RequestJointSetPosition(std::vector<double> joint_angle);
  static void RequestJointSetPositionRange(std::vector<double> joint_angle, int32_t start, int32_t last);
  static void RequestJointSetPIDGainAll(uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);


  static void RequestJointSetPIDGain(uint8_t joint_index, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain);


  static void RequestJointSetLockParameterAll(uint8_t time_ms, uint8_t power, uint8_t output);

  //--- IMU -------------------------//
  static void RequestIMURollPitchYaw(uint8_t imu_index);
  static void RequestIMUGyro(uint8_t imu_index);
  static void RequestIMUAccel(uint8_t imu_index);

  //--- COP -------------------------//
  static void RequestCOPData(uint8_t cop_index);
  static void RequestCOPDataAll();

  static void PublishJointTargetPositionAllZero(uint8_t num_joint);

 private:
  //--- Publisher ---//
  static ros::Publisher pub_joint_target_position_;  // 汎用(表示などに使う)
  static ros::Publisher pub_joint_command_;  // ロボット用
  static ros::Publisher pub_imu_command_;
  static ros::Publisher pub_cop_command_;
};

#endif /* SNAKE_CONTROL_SRC_SNAKE_CONTROL_REQUEST_H_ */
