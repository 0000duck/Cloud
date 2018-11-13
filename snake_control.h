/**
 * @file snake_control.h
 * @brief ヘビ型ロボットの制御を行うクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/05/14
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_SNAKE_CONTROL_H_
#define SNAKE_CONTROL_SRC_SNAKE_CONTROL_H_

#include <vector>

#include <ros/ros.h>
#include <joy_handler/JoySelectedData.h>
#include <snake_msgs/SnakeJointData.h>

#include "snake_control_request.h"

#include "helical_rolling_gait.h"
#include "flange_gait.h"
#include "crawler_gait.h"
#include "robot_spec.h"

#include "sidewinding_gait.h"
#include "lateral_rolling_gait.h"
#include "helical_curve_gait.h"

class SnakeControl : public SnakeControlRequest{
 public:
  enum ControlMode {
    CONTROL_MODE_INIT,
    CONTROL_MODE_ACTIVATE,
    CONTROL_MODE_MOVE,
  };
  enum GaitMode {
    GAIT_MODE_CRAWLER,
    GAIT_MODE_HELICAL_ROLLING,
    GAIT_MODE_FLANGE,

    GAIT_MODE_SIDEWINDING,
    GAIT_MODE_LATERAL_UNDULATION,
    GAIT_MODE_HELICAL_CURVE,
    GAIT_MODE_LATERAL_ROLLING,
    GAIT_MODE_STRAIGHT,
    GAIT_MODE_TEST,
    GAIT_MODE_NONE,
    GAIT_MODE_HELICAL_ROLLING2,
  };

  static void Initialize() {
    SnakeControlRequest::Initialize();
  }

  static double loop_rate(){ return loop_rate_; }

  //  static ros::Timer timer;
  static double loop_rate_;  // [Hz]
  static double sampling_time_;  // [s] サンプリングタイム loop_rateの逆

  static void CallBackOfJoySelectedData(joy_handler::JoySelectedData joy_data);

  static void OperateInitMode(joy_handler::JoySelectedData joy_data);
  static void OperateActivateMode(joy_handler::JoySelectedData joy_data);
  static void OperateMoveCrawler(joy_handler::JoySelectedData joy_data);
  static void OperateMoveHelicalRolling(joy_handler::JoySelectedData joy_data);
  static void OperateMoveFlange(joy_handler::JoySelectedData joy_data);

  static void OperateMoveLateralUndulation(joy_handler::JoySelectedData joy_data);
  static void OperateMoveSidewinding(joy_handler::JoySelectedData joy_data);
  static void OperateMoveLateralRolling(joy_handler::JoySelectedData joy_data);
  static void OperateMoveHelicalCurve(joy_handler::JoySelectedData joy_data);
  static void OperateMoveStraight(joy_handler::JoySelectedData joy_data);
  static void OperateMoveTest(joy_handler::JoySelectedData joy_data);
  static void OperateMoveHelicalRolling2(joy_handler::JoySelectedData joy_data);

  //--- Subscriber ---//
  //static ros::Subscriber sub_joy_selected_data_;

  static ControlMode control_mode_;
  static GaitMode gait_mode_;
  static HelicalRollingGait helical_rolling_gait_;
  static CrawlerGait crawler_gait_;
  static FlangeGait flange_gait_;

  static SidewindingGait sidewinding_gait_;
  static LateralRollingGait lateral_rolling_gait_;
  static HelicalCurveGait helical_curve_gait_;

 private:

};

#endif /* SNAKE_CONTROL_SRC_SNAKE_CONTROL_H_ */
