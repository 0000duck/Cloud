/**
 * @file helical_rolling_gait.h
 * @brief 螺旋捻転推進のためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/08/13
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_HELICAL_ROLLING_GAIT_H_
#define SNAKE_CONTROL_SRC_HELICAL_ROLLING_GAIT_H_

#include <ros/ros.h>
#include "robot_spec.h"
#include "simple_shape_connection.h"

/** @fn
 * @brief 螺旋捻転用のクラス
 * @detail
 *  必ずしもその必要はないが，SimpleShapeConnectionの枠組みで作る
 *  一つの螺旋がずっとループする
 */
class HelicalRollingGait : public SimpleShapeConnection {
 public:
  HelicalRollingGait(RobotSpec spec,
                     uint32_t num_ds_link_body,
                     double min_diameter,
                     double max_diameter,
                     double min_pitch,
                     double max_pitch
                    ) : SimpleShapeConnection(spec, num_ds_link_body) {

    min_diameter_ = min_diameter;
    max_diameter_ = max_diameter;
    min_pitch_ = min_pitch;
    max_pitch_ = max_pitch;
    gait_name_ = "HelicalRolling";
    InitializeShape();
  }

  virtual void InitializeShape() {
    pitch_ = min_pitch_;
    set_diameter(min_diameter_);
    set_roll_angle(0.0);
    ReshapeSegmentParameter();
    SShiftRobotHeadToUnitHead();
  }
  //--- 形状パラメータ変更
  void set_diameter(double diameter);
  void add_diameter(double diameter_add){ set_diameter(diameter_+diameter_add); }
  void set_pitch(double pitch);
  void add_pitch(double pitch_add){ set_pitch(pitch_+pitch_add); }
  //--- 動作
  void Roll(double ratio_to_max, double time_move);
  void SShiftByRatio(double ratio_to_max, double time_move);

 protected:
  // 制限値計算
  double CalcMaxShiftVelocity();
  double CalcMaxRollingVelocity();
  void ReshapeSegmentParameter();
  // セグメントパラメータ計算用
  static double CalcCurvatureHelix(double diameter, double pitch);
  static double CalcTorsionHelix(double diameter, double pitch);

 protected:
  // 形状パラメータ
  double diameter_;  // 螺旋の直径
  double pitch_;  // 螺旋のピッチ
  // 形状パラメータ制限用
  double min_diameter_;
  double max_diameter_;
  double min_pitch_;
  double max_pitch_;
};

#endif /* SNAKE_CONTROL_SRC_HELICAL_ROLLING_GAIT_H_ */
