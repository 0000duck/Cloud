/**
 * @file flange_gait.h
 * @brief フランジ乗り越えのためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/08/13
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_FLANGE_GAIT_H_
#define SNAKE_CONTROL_SRC_FLANGE_GAIT_H_

#include <ros/ros.h>
#include "helical_rolling_gait.h"

class FlangeGait : public HelicalRollingGait {
 public:
  FlangeGait(RobotSpec spec,
             uint32_t num_ds_link_body,
             double min_diameter,
             double max_diameter,
             double min_pitch,
             double max_pitch,
             double max_height,
             double max_width,
             double radius_of_circle
            ) : HelicalRollingGait(spec, num_ds_link_body,
                min_diameter, max_diameter, min_pitch, max_pitch) {
    max_height_ = max_height;
    min_width_ = 2*radius_of_circle;
    max_width_ = max_width;
    radius_of_circle_ = radius_of_circle;
    gain_to_ideal_roll_per_shift_ = 1.0;

    gait_name_ = "Flange";
    InitializeShape();
  }

  void InitializeShape() {
    pitch_ = min_pitch_;
    set_diameter(min_diameter_);
    set_roll_angle(0);
    height_ = CalcHeightMin();
    width_ = min_width_;
    ReshapeSegmentParameter();
    UpdateSHead();
    SShiftRobotHeadToUnitHead();
  }

  void Stride(double ratio_to_max, double time_move);
  void SShiftHeadToEndOfBreagePart();
  void set_height(double height);
  void add_height(double height_add){ set_height(height_+height_add); }
  void set_width(double width);
  void add_width(double width_add){ set_width(width_+width_add); }

 private:
  //--- セグメントパラメータ計算用
  void ReshapeSegmentParameter();
  //--- 制限値計算用
  double CalcHeightMin();
  double CalcMaxShiftVelocity();
  double CalcMaxRollingVelocity();

  // 形状パラメータ
  double width_;  // ロボットの幅に相当
  double height_;  // ロボットの高さ
  double radius_of_circle_;  // 円弧部分の半径
  // 形状パラメータ制限用
  double max_height_;
  double min_width_;
  double max_width_;
  // 動作用パラメータ
  double gain_to_ideal_roll_per_shift_;  // シフト量に対する捻転量の理論値との差(滑りなどによって調整が必要)
};

#endif /* SNAKE_CONTROL_SRC_FLANGE_GAIT_H_ */
