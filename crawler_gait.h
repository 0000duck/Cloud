/**
 * @file crawler_gait.h
 * @brief crawler_gaitのためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/05/12
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_CRAWLER_GAIT_H_
#define SNAKE_CONTROL_SRC_CRAWLER_GAIT_H_

#include <ros/ros.h>
#include "simple_shape_connection.h"

class CrawlerGait : public SimpleShapeConnection {
 public:
  CrawlerGait(RobotSpec spec,
              uint32_t num_ds_link_body,
              double min_height,
              double max_height,
              double min_width,
              double max_width,
              double clearance_circle
             ) : SimpleShapeConnection(spec, num_ds_link_body) {

    min_height_ = min_height;
    max_height_ = max_height;
    min_width_ = min_width;
    max_width_ = max_width;
    clearance_circle_ = clearance_circle;

    gait_name_ = "Crawler";
    InitializeShape();
  }

  void InitializeShape() {
    height_ = min_height_;
    width_ = min_width_;
    while(CalcRadiusByHeightWidth(height_, width_) < spec_.min_radius_body()) {
      width_ += 0.01;
    }
    curvature_turn_ = 0.0;
    reverse_ratio_ = 0.0;
    ReshapeSegmentParameter();
    UpdateSHead();
    SShiftRobotHeadToUnitHead();
  }

  //--- 形状パラメータ変更
  void set_height(double height);
  void add_height(double height_add){ set_height(height_+height_add); }
  void set_width(double width);
  void add_width(double width_add){ set_width(width_+width_add); }
  void set_reverse_ratio(double ratio);
  void add_reverse_ratio(double ratio_add){ set_reverse_ratio(reverse_ratio_+ratio_add); }
  bool is_reversed(){ // どちらかと言えばひっくり返っている場合true
    if (reverse_ratio_ < 0.5) return false;
    else return true;
  }
  void set_curvature_turn_by_ratio(double ratio_to_max);
  void add_curvature_turn(double add_ratio){
    set_curvature_turn_by_ratio(ratio_to_max_curvature_turn_ + add_ratio);
  }
  void close_curvature_turn_to_zero(double diff_ratio);

  //--- 動作
  void Move(double ratio_to_max, double time_move);
  void Roll(double ratio_to_max, double time_move);

 private:
  // セグメントパラメータ計算用
  void ReshapeSegmentParameter();
  double CalcMaxCurvatureTurn();
  double CalcMaxShiftVelocity();
  double CalcMaxRollingVelocity();
  static double CalcRadiusByHeightWidth(double height, double width) {
    return std::sqrt(std::pow(height, 2) + std::pow(width/2.0, 2)) / 2.0;
  }
  static double CalcHeightByRadiusWidth(double radius, double width) {
    return std::sqrt(std::pow(radius*2.0, 2) - std::pow(width/2.0, 2));
  }
  static double CalcWidthByRadiusHeight(double radius, double height) {
    return std::sqrt(std::pow(radius*2.0, 2) - std::pow(height, 2)) * 2.0;
  }
  static double CalcAngleByHeightWidth(double height, double width) {
    return std::atan2(width/2.0, height)*2.0;
  }

  // 形状パラメータ
  double width_;  // ロボットの幅に相当
  double height_;  // ロボットの高さ
  double clearance_circle_;  // 円弧同士が最も接近する部分に開ける余裕
  double ratio_to_max_curvature_turn_;  // 旋回曲率の最大値との比
  double curvature_turn_;  // 旋回の曲率. 正の時左ターン
  double reverse_ratio_;  // [0.0, 1.0] ひっくり返す割合 1のとき完全にひっくり返っている
  // 形状パラメータ制限用
  double min_height_;
  double max_height_;
  double min_width_;
  double max_width_;
};

#endif /* SNAKE_CONTROL_SRC_CRAWLER_GAIT_H_ */
