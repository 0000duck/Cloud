/**
 * @file crawler_gait.cpp
 * @brief crawler_gaitのためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/05/12
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include <algorithm>
#include "crawler_gait.h"

const double MIN_CURVATURE_TURN = 1.0/10000000.0;  // [m] これより旋回の曲率が小さければ直進とみなす

/** @fn
 * @brief ロボットの形状の高さを設定する
 * @param double height [m] ロボットの高さ
 * @return なし
 * @detail
 *  関節可動域などを満たす範囲に収める
 */
void CrawlerGait::set_height(double height){
  // 現在の幅から，関節可動域を満たす高さか確認
  if (CalcRadiusByHeightWidth(height, width_) < spec_.min_radius_body()) {
    height = CalcHeightByRadiusWidth(spec_.min_radius_body(), width_);
  }
  // 代入
  if(height < min_height_) height_ = min_height_;
  else if(height > max_height_) height_ = max_height_;
  else height_ = height;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] height change to %4.3f [m]", gait_name_.c_str(), height_);
}

/** @fn
 * @brief ロボットの形状の幅を設定する
 * @param double width [m] ロボットの幅
 * @return なし
 * @detail
 *  関節可動域などを満たす範囲に収める
 */
void CrawlerGait::set_width(double width){
  // 現在の高さから，関節可動域を満たす高さか確認
  if (CalcRadiusByHeightWidth(height_, width) < spec_.min_radius_body()) {
    width = CalcWidthByRadiusHeight(spec_.min_radius_body(), height_);
  }
  // 代入
  if(width < min_width_) width_ = min_width_;
  else if(width > max_width_) width_ = max_width_;
  else width_ = width;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] width change to %4.3f [m]", gait_name_.c_str(), width_);
}

/** @fn
 * @brief ロボットの復帰動作用の「ひっくり返し具合」を変更する
 * @param double ratio 0.0-1.0 ひっくり返し具合
 * @return なし
 * @detail
 */
void CrawlerGait::set_reverse_ratio(double ratio){
  if(ratio < 0.0) reverse_ratio_ = 0.0;
  else if(ratio > 1.0) reverse_ratio_ = 1.0;
  else reverse_ratio_ = ratio;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] reverse_ratio change to %3.2f", gait_name_.c_str(), reverse_ratio_);
}

/** @fn
 * @brief ロボットの旋回曲率を最大値に対する割合で設定
 * @param double ratio_to_max [-1.0, 1.0] 最大曲率に対する割合
 * @return なし
 * @detail
 */
void CrawlerGait::set_curvature_turn_by_ratio(double ratio_to_max){
  if(ratio_to_max < -1.0) ratio_to_max = -1.0;
  else if(ratio_to_max > 1.0) ratio_to_max = 1.0;
  ratio_to_max_curvature_turn_ = ratio_to_max;
  curvature_turn_ = ratio_to_max*CalcMaxCurvatureTurn();
  ReshapeSegmentParameter();
}

/** @fn
 * @brief ロボットの旋回曲率を0に近づけるように変化させる
 * @param double diff_to_max [0.0, 1.0] 変化させる幅 最大曲率に対する割合
 * @return なし
 * @detail
 */
void CrawlerGait::close_curvature_turn_to_zero(double diff_ratio){
  if (ratio_to_max_curvature_turn_ > diff_ratio) add_curvature_turn(-diff_ratio);
  else if (ratio_to_max_curvature_turn_ < -diff_ratio) add_curvature_turn(diff_ratio);
}

/** @fn
 * @brief ロボットを前進(後退)させるようにパラメータを変更
 * @param double ratio_to_max [-1.0, 1.0] 最大速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void CrawlerGait::Move(double ratio_to_max, double time_move) {
  SShift(ratio_to_max*CalcMaxShiftVelocity()*time_move);
}

/** @fn
 * @brief ロボットを捻転させるようにパラメータを変更
 * @param double ratio_to_max [-1.0, 1.0] 最大捻転速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void CrawlerGait::Roll(double ratio_to_max, double time_move) {
  double angle_add = ratio_to_max*CalcMaxRollingVelocity()*time_move;
  add_roll_angle(angle_add);
}

/** @fn
 * @brief 各セグメントのパラメータを計算
 * @param なし
 * @return なし
 * @detail
 */
void CrawlerGait::ReshapeSegmentParameter() {
  // 計算用変数の作成
  double h = height_;
  double w = width_;
  double r = CalcRadiusByHeightWidth(h, w);
  double a = CalcAngleByHeightWidth(h, w);
  a += reverse_ratio_*(2.0*M_PI-2.0*a);
  double c_c = clearance_circle_;

  // 直進のとき
  if (std::abs(curvature_turn_) < MIN_CURVATURE_TURN) {
    if(curvature_turn_ < 0.0) curvature_turn_ = -MIN_CURVATURE_TURN;
    else curvature_turn_ = MIN_CURVATURE_TURN;
  }
  // 旋回曲率を制限
  double max_curvature_turn = CalcMaxCurvatureTurn();
  if (curvature_turn_ > max_curvature_turn)curvature_turn_ = max_curvature_turn;
  else if (curvature_turn_ < -max_curvature_turn)curvature_turn_ = -max_curvature_turn;

  // 計算用変数の作成
  double r_t = 1.0/curvature_turn_;
  double abs_r_t = std::abs(r_t);
  double sgn_r = 0.0;
  if (r_t >= 0.0) sgn_r = 1.0;
  else if (r_t < 0.0) sgn_r = -1.0;

  // 計算結果用変数
  double r_t_arc[2] = {0.0};
  double phi_t_arc[2] = {0.0};
  double torsion_angle_t_arc[2] = {0.0};
  // 計算
  for (uint8_t i = 0; i<2; i++) {
    double l = abs_r_t + std::pow(-1.0, i)*sgn_r*w/4.0;
    double beta = std::atan2(r, l);
    double gamma = std::asin(c_c/2.0 / std::sqrt(l*l+r*r));
    r_t_arc[i] = abs_r_t + std::pow(-1.0, i+1)*sgn_r*(w/2.0);
    phi_t_arc[i] = 2.0*(beta + gamma);  // rhoを入れるとうまくいかない
    torsion_angle_t_arc[i] = std::pow(-1.0, i) * (M_PI_2 - a/2.0);
    if (sgn_r > 0) torsion_angle_t_arc[i] += (i+1)*M_PI;
    else torsion_angle_t_arc[i] += i*M_PI;
  }
  // Reshape
  segment_.resize(6);
  segment_[0].ReshapeAsCircularArc(r_t_arc[0], phi_t_arc[0], torsion_angle_t_arc[0]);
  segment_[1].ReshapeAsCircularArc(r, M_PI, -torsion_angle_t_arc[0]);
  segment_[2].ReshapeAsCircularArc(r, M_PI, a);
  segment_[3].ReshapeAsCircularArc(r_t_arc[1], phi_t_arc[1], torsion_angle_t_arc[1]);
  segment_[4].ReshapeAsCircularArc(r, M_PI, -torsion_angle_t_arc[1]);
  segment_[5].ReshapeAsCircularArc(r, M_PI, -a);
  segment_parameter_was_changed_ = true;
}

/** @fn
 * @brief 形状パラメータとSpecをもとに最大の旋回曲率を算出する
 * @param なし
 * @return double最大の旋回曲率
 * @detail
 */
double CrawlerGait::CalcMaxCurvatureTurn() {
  double min_radius_turn = spec_.min_radius_body() + width_/2.0;
  return 1.0/min_radius_turn;
}

/** @fn
 * @brief 形状パラメータとSpecをもとに最大のs-shift速さを算出する
 * @param なし
 * @return double [m/sec] 最大のs-shift速さ
 * @detail
 */
double CrawlerGait::CalcMaxShiftVelocity() {
  double angle = CalcAngleByHeightWidth(height_, width_);
  double radius = CalcRadiusByHeightWidth(height_, width_);
  return spec_.max_joint_angle_velocity()*radius/( std::max(1.0, 2*sin(angle/2)) );
}

/** @fn
 * @brief 形状パラメータとSpecをもとに最大の捻転角速度を算出する
 * @param なし
 * @return double [rad/sec]最大の捻転角速度
 * @detail
 */
double CrawlerGait::CalcMaxRollingVelocity() {
  return spec_.max_joint_angle_velocity() /
      (2.0*spec_.link_length_body()/CalcRadiusByHeightWidth(height_, width_));
}

