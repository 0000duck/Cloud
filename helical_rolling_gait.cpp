/**
 * @file helical_rolling_gait.cpp
 * @brief 螺旋捻転推進のためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/08/13
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "helical_rolling_gait.h"

/** @fn
 * @brief 螺旋の直径を変更する
 * @param double diameter [m] 螺旋の直径
 * @detail
 *  螺旋ピッチと制限値から求まる最小の直径を考慮して設定される
 */
void HelicalRollingGait::set_diameter(double diameter) {
  // 現在のpitchから，関節可動域を満たすdiameterか確認
  if (CalcCurvatureHelix(diameter, pitch_) > spec_.max_curvature_body()) {
    // 不適のとき，なるべく小さいdiameterに設定
    double k = spec_.max_curvature_body();
    double b = pitch_/(2*M_PI);
    diameter = (1+std::sqrt(1-4*k*k*b*b))/k;
  }
  // 代入
  if(diameter < min_diameter_) diameter_ = min_diameter_;
  else if(diameter > max_diameter_) diameter_ = max_diameter_;
  else diameter_ = diameter;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] diameter change to %4.3f [m]", gait_name_.c_str(), diameter_);
}

/** @fn
 * @brief 螺旋のピッチを変更する
 * @param double pitch [m] 螺旋のピッチ
 * @detail
 *  螺旋半径と制限値から求まる最小のピッチを考慮して設定される
 */
void HelicalRollingGait::set_pitch(double pitch) {
  // 現在のdiameterから，関節可動域を満たすpitchか確認
  if (CalcCurvatureHelix(diameter_, pitch) > spec_.max_curvature_body()) {
    // 不適のとき，なるべく小さいpitchに設定
    double k = spec_.max_curvature_body();
    double a = diameter_/2;
    pitch = 2.0*M_PI*std::sqrt(a/k-a*a);
  }
  // 代入
  if(pitch < min_pitch_) pitch_ = min_pitch_;
  else if(pitch > max_pitch_) pitch_ = max_pitch_;
  else pitch_ = pitch;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] pitch change to %4.3f [m]", gait_name_.c_str(), pitch_);
}

/** @fn
 * @brief ロボットを捻転させるようにパラメータを変更
 * @param double ratio_to_max [-1.0, 1.0] 最大捻転速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void HelicalRollingGait::Roll(double ratio_to_max, double time_move) {
  double angle_add = ratio_to_max*CalcMaxRollingVelocity()*time_move;
  add_roll_angle(angle_add);
}

/** @fn
 * @brief SShiftを行う．動作としては捻転するだけ
 * @param double ratio_to_max [-1.0, 1.0] 最大速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void HelicalRollingGait::SShiftByRatio(double ratio_to_max, double time_move) {
  SShift(ratio_to_max*CalcMaxShiftVelocity()*time_move);
}

/** @fn
 * @brief 最大のシフト速度を求める
 * @param なし
 * @return なし
 * @detail
 *  ものすごく適当に決める．
 *  動作としては捻転するだけ．
 *  歩容切替時に使うかもしれないので定義
 */
double HelicalRollingGait::CalcMaxShiftVelocity() {
  return 0.20;
}

/** @fn
 * @brief 最大の捻転速度を求める
 * @param なし
 * @return なし
 * @detail
 */
double HelicalRollingGait::CalcMaxRollingVelocity() {
  return spec_.max_joint_angle_velocity() /
      (2*spec_.link_length_body()*CalcCurvatureHelix(diameter_, pitch_));
}

/** @fn
 * @brief 形状パラメータを元にセグメントパラメータを更新する
 * @param なし
 * @return なし
 * @detail
 *  ただの螺旋
 */
void HelicalRollingGait::ReshapeSegmentParameter() {
  segment_.resize(1);
  segment_[0].ReshapeAsHelix(diameter_/2.0, pitch_, 2.0*M_PI, 0.0);
  segment_parameter_was_changed_ = true;
}

/** @fn
 * @brief 螺旋の曲率を計算する
 * @param double diameter [m] 螺旋の直径
 * @param double pitch [m] 螺旋のピッチ
 * @return double [1/m] 螺旋の曲率
 * @detail
 */
double HelicalRollingGait::CalcCurvatureHelix(double diameter, double pitch) {
  double a = diameter/2.0;
  double b = pitch/(2*M_PI);
  return a/(a*a + b*b);
}

/** @fn
 * @brief 螺旋の捻率を計算する
 * @param double diameter [m] 螺旋の直径
 * @param double pitch [m] 螺旋のピッチ
 * @return double 螺旋の捻率
 * @detail
 */
double HelicalRollingGait::CalcTorsionHelix(double diameter, double pitch) {
  double a = diameter/2.0;
  double b = pitch/(2*M_PI);
  return b/(a*a + b*b);
}
