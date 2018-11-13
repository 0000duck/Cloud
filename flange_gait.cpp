/**
 * @file flange_gait.cpp
 * @brief フランジ乗り越えのためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/08/13
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "flange_gait.h"

/** @fn
 * @brief フランジを乗り越える動作．s-shiftに合わせて捻転を行う
 * @param double ratio_to_max [-1.0, 1.0] 最大速さに対する割合 正のとき，頭側に乗り越え
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 *  理論的なシフト量と捻転量の比に加えて，ゲインをかけて調整する
 */
void FlangeGait::Stride(double ratio_to_max, double time_move) {
  double r_h = diameter_/2;
  double b_h = pitch_/(2.0*M_PI);
  double a = std::atan(b_h/r_h);
  double d_shift = -ratio_to_max*CalcMaxShiftVelocity()*time_move;
  double ratio_roll_per_shift = -2.0*std::tan(a)/spec_.link_diameter();
  SShift(d_shift);
  add_roll_angle(d_shift*ratio_roll_per_shift*gain_to_ideal_roll_per_shift_);
}

/** @fn
 * @brief 先頭位置を橋部の終わりにシフトさせる．初期化時にのみ使う
 * @paran なし
 * @return なし
 * @detail
 *  橋部をちょうど跨ぎ終わったところに先頭がくるようにする
 */
void FlangeGait::SShiftHeadToEndOfBreagePart() {
  SShiftRobotHeadToUnitHead();
  SShift( -(unit_length()-spec_.full_length()) );  // 橋部の長さ分シフトする．螺旋の長さ=full_length
  UpdateSHead();
}

/** @fn
 * @brief 橋部の高さを設定
 * @param double height [m] 希望する高さ
 * @return なし
 * @detail
 *  他の形状パラメータから決まる制限範囲に収まるようにする
 */
void FlangeGait::set_height(double height){
  double min_height = CalcHeightMin();
  if(height < min_height) height_ = min_height;
  else if(height > max_height_) height_ = max_height_;
  else height_ = height;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] height change to %4.3f [m]", gait_name_.c_str(), height_);
}

/** @fn
 * @brief 橋部の幅を設定
 * @param double width [m] 希望する幅
 * @return なし
 * @detail
 */
void FlangeGait::set_width(double width){
  if(width < min_width_) width_ = min_width_;
  else if(width > max_width_) width_ = max_width_;
  else width_ = width;
  ReshapeSegmentParameter();
  ROS_INFO("[%s] width change to %4.3f [m]", gait_name_.c_str(), width_);
}

/** @fn
 * @brief 各セグメントのパラメータを計算
 * @param なし
 * @return なし
 * @detail
 */
void FlangeGait::ReshapeSegmentParameter() {
  // 計算用変数の作成
  double r_h = diameter_/2;
  double p_h = pitch_;
  double b_h = p_h/(2.0*M_PI);
  double h = height_;
  double w = width_;
  double r_c = radius_of_circle_;
  double a = std::atan(b_h/r_h);

  double rcsina = r_c * sin(a);

  double h_b = std::sqrt(std::pow(rcsina,2) + 2*r_h*r_c + std::pow(r_h,2)) - r_h + r_c;
  double h_min = CalcHeightMin();
  if (h < h_min) h = h_min;

  double beta = 0;
  double l_s = 0;
  double gamma = 0;
  if (h_b < h) {
    beta = std::acos( r_c/std::sqrt(std::pow(r_h+r_c,2)+std::pow(rcsina,2)) ) - std::atan(rcsina/(r_h+r_c));
    l_s = h - std::sqrt(std::pow(rcsina,2) + 2*r_h*r_c + std::pow(r_h,2)) + r_h - r_c;
    gamma = 0;
  } else {
    double beta_pre = std::acos( ( std::pow(r_c+r_h,2)+std::pow(rcsina,2)+2*std::pow(r_c,2)-std::pow(r_h+h,2) )
        / ( 2*std::sqrt(std::pow(r_c+r_h,2)+std::pow(rcsina,2))*std::sqrt(2)*r_c ) );
    beta = beta_pre - std::atan(rcsina/(r_h+r_c)) - M_PI/4;
    l_s = 0;
    gamma = M_PI
        + 2*( beta + std::atan( (rcsina+std::sqrt(2)*r_c*std::sin(beta+M_PI/4))/(r_h+r_c-std::sqrt(2)*r_c*std::cos(beta+M_PI/4)) ) );
  }

  double angle_helix_full_body = spec_.full_length()/std::sqrt(std::pow(r_h,2)+std::pow(b_h,2));
  // Reshape
  segment_.resize(10);
  segment_[0].ReshapeAsHelix(r_h, p_h, angle_helix_full_body, -M_PI/2);
  segment_[1].ReshapeAsCircularArc(r_c, a, -M_PI/2);
  segment_[2].ReshapeAsCircularArc(r_c, beta, -M_PI/2);
  segment_[3].ReshapeAsStraightLine(l_s);
  segment_[4].ReshapeAsCircularArc(r_c, M_PI/2, -M_PI/2);
  segment_[5].ReshapeAsStraightLine(w-2*r_c);
  segment_[6].ReshapeAsCircularArc(r_c, M_PI/2, gamma);
  segment_[7].ReshapeAsStraightLine(l_s);
  segment_[8].ReshapeAsCircularArc(r_c, beta, -M_PI/2);
  segment_[9].ReshapeAsCircularArc(r_c, a, -M_PI/2);
  segment_parameter_was_changed_ = true;
}

/** @fn
 * @brief 形状パラメータから橋部の高さの最小値を求める
 * @param なし
 * @return double [m] 橋部の高さの最小値
 * @detail
 */
double FlangeGait::CalcHeightMin() {
  double r_h = diameter_/2;
  double p_h = pitch_;
  double b_h = p_h/(2.0*M_PI);
  double r_c = radius_of_circle_;
  double a = std::atan(b_h/r_h);
  return std::sqrt(std::pow(r_h,2) + std::pow(r_c*(1+std::sin(a)),2)) - r_h + 0.02; // 最後の項は計算誤差対策
}

/** @fn
 * @brief 形状パラメータとSpecをもとに最大のs-shift速さを算出する
 * @param なし
 * @return double [m/sec] 最大のs-shift速さ
 * @detail
 *  螺旋と円弧にまたがるときが最悪ケースとして，ざっくり求める
 */
double FlangeGait::CalcMaxShiftVelocity() {
  double r_h = diameter_/2;
  double b_h = pitch_/(2.0*M_PI);
  double curvature = CalcCurvatureHelix(diameter_, pitch_);
  double a = std::atan(b_h/r_h);
  double r_c = radius_of_circle_;
  return spec_.max_joint_angle_velocity()/( (curvature+1.0/r_c)*std::cos(a/2.0) );  // ざっくり
}

/** @fn
 * @brief 形状パラメータとSpecをもとに最大の捻転角速度を算出する
 * @param なし
 * @return double [rad/sec]最大の捻転角速度
 * @detail
 */
double FlangeGait::CalcMaxRollingVelocity() {
  double r_h = diameter_/2;
  double b_h = pitch_/(2.0*M_PI);
  double curvature = CalcCurvatureHelix(diameter_, pitch_);
  return spec_.max_joint_angle_velocity() /
      ( 2.0*spec_.link_length_body()/std::min(1.0/curvature, radius_of_circle_) );
}
