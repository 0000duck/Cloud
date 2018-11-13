/**
 * @file simple_shape_connection.h
 * @brief 単純形状を連結して歩容設計するクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/05/12
 * @detail
 *  - segment : 目標形状を構成する各単純形状のこと
 *  - unit    : 連結した一連の単純形状は同じ構成を繰り返す．その１単位のことをUnitという
 */

#include "simple_shape_connection.h"

//=== Segment class =========================================================//

/** @fn
 * @brief 螺旋の傾き角を求める
 * @param double radius [m] 螺旋半径
 * @param double pitch [m] 螺旋ピッチ
 * @return double [rad] 傾き角
 */
double Segment::CalcElevationOfHelixFromRadiusAndPitch(double radius, double pitch) {
  return std::atan2(pitch/(2*M_PI), radius);
}

/** @fn
 * @brief 螺旋の傾き角を求める
 * @param double radius [m] 螺旋半径
 * @param double slope [m/rad] 螺旋の傾き(z=btのb)
 * @return double [rad] 傾き角
 */
double Segment::CalcElevationOfHelixFromRadiusAndSlope(double radius, double slope) {
  return std::atan2(slope, radius);
}

/** @fn
 * @brief 螺旋ピッチから螺旋の傾きを求める
 * @param double pitch [m] 螺旋ピッチ
 * @return double [m/rad] 螺旋の傾き(z=btのb)
 */
double Segment::Pitch2SlopeHelix(double pitch) {
  return pitch/(2.0*M_PI);
}

/** @fn
 * @brief 螺旋の傾きから螺旋ピッチを求める
 * @param double slope [m/rad] 螺旋の傾き(z=btのb)
 * @return double [m] 螺旋ピッチ
 */
double Segment::Slope2PitchHelix(double slope) {
  return slope*(2.0*M_PI);
}

/** @fn
 * @brief 螺旋の曲率を求める
 * @param double radius [m] 螺旋半径
 * @param double pitch [m] 螺旋ピッチ
 * @return double 曲率
 */
double Segment::CalcCurvatureOfHelixFromRadiusAndPitch(double radius, double pitch) {
  double b = pitch/(2.0*M_PI);
  return radius/(radius*radius + b*b);
}

//=== SimpleShapeConnection class ===========================================//

/** @fn
 * @brief セグメント連結部の位置と，ユニットの各位置(弧長s)における曲率，捻れ角を更新
 * @param なし
 * @return なし
 * @detail
 *  事前にセグメント形状が決定知る必要がある
 *  更新されるのは以下の値
 *   - segment_.s_edge_tailside_
 *   - unit_length_
 *   - curvature_  セグメントの曲率そのまま
 *   - torsion_angle_  セグメントの捻率をs(弧長)で積分
 *   - tarsion_angle_per_unit_
 *   加えて，形状変化のフラグも折られる
 *   segment_parameter_was_changed_ <-- false
 */
void SimpleShapeConnection::UpdateUnitParameter() {
  // 各連結部の弧長パラメータにおける位置と，unit全体の長さを計算
  segment_[0].set_s_edge_tailside(segment_[0].length());
  for(Segment::ItrType itr_seg = segment_.begin()+1; itr_seg != segment_.end(); ++itr_seg) {
    itr_seg->set_s_edge_tailside( (itr_seg-1)->s_edge_tailside() + itr_seg->length() );
  }
  unit_length_ = segment_.back().s_edge_tailside();

  //--- 捻率を積分して各点における捻れ角を計算
  //--- あとで計算しやすいように曲率も取り出しておく
  // 初期化
  double s = 0;
  curvature_.clear();
  torsion_angle_.clear();
  num_ds_unit_ = 0;
  double torsion_angle_pre = 0.0;  // 一つ前の捻れ角
  // segmentでループ
  for(Segment::ItrType itr_seg = segment_.begin(); itr_seg != segment_.end(); ++itr_seg) {
    // segment連結部での捻れを最初だけ足す．足したらこの値は0にする．
    double torsion_angle_headside = itr_seg->torsion_angle_headside();
    // 積分を進める
    while(s < itr_seg->s_edge_tailside()) {  // sがセグメント末尾を超えるまでループ
      curvature_.push_back( itr_seg->curvature() );
      torsion_angle_.push_back( torsion_angle_headside + torsion_angle_pre + itr_seg->torsion()*ds_ );
      torsion_angle_headside = 0;  // セグメントの初めに足したらあとは0
      s += ds_;
      num_ds_unit_++;
      torsion_angle_pre = torsion_angle_.back();
    }
  }  // for segment
  torsion_angle_per_unit_ = torsion_angle_.back();
  segment_parameter_was_changed_ = false;
}

/** @fn
 * @brief 目標関節角を計算する
 * @param なし
 * @return std::vector<double> joint_angle(spec_.num_joint())
 * @detail
 */
std::vector<double> SimpleShapeConnection::CalcJointAngle() {
  std::vector<double> joint_angle(spec_.num_joint(), 0.0);  // 値0.0で初期化

  // 形状が変わった時だけUnitパラメータを更新
  if (segment_parameter_was_changed_) {
    UpdateUnitParameter();
  }

  UpdateSHead();
  int32_t i_ds_head = std::floor(s_head_/ds_);
  int32_t i_offset1 = 0;  // ロボットの先頭位置から積分範囲の先頭までのoffset

  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {  // 関節ごとにループ

    // 積分範囲内でループ i_offset2は積分範囲先頭からのoffset
    for (int32_t i_offset2=0; i_offset2<num_ds_link_[i_joint]+num_ds_link_[i_joint+1]; i_offset2++) {
      // インデックスを計算
      int32_t sum_of_i_offset = i_ds_head + i_offset1 + i_offset2;
      int32_t i_unit = sum_of_i_offset / num_ds_unit_;  // 何番目のユニットにいるか
      int32_t i_ds = sum_of_i_offset % num_ds_unit_;  // ユニット内のどこにいるか
      if ( i_ds < 0) {  // c++の仕様上，余りが負になることがあるので対策
        i_ds += num_ds_unit_;
        i_unit--;  // 思ってたより一つ後のunitにいた事になるので一つ戻す
      }
      double sum_of_torsion_angle = roll_angle_ + i_unit*torsion_angle_per_unit_ + torsion_angle_[i_ds];
      // 積分を実行 関節の向きにより-sinかcosか
      if ( (i_joint+1)%2 == (uint8_t)spec_.odd_joint_is_yaw() ) {
        joint_angle[i_joint] += ds_ * curvature_[i_ds] * (-std::sin(sum_of_torsion_angle));
      } else {
        joint_angle[i_joint] += ds_ * curvature_[i_ds] * std::cos(sum_of_torsion_angle);
      }
    }
    // 次の関節に進む
    i_offset1 += num_ds_link_[i_joint];
  }

  return joint_angle;
}

/** @fn
 * @brief ロボットの先頭が目標曲線上のどこにいるかを更新する
 * @param なし
 * @return
 * @detail
 *  形状変更時に急激に体形が変化しないように，
 *  S-Shiftの値はロボットの中心がUnitの何%の位置にいるかで管理している．
 *  基本はこの%の値で値を更新しているため，現在の形状の値を用いて先頭の弧長sの値を更新する
 */
void SimpleShapeConnection::UpdateSHead() {
  // 形状を更新
  if (segment_parameter_was_changed_) {
    UpdateUnitParameter();
  }
  s_robot_center_ratio_ += s_add_buff_/unit_length_;
  s_add_buff_ = 0;
  s_head_ = s_robot_center_ratio_*unit_length_ - 0.5*spec_.full_length();
}
