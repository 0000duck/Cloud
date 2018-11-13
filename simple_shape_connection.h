/**
 * @file simple_shape_connection.h
 * @brief 単純形状を連結して歩容設計するクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/05/12
 * @detail
 *  単純形状はループする
 */

#ifndef SNAKE_CONTROL_SRC_SIMPLE_SHAPE_CONNECTION_H_
#define SNAKE_CONTROL_SRC_SIMPLE_SHAPE_CONNECTION_H_

#include <cmath>
#include <stdint.h>
#include <vector>
#include <string>
#include "robot_spec.h"

/** @class
 * @brief セグメント(SimpleShape)のクラス
 * @detail
 *  形状のパラメータから計算した曲率と捻率を保持する
 */
class Segment {
 public:
  typedef std::vector<Segment>::iterator ItrType;

 public:
  Segment(){
    torsion_angle_headside_ = 0;
    curvature_ = 0;
    torsion_ = 0;
    length_ = 0;
    s_edge_tailside_ = 0;
  }

  ~Segment(){

  }

  Segment(const Segment& obj) {  // コピーコンストラクタ
    this->torsion_angle_headside_ = obj.torsion_angle_headside_;
    this->curvature_ = obj.curvature_;
    this->torsion_ = obj.torsion_;
    this->length_ = obj.length_;
    this->s_edge_tailside_ = obj.s_edge_tailside_;
  }

  void ReshapeAsStraightLine(double length) {
    length_ = length;
    curvature_ = 0;
    torsion_ = 0;
    torsion_angle_headside_ = 0;
  }
  void ReshapeAsCircularArc(double radius, double central_angle, double torsion_angle_head){
    length_ = radius* central_angle;
    curvature_ = 1.0/radius;
    torsion_ = 0;
    torsion_angle_headside_ = torsion_angle_head;
  }
  void ReshapeAsHelix(double radius, double pitch, double central_angle, double torsion_angle_head) {
    double a = radius;
    double b = pitch/(2.0*M_PI);
    double sum_of_squares = a*a + b*b;
    length_ = central_angle * std::sqrt(sum_of_squares);
    curvature_ = a/sum_of_squares;
    torsion_ = b/sum_of_squares;
    torsion_angle_headside_ = torsion_angle_head;
  }

  //--- setter --------------------//
  void set_s_edge_tailside(double s_edge_tailside){ s_edge_tailside_ = s_edge_tailside; }

  //--- getter --------------------//
  double torsion_angle_headside(){ return torsion_angle_headside_; }
  double curvature(){ return curvature_; }
  double torsion(){ return torsion_; }
  double length(){ return length_; }
  double s_edge_tailside(){ return s_edge_tailside_; }

  //--- static 関数 -------------------//
 public:
  static double CalcElevationOfHelixFromRadiusAndPitch(double radius, double pitch);
  static double CalcElevationOfHelixFromRadiusAndSlope(double radius, double slope);
  static double Pitch2SlopeHelix(double pitch);
  static double Slope2PitchHelix(double slope);
  static double CalcCurvatureOfHelixFromRadiusAndPitch(double radius, double pitch);

 private:
  double torsion_angle_headside_;  // 頭側のセグメント連結部における捻れ角 直線では0
  double curvature_;  // [1/m] 曲率
  double torsion_;  // 捻率
  double length_;  // [m] セグメント長さ
  double s_edge_tailside_;  // しっぽ側のセグメント連結部における弧長パラメータsの値
};

/** @class
 * @brief 単純形状の連結で目標形状を表現し，動かすためのクラス
 * @detail
 *  歩容のクラスでこれを継承すればよい
 *  "Unit"というのは一連のSegmentのつながりを指す．これが無限ループした形状が目標形状となる．
 */
class SimpleShapeConnection {
 public:
  SimpleShapeConnection(RobotSpec spec, uint32_t num_ds_link_body) {
    spec_ = spec;
    ds_ = spec_.link_length_body() / num_ds_link_body;
    num_ds_link_.clear();
    num_ds_full_robot_ = 0.0;
    for (uint8_t i_link=0; i_link<spec.num_joint()+1; i_link++) {
      num_ds_link_.push_back(std::ceil(spec.link_length().at(i_link)/ds_));  // 気持ち長めに取るのでceil
      num_ds_full_robot_ += num_ds_link_.back();
    }

    segment_parameter_was_changed_ = true;
    s_robot_center_ratio_ = 0.0;
    roll_angle_ = 0;
    s_head_ = 0.0;
    unit_length_ = 1.0;  // 0でなくしておく
  }

  std::vector<double> CalcJointAngle();

  double s_head(){
    UpdateSHead();
    return s_head_;
  };

 protected:
  void SShift(double s_add){ s_add_buff_ += s_add; }
  void SShiftRobotHeadToUnitHead() {// robotの先頭がUnitの先頭になるようにS-Shiftする
    if (segment_parameter_was_changed_) { UpdateUnitParameter(); }
    s_robot_center_ratio_ = 0.5*spec_.full_length()/unit_length_;
  }
  void set_roll_angle(double angle){ roll_angle_ = angle; }
  void add_roll_angle(double angle_add){ roll_angle_ += angle_add; }
  void UpdateUnitParameter();
  void UpdateSHead();
  RobotSpec spec_;
  std::vector<Segment> segment_;  // セグメントの配列 この配列にしたがって全体の形状が出来上がる．形状はループする
  // 形状を変えた時に立てる その時だけClacJointAngle()内でUpdateUnitParameter()を実行
  bool segment_parameter_was_changed_;
  double unit_length(){ return unit_length_; }
  // インターフェース
  std::string gait_name_;  // print用に歩容の名前を設定

 private:
  double roll_angle_;  // [rad] これを捻ることで捻転．s=0における捻れ角に相当
  double s_head_; // [m] 先頭の弧長パラメータにおける値
  // ユニットのどこにロボットの中心があるかを基準にすることで形状変化時に大きく関節角が変わらないようにする
  double s_robot_center_ratio_;  // ロボットの中心位置がユニットのどの場所にあるか．ユニットの全長を1とする
  double s_add_buff_;  // [m] sを動かす長さ．ユニットのパラメータを更新してから加算する必要があるので，一旦この変数にキープする
  // 目標形状に関する変数
  double unit_length_;  // [m] 連結した形状１ループあたりの長さ

  // 目標形状の計算用
  double ds_;  // [m] 長さパラメータの刻み幅
  std::vector<double> curvature_;  // [rad/m] 各点における曲率
  std::vector<double> torsion_angle_;  // [rad] 各点における捻れ角の値(捻率の積分値)．roll_angle_分は含まない
  double torsion_angle_per_unit_;  // [rad] 単純形状のユニットを1周したときの捻れ角の積分値の変化量
  // 特に関節角の計算用
  std::vector<int32_t> num_ds_link_;  // 各リンクが何区分されてるか(積分するときの分割数)
  int32_t num_ds_full_robot_;  // ロボット全体で何区分か(積分するときの分割数)
  int32_t num_ds_unit_;  // 1unitあたりに何区分分あるか(積分するときの分割数)

};

#endif /* SNAKE_CONTROL_SRC_SIMPLE_SHAPE_CONNECTION_H_ */
