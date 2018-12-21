/**
 * @file pedal_wave_motion.h
 * @brief 横うねり運動のためのクラス
 * @author Taichi AKIYAMA
 * @date 2018/10/02
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_PEDAL_WAVE_H_
#define SNAKE_CONTROL_SRC_PEDAL_WAVE_H_

#include <cmath>
#include <stdint.h>
#include <vector>
#include "robot_spec.h"
#include <joy_handler/JoySelectedData.h>

#define NUM_YAW_STEERING 2

class PedalWaveMotion2{
public:

  PedalWaveMotion2(RobotSpec spec);
  virtual ~PedalWaveMotion2();

  std::vector<double> CalcJointAngle();
  void ChangeParam(joy_handler::JoySelectedData joy_data, double sampling_time);
  void PrintParameters();

protected:

  RobotSpec spec_;

private:

  double CalcKappaPW(double s, double t);
  double CalcKappaYNoSteering(double s);
  double CalcKappaSY(int i, double s);
  double CalcKappaSP(double s);

  void AddT(double dt);
  void AddBeta(int i, double db);

  void StartSteering(int i);
  void FinishSteering(int i);

  void AddBetaP(double db);
  void StartSteeringP();
  void FinishSteeringP();


  double l_p_; //縦波の波長[m]
  double alpha_p_; //縦波における体と推進方向の最大角度,大きいほど縦波の振幅が大きい[rad]
  double l_; //横波の波長[m]
  double alpha_; //横波における体と推進方向の最大角度、大きいほど横波の振幅が大きい[rad]
  double t_p_;  //ペダルウェーブの周期[s], 任意に決定する
  double t_;  //時間（実時間ではない、減算する場合も）
  double v_; //ペダルウェーブの移動速度[m/s]
  //水平ステアリングパラメータ
  bool flag_steering_y_[NUM_YAW_STEERING]; //水平方向のステアリングの有無
  double beta_[NUM_YAW_STEERING]; //ステアリングの振幅
  double min_beta_; //ステアリングの最小振幅;
  double limit_beta_; //ステアリングの最大振幅
  double t_s_[NUM_YAW_STEERING]; //ステアリングを開始したタイミングの時間
  double l_s_[NUM_YAW_STEERING]; //ステアリング時の横波の波長
  //垂直ステアリングパラメータ
  bool flag_steering_p_;
  double beta_p_;
  double min_beta_p_;
  double limit_beta_p_;
  double t_s_p_;
  double l_s_p_;


};





#endif /* SNAKE_CONTROL_SRC_PEDAL_WAVE_H_ */
