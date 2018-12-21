/**
 * @file pedal_wave_motion2.cpp
 * @brief 横うねり運動のためのクラス
 * @author Taichi AKIYAMA
 * @date 2018/10/02
 * @detail
 */

#include <ros/ros.h>
#include "pedal_wave_motion2.h"

/** @fn
 * @brief コンストラクタ
 * @param なし
 * @detail
 */
PedalWaveMotion2::PedalWaveMotion2(RobotSpec spec)
{
  spec_ = spec;
  l_ = spec_.full_length()/1.5;
  alpha_ = M_PI/6;
  l_p_ = spec_.full_length()/3;
  alpha_p_ = M_PI/12;
  t_p_ = 1.0;
  t_ = 0.0;
  //v_ = l_p_/t_p_;
  v_ = 0.05/t_p_;
  min_beta_ = 1.0;
  limit_beta_ = 2.0;
  for(int i=0; i<NUM_YAW_STEERING; i++){
    flag_steering_y_[i] = false;
    beta_[i] = 0.0;
    l_s_[i] = spec_.full_length()*2/3;
  }
  flag_steering_p_ = false;
  min_beta_p_ = 0.1;
  limit_beta_p_ = 2.0;
  beta_p_ = 0.0;
  l_s_p_ = spec_.full_length()/2;
}

/** @fn
 * @brief デストラクタ
 * @param なし
 * @detail
 */
PedalWaveMotion2::~PedalWaveMotion2()
{
}

/** @fn
 * @brief 関節角度計算
 * @param なし
 * @return std::vector<double>
 * @detail
 */
std::vector<double> PedalWaveMotion2::CalcJointAngle()
{
  std::vector<double> target_joint_angle(spec_.num_joint(), 0.0); //値0.0で初期化
  double s = 0.0;
  double kappa_p, kappa_y = 0.0;
  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {
    s = (i_joint+1)*spec_.link_length_body();
    if ( (i_joint+1)%2 == (uint8_t)spec_.odd_joint_is_yaw() ) { //ヨ−軸、すなわち横波、ステアリング
      for(int i=0; i<NUM_YAW_STEERING; i++){
        if(flag_steering_y_[i]){
          if(spec_.link_length_body()<=s-v_*(t_-t_s_[i])&&s-v_*(t_-t_s_[i])<=l_s_[i]/2){
            kappa_y = CalcKappaSY(i, s-v_*(t_-t_s_[i]));
            break;
          }else{
            kappa_y = 0;
          }
          if(i_joint+1 == 2 - (uint8_t)spec_.odd_joint_is_yaw()){ //ピッチ軸、すなわち進行波
            if((2- (uint8_t)spec_.odd_joint_is_yaw())*spec_.link_length_body()<s-v_*(t_-t_s_[i])){
              FinishSteering(i);
            }
          }
          else if(i_joint+1 == spec_.num_joint()- (uint8_t)spec_.odd_joint_is_yaw()){
            if(s-v_*(t_-t_s_[i])<(1-(uint8_t)spec_.odd_joint_is_yaw())*spec_.link_length_body() + l_s_[i]/2){
              FinishSteering(i);
            }
          }
        }else{
          kappa_y = CalcKappaYNoSteering(s-v_*t_);
        }
      }
      target_joint_angle[i_joint] = 2*spec_.link_length_body()*kappa_y;
    }else{
      if(flag_steering_p_){
        if(spec_.link_length_body()<=s-v_*(t_-t_s_p_)&&s-v_*(t_-t_s_p_)<=l_s_p_/2){
          kappa_p = CalcKappaPW(s, t_) + CalcKappaSP(s-v_*(t_-t_s_p_));
        }else{
          kappa_p = CalcKappaPW(s, t_);
        }
      }else{
        kappa_p = CalcKappaPW(s, t_);
      }
      if(i_joint+1 == 1 + (uint8_t)spec_.odd_joint_is_yaw()){
        kappa_p = kappa_p*0.8;
      }
      else if(i_joint+1 == spec_.num_joint() - (1 - (uint8_t)spec_.odd_joint_is_yaw())){
        kappa_p = kappa_p*0.8;
      }
      target_joint_angle[i_joint] = 2*spec_.link_length_body()*kappa_p;
    }
    ROS_INFO("angle %d = %f", i_joint, target_joint_angle[i_joint]);
  }
  return target_joint_angle;
}

/** @fn
 * @brief コントローラによるパラメータ変更
 * @param joy_handler::JoySelectedData, double
 * @return なし
 * @detail
 */
void PedalWaveMotion2::ChangeParam(joy_handler::JoySelectedData joy_data, double sampling_time)
{
  //前進後退
  if(joy_data.joy_stick_l_y_upwards != 0.05){
   AddT(sampling_time*joy_data.joy_stick_l_y_upwards);
  }

  //水平方向ステアリング
  if(joy_data.closs_key_left){
    AddBeta(0, 0.01);
  }
  if(joy_data.closs_key_right){
    AddBeta(0, -0.01);
  }
  if(joy_data.button_square){
    AddBeta(1, 0.01);
  }
  if(joy_data.button_circle){
    AddBeta(1, -0.01);
  }

  //垂直方向ステアリング
  if(joy_data.closs_key_up){
    AddBetaP(0.01);
  }
  if(joy_data.closs_key_down){
    AddBetaP(-0.01);
  }

  // static bool flag_auto = false;
  // if(joy_data.button_triangle){
  //   flag_auto = true;
  // }
  // if(joy_data.button_cross){
  //   flag_auto = false;
  // }
  //
  // if(flag_auto){
  //   AddT(sampling_time);
  // }
}

/** @fn
 * @brief 各種パラメータ表示
 * @param なし
 * @return なし
 * @detail
 */
void PedalWaveMotion2::PrintParameters()
{
	ROS_INFO("* --> flag_steering_y1   = [   %d   ] *", flag_steering_y_[0]);
	ROS_INFO("* -->            beta1   = [%4.3f ] *", beta_[0]);
  ROS_INFO("* --> flag_steering_y2   = [   %d   ] *", flag_steering_y_[1]);
  ROS_INFO("* -->            beta2   = [%4.3f ] *", beta_[1]);
  ROS_INFO("* --> flag_steering_p1   = [   %d   ] *", flag_steering_p_);
	ROS_INFO("* -->            betap   = [%4.3f ] *", beta_p_);
	ROS_INFO("* -->                t   = [%4.3f ] *", t_);
	ROS_INFO("------   Pedal Wave Motion  ---------");
}

/** @fn
 * @brief ペダルウェーブ成分のピッチ軸曲率
 * @param double
 * @return double
 * @detail
 */
double PedalWaveMotion2::CalcKappaPW(double s, double t)
{
  double kappa_pw = 0;
  kappa_pw = (2*M_PI/l_p_)*alpha_p_*sin(2*M_PI*(s/l_p_ + t/t_p_));
  return kappa_pw;
}

/** @fn
* @brief ステアリングがない場合のヨー軸の曲率計算
* @param double
* @return double
* @detail
*/
double PedalWaveMotion2::CalcKappaYNoSteering(double s)
{
  double kappa_y = 0;
  kappa_y = (2*M_PI/l_)*alpha_*sin(2*M_PI*s/l_);
  return kappa_y;
}

/** @fn
 * @brief ステアリングがある場合のステアリング部のヨー軸の曲率計算
 * @param  int,double
 * @return double
 * @detail
 */
double PedalWaveMotion2::CalcKappaSY(int i, double s)
{
  double kappa_sy = 0;
  kappa_sy = (M_PI/l_s_[i])*beta_[i]*sin(2*M_PI*s/l_s_[i]); //ステアリング時のl_はまた別に定義してもいいかも
  return kappa_sy;
}

/** @fn
 * @brief ステアリングがある場合のステアリング部のピッチ軸の曲率計算
 * @param  double
 * @return double
 * @detail
 */
double PedalWaveMotion2::CalcKappaSP(double s)
{
  double kappa_sp = 0;
  kappa_sp = (M_PI/l_s_p_)*beta_p_*sin(2*M_PI*s/l_s_p_); //ステアリング時のl_はまた別に定義してもいいかも
  return kappa_sp;
}

/** @fn
 * @brief t_の加算
 * @param double
 * @return なし
 * @detail
 */
 void PedalWaveMotion2::AddT(double dt)
 {
   t_ += dt;
 }

/** @fn
 * @brief beta_の加算
 * @param double
 * @return なし
 * @detail
 */
void PedalWaveMotion2::AddBeta(int i, double db)
{
  double pre_beta = beta_[i];
  if(pre_beta>limit_beta_ && db>0)
    return;
  if(pre_beta<-limit_beta_ && db<0)
    return;
  beta_[i] += db;
  if (fabs(pre_beta) <= min_beta_ && fabs(beta_[i]) > min_beta_){
    StartSteering(i);
  }
  if (fabs(pre_beta) > min_beta_ && fabs(beta_[i]) <= min_beta_){
    FinishSteering(i);
  }
}

/** @fn
 * @brief ステアリング状態の開始
 * @param なし
 * @return bool
 * @detail
 */
void PedalWaveMotion2::StartSteering(int i)
{
  flag_steering_y_[i] = true;
  t_s_[i] = t_;
}

/** @fn
 * @brief ステアリング状態の終了
 * @param なし
 * @return bool
 * @detail
 */
void PedalWaveMotion2::FinishSteering(int i)
{
  flag_steering_y_[i] = false;
  beta_[i] = 0.0;
}

/** @fn
 * @brief beta_の加算
 * @param double
 * @return なし
 * @detail
 */
void PedalWaveMotion2::AddBetaP(double db)
{
  double pre_beta = beta_p_;
  if(pre_beta>limit_beta_p_ && db>0)
    return;
  if(pre_beta<-limit_beta_p_ && db<0)
    return;
  beta_p_ += db;
  if (fabs(pre_beta) <= min_beta_p_ && fabs(beta_p_) > min_beta_p_){
    StartSteeringP();
  }
  if (fabs(pre_beta) > min_beta_p_ && fabs(beta_p_) <= min_beta_p_){
    FinishSteeringP();
  }
}

/** @fn
 * @brief ステアリング状態の開始
 * @param なし
 * @return bool
 * @detail
 */
void PedalWaveMotion2::StartSteeringP()
{
  flag_steering_p_ = true;
  t_s_p_ = t_;
}

/** @fn
 * @brief ステアリング状態の終了
 * @param なし
 * @return bool
 * @detail
 */
void PedalWaveMotion2::FinishSteeringP()
{
  flag_steering_p_ = false;
  beta_p_ = 0.0;
}
