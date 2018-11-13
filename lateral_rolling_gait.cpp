/**
 * @file lateral_rolling.cpp
 * @brief 
 * @author Hiroki_SUHARA
 * @date 2016/09/13
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "lateral_rolling_gait.h"


/** @fn
 * @brief 
 * @param double A 
 * @detail
 *  
 */
void LateralRollingGait::set_A(double A){ A_ = A; }


/** @fn
 * @brief 
 * @param double pitch [m] 
 * @detail
 *  螺旋半径と制限値から求まる最小のピッチを考慮して設定される
 */
void LateralRollingGait::set_omega(double omega){ omega_ = omega; }

void LateralRollingGait::init(){}

std::vector<double> LateralRollingGait::Rolling2(RobotSpec spec)
{
  std::vector<double> joint_angle(spec_.num_joint(), 0.0);  // 値0.0で初期化

  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {
    if ( (i_joint+1)%2 == (uint8_t)spec_.odd_joint_is_yaw() ) {
      joint_angle[i_joint] = A_ * cos(omega_);
    }else{
      joint_angle[i_joint] = A_ * sin(omega_);
    }
  }


  return joint_angle;
}

/** @fn
 * @brief ロボットを捻転させるようにパラメータを変更
 * @param double ratio_to_max [-1.0, 1.0] 最大捻転速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void LateralRollingGait::Rolling(RobotSpec spec)
{

  /*
    int unit_num  = spec.num_joint();

    curve.A       = A_;
    curve.omega   = omega_;
    ROS_INFO("curve.A is  %f", curve.A);
    ROS_INFO("curve.omega is  %f", curve.omega);

    for(int i=0; i<unit_num; i++){

    snake_model_param.A.insert(snake_model_param.A.begin()+i, curve.A);
    snake_model_param.omega.insert(snake_model_param.omega.begin()+i, curve.omega);
    }

    double target_angle = 0;
    snake_model_param.angle.resize(spec.num_joint());
    	
    for(int i=0; i<unit_num; i++){
    //(奇数番目)
    if(i%2 != 0){

    target_angle = snake_model_param.A[i] * cos(snake_model_param.omega[i]);
    snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
		  
    snake_model_param.A.pop_back();
    snake_model_param.omega.pop_back();
    snake_model_param.angle.pop_back();
    //ROS_INFO("snake_model_param.angle[ %d ] is  %f", i, snake_model_param.angle[i]);

    //(偶数番目)
    }else{

    target_angle = snake_model_param.A[i] * sin(snake_model_param.omega[i]);
    snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
		  
    snake_model_param.A.pop_back();
    snake_model_param.omega.pop_back();        	  
    snake_model_param.angle.pop_back();
    //ROS_INFO("snake_model_param.angle[ %d ] is  %f", i, snake_model_param.angle[i]);
    }
    }

  */
}








