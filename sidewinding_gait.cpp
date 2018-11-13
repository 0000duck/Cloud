/**
 * @file sidewinding_gait.cpp
 * @brief 
 * @author TI
 * @date 2016/09/7
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "sidewinding_gait.h"


/** @fn
 * @brief 
 * @param double alpha
 * @detail
 *  
 */
void SidewindingGait::set_psi(double psi){ psi_ = psi; }

/** @fn
 * @brief 螺旋のピッチを変更する
 * @param double pitch [m] 螺旋のピッチ
 * @detail
 *  螺旋半径と制限値から求まる最小のピッチを考慮して設定される
 */
void SidewindingGait::set_b(double b){ b_ = b; }

void SidewindingGait::set_radius(double radius){ radius_ = radius; };



void SidewindingGait::init()
{
}


/** @fn
 * @brief ロボットを捻転させるようにパラメータを変更
 * @param double ratio_to_max [-1.0, 1.0] 最大捻転速さに対する割合snake_model_param.phi[i]
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void SidewindingGait::Sidewinding(RobotSpec spec) {


	curve.radius = radius_;
	curve.b      = b_;
	double r  = curve.radius;
	double b  = curve.b;

	ROS_INFO("curve.radius is  %f", curve.radius);
	ROS_INFO("curve.b is  %f", curve.b);

	double ds = spec.link_length_body();

	int unit_num = spec.num_joint();

	curve.kappa = r/(pow(r, 2) + pow(b, 2));
	curve.tau   = b/(pow(r, 2) + pow(b, 2));
 

	for(int i=0; i<unit_num; i++){
		if(i>1){

			snake_model_param.tau.insert(snake_model_param.tau.begin()+i, curve.tau * ds + snake_model_param.tau[i-1]);
		}else{

			snake_model_param.tau.insert(snake_model_param.tau.begin()+i, curve.tau * ds);
		}
		snake_model_param.kappa.insert(snake_model_param.kappa.begin()+i, curve.kappa);

		if(i%2!= 0){
			snake_model_param.beta.insert(snake_model_param.beta.begin()+i, 0);

		}else{
			snake_model_param.beta.insert(snake_model_param.beta.begin()+i, M_PI/4);

		}
		snake_model_param.psi.insert(snake_model_param.psi.begin()+i, psi_);

	}

	double target_angle = 0;
	snake_model_param.angle.resize(spec.num_joint());

	for(int i=0; i<unit_num; i++){
	  	//(奇数番目)
	        if(i%2 != 0){
			target_angle = 2 * ds * snake_model_param.kappa[i] * cos(snake_model_param.tau[i] + snake_model_param.psi[i] + snake_model_param.beta[i]);

        		snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
        		snake_model_param.angle.pop_back();
			snake_model_param.tau.pop_back();
			snake_model_param.beta.pop_back();
			snake_model_param.psi.pop_back();
              		//ROS_INFO("snake_model_param.angle[ %d ] is  %f", i, snake_model_param.angle[i]);

          	//(偶数番目)
          	}else{
        		target_angle = 2 * ds * snake_model_param.kappa[i] * sin(snake_model_param.tau[i] + snake_model_param.psi[i] + snake_model_param.beta[i]);
        	  
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
        		snake_model_param.angle.pop_back();
			snake_model_param.tau.pop_back();
			snake_model_param.beta.pop_back();
			snake_model_param.psi.pop_back();
                	//ROS_INFO("snake_model_param.angle[ %d ] is  %f", i, snake_model_param.angle[i]);
          }
     }

}

/** @fn
 * @brief SShiftを行う．動作としては捻転するだけ
 * @param double ratio_to_max [-1.0, 1.0] 最大速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
 */
void SidewindingGait::WindingShift() {
  //SShift(ratio_to_max*CalcMaxShiftVelocity()*time_move);
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
//double HelicalRollingGait::CalcMaxShiftVelocity() {
//  return 0.20;
//}

/** @fn
 * @brief 最大の捻転速度を求める
 * @param なし
 * @return なし
 * @detail
 */
//double HelicalRollingGait::CalcMaxRollingVelocity() {
//  return spec_.max_joint_angle_velocity() /
//      (2*spec_.link_length_body()*CalcCurvatureHelix(diameter_, pitch_));
//}

/** @fn
 * @brief 形状パラメータを元にセグメントパラメータを更新する
 * @param なし
 * @return なし
 * @detail
 *  ただの螺旋
 */
//void HelicalRollingGait::ReshapeSegmentParameter() {
//  segment_.resize(1);
//  segment_[0].ReshapeAsHelix(diameter_/2.0, pitch_, 2.0*M_PI, 0.0);
//  segment_parameter_was_changed_ = true;
//}

/** @fn
 * @brief 螺旋の曲率を計算する
 * @param double diameter [m] 螺旋の直径
 * @param double pitch [m] 螺旋のピッチ
 * @return double [1/m] 螺旋の曲率
 * @detail
 */
//double HelicalRollingGait::CalcCurvatureHelix(double diameter, double pitch) {
//  double a = diameter/2.0;
//  double b = pitch/(2*M_PI);
//  return a/(a*a + b*b);
//}

/** @fn
 * @brief 螺旋の捻率を計算する
 * @param double diameter [m] 螺旋の直径
 * @param double pitch [m] 螺旋のピッチ
 * @return double 螺旋の捻率
 * @detail
 */
//double HelicalRollingGait::CalcTorsionHelix(double diameter, double pitch) {
//  double a = diameter/2.0;
//  double b = pitch/(2*M_PI);
//  return b/(a*a + b*b);
//}

