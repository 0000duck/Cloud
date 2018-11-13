/**
 * @file lateral_rolling.h
 * @brief ラテラルのためのクラス
 * @author TI
 * @date 2016/09/14
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_LATERAL_ROLLING_GAIT_H_
#define SNAKE_CONTROL_SRC_LATERAL_ROLLING_GAIT_H_

#include <ros/ros.h>
#include <vector>

#include "robot_spec.h"
#include "simple_shape_connection.h"

#include <stdint.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

/** @fn
 * @brief 
 * @detail
 *  必ずしもその必要はないが，SimpleShapeConnectionの枠組みで作る
 *  一つの螺旋がずっとループする
 */

class LateralRollingGait : public SimpleShapeConnection {
 public:

  virtual ~LateralRollingGait(){}

 LateralRollingGait(RobotSpec spec,
		    uint32_t num_ds_link_body,
		    double min_A,
		    double max_A,
		    double min_omega,
		    double max_omega,
		    double ds
                    ) : SimpleShapeConnection(spec, num_ds_link_body) {

    min_A_ = min_A;
    max_A_ = max_A;
    min_omega_ = min_omega;
    max_omega_ = max_omega;

    gait_name_ = "LateralRolling";
    InitializeShape();
		
    A_ = 0.15;
    omega_ = 0.0;
  }

  // 形状パラメータ
  double omega_;
  double A_;

  //--- 形状パラメータ変更
  void set_A(double A);
  void add_A(double A_add){ set_A(A_ + A_add); }

  void set_omega(double omega);
  void add_omega(double omega_add){ set_omega(omega_ + omega_add); }

  //--- 動作
  void init();
  void Rolling(RobotSpec spec);
  //void RollingShift();
  std::vector<double> Rolling2(RobotSpec spec);

  virtual void InitializeShape() {}

  // 形状パラメータ制限用
  double min_A_;
  double max_A_;
  double min_omega_;
  double max_omega_;

  /***   ヘビの各関節の実行するパラメータ    ***/
  typedef struct{     
    std::vector<double> angle;
    std::vector<double> A;
    std::vector<double> omega;

  }SNAKE_MODEL_PARAM;

  typedef struct{
    double A;
    double omega;

  }CURVATURE;

  SNAKE_MODEL_PARAM   snake_model_param;
  CURVATURE           curve;
	
};

#endif /* SNAKE_CONTROL_SRC_SIDEWINDING_GAIT_H_ */
