/**
 * @file helical_rolling_gait.h
 * @brief 螺旋捻転推進のためのクラス
 * @author Tatsuya TAKEMORI
 * @date 2016/08/13
 * @detail
 */

#ifndef SNAKE_CONTROL_SRC_SIDEWINDING_GAIT_H_
#define SNAKE_CONTROL_SRC_SIDEWINDING_GAIT_H_

#include <ros/ros.h>
#include <vector>

#include "robot_spec.h"
#include "simple_shape_connection.h"

#include <stdint.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

/** @fn
 * @brief 横うねりのクラス
 * @detail
 *  必ずしもその必要はないが，SimpleShapeConnectionの枠組みで作る
 *  一つの螺旋がずっとループする
 */

class SidewindingGait : public SimpleShapeConnection {
 public:

	virtual ~SidewindingGait(){}

	SidewindingGait(RobotSpec spec,
                     uint32_t num_ds_link_body,
                     double min_alpha,
                     double max_alpha,
                     double min_l,
                     double max_l,
					 double ds
                    ) : SimpleShapeConnection(spec, num_ds_link_body) {

		min_alpha_ = min_alpha;
		max_alpha_ = max_alpha;
		min_l_ = min_l;
		max_l_ = max_l;

		gait_name_ = "Sidewinding";
		InitializeShape();

		radius_ = 0.05;
		b_      = 0.15;

	}

	double radius_;  //半径
	double b_;

	//--- 動作
	void init();
	void Sidewinding(RobotSpec spec);
	void WindingShift();

	virtual void InitializeShape() {

	}

	//--- 形状パラメータ変更
	void set_psi(double psi);
	void add_psi(double psi_add){ set_psi(psi_+psi_add); }

	void set_radius(double radius);
	void add_radius(double radius_add){ set_radius(radius_+radius_add); }

	void set_b(double b);
	void add_b(double b_add){ set_b(b_ + b_add); }

 //protected:
  // 制限値計算
  //double CalcMaxShiftVelocity();
  //double CalcMaxRollingVelocity();
  //void ReshapeSegmentParameter();
  // セグメントパラメータ計算用
  //static double CalcCurvatureHelix(double diameter, double pitch);
  //static double CalcTorsionHelix(double diameter, double pitch)
 //protected:
  // 形状パラメータ

	double psi_;
	double l_;      //
	double alpha_;     //

	double alpha_s;   //

 //private:

	// 形状パラメータ制限用
	double min_alpha_;
	double max_alpha_;
	double min_l_;
	double max_l_;
	double bias_max_;		// 最大バイアス 単位は[rad]
	double v_max_;			// 最大速度[m/s]

	/***   サーペノイド曲線用パラメータ    ***/
	typedef struct {
    		double alpha_s;       // 生物機械工学の(3.10)式
        	double alpha;         // くねり角[rad]
        	double l;             // 曲線の1/4周期の長さ[m]

    	}SERPENOID_CURVE;

        /***   ヘビの各関節の実行するパラメータ    ***/
    typedef struct{      // τ(s)=0，κ_p(s)およびκ_y(s)は任意の背びれ曲線のパラメータ．横うねり推進を考慮して"bias"もパラメータとしてメンバにした．
    	 std::vector<double> angle;
         std::vector<double> bias;
         std::vector<double> kappa;
         std::vector<double> tau;
         std::vector<double> psi;
         std::vector<double> beta;

     }SNAKE_MODEL_PARAM;


             /***   ヘビの各関節のデックのなかのデータ,  シフトするパラメータ    ***/
     typedef struct{

    	 std::vector<double> kappa_hold;
    	 std::vector<double> tau_hold;
    	 std::vector<double> bias_hold;
    	 std::vector<double> psi_hold;
    	 std::vector<double> phi_hold;
    	 std::vector<double> flag_hold;

     }SHIFT_PARAM;

     typedef struct{
    	 std::vector<SHIFT_PARAM> shift_param;

     }HOLD_DATA;

     typedef struct{

    	double radius;
     	double b;
     	double psi;
	double beta;

	double kappa;
	double tau;

     }CURVATURE;

     HOLD_DATA hold_data;

     SERPENOID_CURVE     serpenoid_curve;
     SNAKE_MODEL_PARAM   snake_model_param;
     CURVATURE           curve;
     //std::vector<SHIFT_PARAM> shift_param;
	
};

#endif /* SNAKE_CONTROL_SRC_SIDEWINDING_GAIT_H_ */
