/**
 * @file helical_curve_gait.cpp
 * @brief 
 * @author TI
 * @date 2016/09/7
 * @detail
 */

#include <ros/ros.h>
#include <cmath>
#include "helical_curve_gait.h"

#include "snake_control_request.h"
#include "snake_control.h"

/** @fn
 * @brief 
 * @param 
 * @detail
 *  
 */
// void HelicalCurveGait::set_alpha(double alpha) {

// }

// void HelicalCurveGait::set_l(double l) {
 
// }

// void HelicalCurveGait::init()
// {

// }

/** @fn
 * @brief 目標関節角を計算する
 * @param なし
 * @return std::vector<double> joint_angle(spec_.num_joint())
 * @detail
 */
std::vector<double> HelicalCurveGait::CalcJointAngle2() {

  std::vector<double> kappa(spec_.num_joint(), 0.0);  // 値0.0で初期化
  std::vector<double> tau(spec_.num_joint(), 0.0);  // 値0.0で初期化


  //double ds = spec_.link_length_body();
  double ds = 0.0905;	//highpower
  //double ds = 0.08;	//middle
  //double r = spec_.link_diameter()/2.0;
  double r = 0.035;	//highpower
  //double r = 0.05;	//middle

  double s = 0;
  double cs = 0;
  double t = 0;
  //double vs = 0;
  double to = (r*omega*dt*sin(al))/R + theta_off_0; //theta_offset
  double b = 0;
  double dth_reso = 1000; //積分の分解能
  double dth = (ds/R)/dth_reso; //積分の幅

  //  double R00 = 0.3;
  double R00 = 0.2;
  double R01 = 100.0;

  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {

    s = ds * (i_joint + 1);
 
    int cnt = 0;
    // 線積分
    while( cs < s ){

      if( (0 <=cs) and (cs<s6))  R0 = R01;
      if( (s6<=cs) and (cs<s5))  R0 = R00;
      if( (s5<=cs) and (cs<s4))  R0 = R01;
      if( (s4<=cs) and (cs<sss)) R0 = R00;
      if( (sss<=cs) )            R0 = R01;

      end_curve_joint = abs(sss / ds);
      top_curve_joint = abs(s4 / ds);
      

/*	自動ゲインシフト（バグあり）
      if(pre_top_curve_joint != top_curve_joint){
	lowgain=1;
	pre_top_curve_joint = top_curve_joint;
      }
      if(pre_end_curve_joint != end_curve_joint){
	lowgain=1;
	pre_end_curve_joint = end_curve_joint;
      }
*/

//      if(joystick.button_select and joystick.closs_key_right){
      if(lowgain==1){
        // SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0);
	// for(int j = top_curve_joint; j<end_curve_joint+1; j++){
	// 	SnakeControl::RequestJointSetPIDGain(j, 1.0, 0.0, 0.0);	
	sss = 0.0;
	s4 = 0.0;
	s5 = 0.0;
	s6 = 0.0;
	}
      //   }


      

      //      else{
//	SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0);
//      }


      //      SnakeControl::RequestJointSetPIDGainAll(1.0, 0.0, 0.0); 


      //      SnakeControl::RequestJointSetPIDGain(2, 1.0, 0.0, 0.0); 
      
      /*
      if( (0 <=cs) and (cs<s6)) {
	R0 = R01;
	SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0); 
      }

      if( (s6<=cs) and (cs<s5)) {
	R0 = R00;
	SnakeControl::RequestJointSetPIDGainAll(3.0, 0.0, 0.0); 
      }

      if( (s5<=cs) and (cs<s4)) {
	R0 = R01;
	SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0); 
      }

      if( (s4<=cs) and (cs<sss)) {
	R0 = R00;
	SnakeControl::RequestJointSetPIDGainAll(3.0, 0.0, 0.0); 
      }
	
      if( (sss<=cs) ) {
	R0 = R01;
	SnakeControl::RequestJointSetPIDGainAll(32.0, 0.0, 0.0); 
      }
      */


      double A = (pow(R,2)*pow(tan(al),2))/2 + pow(R0,2)*pow(tan(al),2) + pow(R0,2);
      double B = 2*R0*R*pow(tan(al),2);
      double C = (pow(R,2)*pow(tan(al),2))/2;
      
      //旧式
      //cs = cs + R/R0 * sqrt( A + B*cos(b) + C*(2*pow(cos(b),2) - 1) ) * dth;
      //t = t + dth;
      //b = t + to;
      
      //せめて台形法で積分
      double F1 = R/R0 * sqrt( A + B*cos(b) + C*(2*pow(cos(b),2) - 1) );
      t = t + dth;
      b = t + to;
      double F2 = R/R0 * sqrt( A + B*cos(b) + C*(2*pow(cos(b),2) - 1) );
      double dA = (F1 + F2)*dth/2.0;
      cs = cs + dA;

      cnt++;
    }
    //ROS_INFO("s = %f, cs = %f, th = %f, cnt = %d", s, cs, t, cnt);

kappa[i_joint] = pow(fabs(R),-1)*pow((pow(R,2)*pow(tan(al),2)*cos(2*(to+t)))/
    2.0+2*R*R0*pow(tan(al),2)*cos(to+t)+pow(R0,2)*pow(tan(al),2)+(pow(R,2)*
    pow(tan(al),2))/2.0+pow(R0,2),(-3.0)/
    2.0)*sqrt((pow(R,6)*pow(tan(al),6)*cos(4*(to+t)))/
    8.0-(pow(R,4)*pow(R0,2)*pow(tan(al),4)*cos(4*(to+t)))/
    8.0+pow(R,5)*R0*pow(tan(al),6)*cos(3*(to+t))+3*pow(R,4)*pow(R0,2)*
    pow(tan(al),6)*cos(2*(to+t))+(pow(R,6)*pow(tan(al),6)*cos(2*(to+t)))/
    2.0+(3.0*pow(R,2)*pow(R0,4)*pow(tan(al),4)*cos(2*(to+t)))/
    2.0+(3.0*pow(R,4)*pow(R0,2)*pow(tan(al),4)*cos(2*(to+t)))/
    2.0-(pow(R,2)*pow(R0,4)*pow(tan(al),2)*cos(2*(to+t)))/
    2.0+4*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos(to+t)+3*pow(R,5)*R0*
    pow(tan(al),6)*cos(to+t)+2*R*pow(R0,5)*pow(tan(al),4)*cos(to+t)+8*pow(R,3)*
    pow(R0,3)*pow(tan(al),4)*cos(to+t)+4*R*pow(R0,5)*pow(tan(al),2)*
    cos(to+t)+pow(R,2)*pow(R0,4)*pow(tan(al),6)+3*pow(R,4)*pow(R0,2)*
    pow(tan(al),6)+(3.0*pow(R,6)*pow(tan(al),6))/
    8.0+(11.0*pow(R,2)*pow(R0,4)*pow(tan(al),4))/
    2.0+(13.0*pow(R,4)*pow(R0,2)*pow(tan(al),4))/
    8.0+pow(R0,6)*pow(tan(al),2)+(7.0*pow(R,2)*pow(R0,4)*pow(tan(al),2))/
    2.0+pow(R0,6));

tau[i_joint] = pow(R,-1)*(pow(tan(al),3)*(2*pow(R,3)*R0*(pow(R0,2)*cos(3*(to+t))-13*pow(R0,2)*
    cos(to+t))-16*pow(R,2)*pow(R0,4)*cos(2*(to+t))+8*R*pow(R0,5)*
    cos(to+t))+pow(tan(al),5)*((-2*pow(R,5)*R0*(cos(3*(to+t))+3*cos(to+t)))-2*
    pow(R,4)*R0*(4*R0*cos(2*(to+t))+4*R0)-8*pow(R,3)*pow(R0,3)*
    cos(to+t))+tan(al)*(8*pow(R0,6)-16*R*pow(R0,5)*cos(to+t)))*pow(pow(R,6)*
    pow(tan(al),6)*cos(4*(to+t))-pow(R,4)*pow(R0,2)*pow(tan(al),4)*cos(4*
    (to+t))+8*pow(R,5)*R0*pow(tan(al),6)*cos(3*(to+t))+24*pow(R,4)*pow(R0,2)*
    pow(tan(al),6)*cos(2*(to+t))+4*pow(R,6)*pow(tan(al),6)*cos(2*(to+t))+12*
    pow(R,2)*pow(R0,4)*pow(tan(al),4)*cos(2*(to+t))+12*pow(R,4)*pow(R0,2)*
    pow(tan(al),4)*cos(2*(to+t))-4*pow(R,2)*pow(R0,4)*pow(tan(al),2)*cos(2*
    (to+t))+32*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos(to+t)+24*pow(R,5)*R0*
    pow(tan(al),6)*cos(to+t)+16*R*pow(R0,5)*pow(tan(al),4)*cos(to+t)+64*
    pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos(to+t)+32*R*pow(R0,5)*pow(tan(al),2)*
    cos(to+t)+8*pow(R,2)*pow(R0,4)*pow(tan(al),6)+24*pow(R,4)*pow(R0,2)*
    pow(tan(al),6)+3*pow(R,6)*pow(tan(al),6)+44*pow(R,2)*pow(R0,4)*
    pow(tan(al),4)+13*pow(R,4)*pow(R0,2)*pow(tan(al),4)+8*pow(R0,6)*
    pow(tan(al),2)+28*pow(R,2)*pow(R0,4)*pow(tan(al),2)+8*pow(R0,6),-1);


    /*
    //常螺旋
    double a = R;
    double b = al/(2*M_PI);
    kappa[i_joint] = a/(a*a + b*b);
    tau[i_joint] = b/(a*a + b*b);
    */
  }


  //std::vector<double> kappa(spec_.num_joint(), 5.0); 
  //std::vector<double> tau(spec_.num_joint(), 1.0);  

  double tauds; 
  double psi;
  psi = psi_0;
  double kappa_p;
  double kappa_y;
  std::vector<double> joint_angle(spec_.num_joint(), 0.0);  // 値0.0で初期化




////
 if(x==0 && y ==0){
////
  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {

    tauds = tau[i_joint] * ds;
    psi = psi + tauds;
    
    if ( (i_joint+1)%2 == (uint8_t)spec_.odd_joint_is_yaw() ) {
      kappa_y = kappa[i_joint] * (-std::sin(psi));
      joint_angle[i_joint] = 2 * ds * kappa_y;
    }else{
      kappa_p = kappa[i_joint] * std::cos(psi);
      joint_angle[i_joint] = 2 * ds * kappa_p;
    }
  }
 }



///////////////////先頭カメラの向き変更


//	if(x > M_PI){
//		x = M_PI;
//	}

//	if(x < - M_PI){
//		x = - M_PI;
//	}

//	if(y > M_PI){
//		y = M_PI;
//	}

//	if(y < - M_PI){
//		y = - M_PI;
//	}


/////
	else{
	
  for (uint32_t i_joint=0; i_joint<spec_.num_joint(); i_joint++) {

    tauds = tau[i_joint] * ds;
    psi = psi + tauds;
    
    if ( (i_joint+1)%2 == (uint8_t)spec_.odd_joint_is_yaw() ) {
      kappa_y = kappa[i_joint] * (-std::sin(psi));
      joint_angle[i_joint] = 2 * ds * kappa_y;
    }else{
      kappa_p = kappa[i_joint] * std::cos(psi);
      joint_angle[i_joint] = 2 * ds * kappa_p;
    }
  }
/////

	joint_angle[0] = joint_angle[0] + x;
	joint_angle[1] = joint_angle[1] + y;


	if(joint_angle[0] > M_PI/2){
		joint_angle[0] = M_PI/2-0.05;
	}	

	if(joint_angle[0] < - M_PI/2){
		joint_angle[0] = - M_PI/2+0.05;
	}

	if(joint_angle[1] > M_PI/2){
		joint_angle[1] = M_PI/2-0.05;
	}	

	if(joint_angle[1] < - M_PI/2){
		joint_angle[1] = - M_PI/2+0.05;
	}
	}




////////////////////////////

     ROS_INFO(" sss = %f", sss); 
     ROS_INFO(" s4 = %f", s4); 
     ROS_INFO(" s5 = %f", s5);
     ROS_INFO(" s6 = %f", s6);
     ROS_INFO(" theta_off_0 = %f", theta_off_0);
     ROS_INFO(" x = %f", x);
     ROS_INFO(" y = %f", y);
     //     ROS_INFO(" joint_angle[0] = %f", joint_angle[0]);
     //     ROS_INFO(" joint_angle[1] = %f", joint_angle[1]);
     ROS_INFO(" R = %f", R);
     ROS_INFO(" top_curve_joint = %f", top_curve_joint);
     ROS_INFO(" end_curve_joint = %f", end_curve_joint);
     //     ROS_INFO(" pre_top_curve_joint = %f", pre_top_curve_joint);
     //     ROS_INFO(" pre_end_curve_joint = %f", pre_end_curve_joint);
     //     ROS_INFO(" lowgain = %f", lowgain);
     //     ROS_INFO(" cop_time = %f", cop_time);
     //     ROS_INFO(" count = %f", count);
     //     ROS_INFO(" abs_count = %f", abs_count);

     /*	if(cop_time==1){
	    SnakeControlRequest::RequestCOPDataAll(); 
	    ros::Duration(0.1).sleep();
	    }*/

     // for(int i=0;i<20;i++){
     //   ROS_INFO(" joint_angle[i] = %f", joint_angle[i]);
     // }

  return joint_angle;
  

}


////////////////////////////////////////////////////////////////////////////////////////////

void HelicalCurveGait::Winding(RobotSpec spec) {

  
  // double kappa = 0;
  // double tau = 0;

  // for(int s=0;s<dss*30;s=s+dss){

  //   while(cs <= s){

  //     to = (vs*r*dt*sin(al))/R + beta;

  //     cs = cs + R/R0 * sqrt( A + B*cos(b) + C*(2*pow(cos(b),2) - 1) ) * dth;

  //     A = (pow(R,2)*pow(tan(al),2))/2 + pow(R0,2)*pow(tan(al),2) + pow(R0,2);
  //     B = 2*R0*R*pow(tan(al),2);
  //     C = (pow(R,2)*pow(tan(al),2))/2;


  //     kappai = sqrt((pow(R,4)*((3*pow(R,6)*pow(tan(al),6))/8 + pow(R0,6)*pow(tan(al),2) + pow(R0,6) + (pow(R,6)*cos(2*(t + to))*pow(tan(al),6))/2 + (pow(R,6)*cos(4*(t + to))*pow(tan(al),6))/8 + (7*pow(R,2)*pow(R0,4)*pow(tan(al),2))/2 + (11*pow(R,2)*pow(R0,4)*pow(tan(al),4))/2 + (13*pow(R,4)*pow(R0,2)*pow(tan(al),4))/8 + pow(R,2)*pow(R0,4)*pow(tan(al),6) + 3*pow(R,4)*pow(R0,2)*pow(tan(al),6) - (pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),2))/2 + (3*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),4))/2 + (3*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),4))/2 + 3*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),6) - (pow(R,4)*pow(R0,2)*cos(4*(t + to))*pow(tan(al),4))/8 + 4*R*pow(R0,5)*pow(tan(al),2)*cos(t + to) + 2*R*pow(R0,5)*pow(tan(al),4)*cos(t + to) + 3*pow(R,5)*R0*pow(tan(al),6)*cos(t + to) + pow(R,5)*R0*cos(3*(t + to))*pow(tan(al),6) + 8*pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos(t + to) + 4*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos(t + to)))/pow(R0,6))/pow(((pow(R,2)*((pow(R,2)*pow(tan(al),2))/2 + pow(R0,2)*pow(tan(al),2) + pow(R0,2) + (pow(R,2)*cos(2*(t + to))*pow(tan(al),2))/2 + 2*R*R0*pow(tan(al),2)*cos((t + to))))/pow(R0,2)),(3/2)); 

  //     taui = ((- 2*pow(R,5)*R0*(cos(3*(t + to)) + 3*cos(t + to)) - 2*pow(R,4)*R0*(4*R0 + 4*R0*cos(2*(t + to))) - 8*pow(R,3)*pow(R0,3)*cos(t + to))*pow(tan(al),5) + (8*R*pow(R0,5)*cos(t + to) + 2*pow(R,3)*R0*(pow(R0,2)*cos(3*(t + to)) - 13*pow(R0,2)*cos(t + to)) - 16*pow(R,2)*pow(R0,4)*cos(2*(t + to)))*pow(tan(al),3) + (8*pow(R0,6) - 16*R*pow(R0,5)*cos((t + to)))*tan(al))/(R*(3*pow(R,6)*pow(tan(al),6) + 8*pow(R0,6)*pow(tan(al),2) + 8*pow(R0,6) + 4*pow(R,6)*cos(2*(t + to))*pow(tan(al),6) + pow(R,6)*cos(4*(t + to))*pow(tan(al),6) + 28*pow(R,2)*pow(R0,4)*pow(tan(al),2) + 44*pow(R,2)*pow(R0,4)*pow(tan(al),4) + 13*pow(R,4)*pow(R0,2)*pow(tan(al),4) + 8*pow(R,2)*pow(R0,4)*pow(tan(al),6) + 24*pow(R,4)*pow(R0,2)*pow(tan(al),6) - 4*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),2) + 12*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),4) + 12*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),4) + 24*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),6) - pow(R,4)*pow(R0,2)*cos(4*(t + to))*pow(tan(al),4) + 32*R*pow(R0,5)*pow(tan(al),2)*cos(t + to) + 16*R*pow(R0,5)*pow(tan(al),4)*cos((t + to)) + 24*pow(R,5)*R0*pow(tan(al),6)*cos(t + to) + 8*pow(R,5)*R0*cos(3*(t + to))*pow(tan(al),6) + 64*pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos(t + to) + 32*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos(t + to)));

  //     t = t + dth;
  //     b = t + to;

  //   }

  //   snake_model_param.kappa.insert(snake_model_param.kappa.begin()+j,kappai);
  //   snake_model_param.tau.insert(snake_model_param.tau.begin()+j,taui);

  //   j=j+1;

  // }

  // //	t = t + dth;

  // ppsi = ppsi_old + vs * dt;
  // ppsi_old = ppsi;

  // ROS_INFO(" ppsi = %f", ppsi); 	
  // ROS_INFO(" vs = %f", vs); 
  // ROS_INFO(" al = %f", al); 	
  // ROS_INFO(" dss = %f", dss); 

  // double target_angle = 0; 
  // snake_model_param.angle.resize(spec.num_joint());
  // for(int i=0; i<spec.num_joint(); i++){
  //   //(奇数番目)
  //   if(i%2){

  //     kappa_p = -snake_model_param.kappa[i] * sin(ppsi);
  //     ppsi=ppsi + snake_model_param.tau[i]*dss;
  //     target_angle = 2 * dss * kappa_p;

  //     //target_angle = -kappa * sin(i*dss+ppsi);
  //     //	    target_angle = snake_model_param.kappa[i];
  //     snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
  //     //snake_model_param.angle.pop_back();
	


  //     //(偶数番目)
  //   }else{

  //     kappa_y = snake_model_param.kappa[i]* cos(ppsi);
  //     ppsi=ppsi + snake_model_param.tau[i]*dss;
  //     target_angle = 2 * dss * kappa_y;
	
  //     //target_angle = -kappa * sin(i*dss+ppsi);
  //     //	    target_angle =snake_model_param.kappa[i];
  //     snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
  //     //snake_model_param.angle.pop_back();

  //   }
  // }























  // //  dt_       = 0.01;				// サンプリングタイム10[msec]
  // //  bias_     = 0.0;
  // //  v_max_    = 0.5;			// 最大速度[m/s]
  // //  bias_max_ = M_PI/8;			// 最大バイアス 単位は[rad]
  // //  kp_bias_  = 0.05;			// 操舵バイアス比例ゲイン
  // //  //pre_s = s = 0;
  // //  //s = 0;
  // //  v = 1;

  // //  serpenoid_curve.alpha = M_PI/8;
  // //  serpenoid_curve.l     = 0.2;

  // //  double step  = 1*v_max_*dt_;			// データ保持刻み（3*Δs_max移動したら角度を保持）

  // //  static int init_flag =1;

  // //  //ROS_INFO("**********  1  OK   *********");



  // //  /***  角度保持初期化 (一回だけ入る)***/
  // //  if(init_flag==1){
  // //    hold_data.shift_param.resize(spec.num_joint());
  // //    //init();
  // //    ////////////////////////////////	ROS_INFO(" 1 snake_model_param.size is  %d", hold_data.shift_param.size());
  // //    ///////////////////////////////	ROS_INFO(" 2 spec.num_joint() : %d ", spec.num_joint());

  // //    //ROS_INFO("**********  2  OK *********");
  // //    int max_hold_num = (int)(ds_new/step);		// 角度を保持する数 角度を保持する数
  // //    //ROS_INFO("********  2.2  OK  *********");
  // //    int hold_num     = (int)hold_data.shift_param[0].kappa_hold.size();   // デックの大きさ

  // //    //ROS_INFO(" 3 hold_num = %d ", hold_num);
  // //    //ROS_INFO(" 4 max_hold_num = %d ", max_hold_num);

  // //    //ROS_INFO("snake_model_param.angle.size is  %d", hold_data.shift_param[0].kappa_hold.size());

  // //    for(int i=0; i<spec.num_joint(); i++){

  // //      //ROS_INFO("**********  2.3  OK  *********");
  // //      hold_data.shift_param[i].bias_hold.clear();
  // //      hold_data.shift_param[i].kappa_hold.clear();
  // //      hold_data.shift_param[i].tau_hold.clear();
  // //      hold_data.shift_param[i].psi_hold.clear();

  // //      //ROS_INFO("**********  2.4  OK  *********");
  // //      //ROS_INFO("hold_data.shift_param[ %d ].kappa_hold is  %d", i, hold_data.shift_param[i].kappa_hold.size());
  // //    }

  // //    while(hold_num < max_hold_num){           // 初期値0をデックに追加しておく
  // //      for(int i=0; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  3 OK  *********");

  // //	hold_data.shift_param[i].bias_hold.push_back(0);
  // //	hold_data.shift_param[i].kappa_hold.push_back(0);
  // //	hold_data.shift_param[i].tau_hold.push_back(0);
  // //	hold_data.shift_param[i].psi_hold.push_back(0);

  // //      }
  // //      hold_num = (int)hold_data.shift_param[0].kappa_hold.size();

  // //      //ROS_INFO("hold_data.shift_param[ %d ].kappa_hold is  %d", 0, hold_data.shift_param[0].kappa_hold.size());
  // //      //ROS_INFO("**********  4 OK *********");
  // //    }
  // //    init_flag = 0;
  // //  }
  // //  //ROS_INFO(" *hold_data.shift_param[ %d ].kappa_hold is  %d", 0, hold_data.shift_param[0].kappa_hold.size());






  // //  //ROS_INFO("**********  5 OK  *********");
  // //  // 操舵指令
  // //  //ROS_INFO("v is now at position %f", v);
  // //  s += v*dt_;  // 接線方向の移動距離（移動するときの軌跡の長さ）

  // //  int j=0;
  // //  // 操舵バイアス（速度にも比例してインクリメント）

  // //  //	double psi ;

  // //  double new_target_kappa_yaw   = 0;
  // //  double new_target_kappa_pitch = 0;
  // //  double new_target_tau_yaw   = 0;
  // //  double new_target_tau_pitch = 0;
  // //  double new_target_bias        = 0;


  // //  //ROS_INFO("**********  6 OK *********");
  // //  //ROS_INFO(" s = %f ", s);
  // //  //ROS_INFO(" pre_s = %f ", pre_s);


  // //  if(mode==0){


  // //    while(s > (pre_s+step)){

  // //      //ROS_INFO("**********  7 OK  *********");

  // //      /***　先頭UNITのyaw,pitch 軸のkappa を計算する  ***/
  // //      //	      serpenoid_curve.alpha_s = (pre_s * M_PI / (2 * serpenoid_curve.l));

  // //      t = t + dth;
  // //      to = (vs*r*dt*sin(al))/R + beta;


  // //      kappa = sqrt((pow(R,4)*((3*pow(R,6)*pow(tan(al),6))/8 + pow(R0,6)*pow(tan(al),2) + pow(R0,6) + (pow(R,6)*cos(2*(t + to))*pow(tan(al),6))/2 + (pow(R,6)*cos(4*(t + to))*pow(tan(al),6))/8 + (7*pow(R,2)*pow(R0,4)*pow(tan(al),2))/2 + (11*pow(R,2)*pow(R0,4)*pow(tan(al),4))/2 + (13*pow(R,4)*pow(R0,2)*pow(tan(al),4))/8 + pow(R,2)*pow(R0,4)*pow(tan(al),6) + 3*pow(R,4)*pow(R0,2)*pow(tan(al),6) - (pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),2))/2 + (3*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),4))/2 + (3*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),4))/2 + 3*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),6) - (pow(R,4)*pow(R0,2)*cos(4*(t + to))*pow(tan(al),4))/8 + 4*R*pow(R0,5)*pow(tan(al),2)*cos(t + to) + 2*R*pow(R0,5)*pow(tan(al),4)*cos(t + to) + 3*pow(R,5)*R0*pow(tan(al),6)*cos(t + to) + pow(R,5)*R0*cos(3*(t + to))*pow(tan(al),6) + 8*pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos(t + to) + 4*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos(t + to)))/pow(R0,6))/pow(((pow(R,2)*((pow(R,2)*pow(tan(al),2))/2 + pow(R0,2)*pow(tan(al),2) + pow(R0,2) + (pow(R,2)*cos(2*(t + to))*pow(tan(al),2))/2 + 2*R*R0*pow(tan(al),2)*cos((t + to))))/pow(R0,2)),(3/2)); 

  // //      tau = ((- 2*pow(R,5)*R0*(cos(3*(t + to)) + 3*cos((t + to))) - 2*pow(R,4)*R0*(4*R0 + 4*R0*cos(2*(t + to))) - 8*pow(R,3)*pow(R0,3)*cos((t + to)))*pow(tan(al),5) + (8*R*pow(R0,5)*cos((t + to)) + 2*pow(R,3)*R0*(pow(R0,2)*cos(3*(t + to)) - 13*pow(R0,2)*cos((t + to))) - 16*pow(R,2)*pow(R0,4)*cos(2*(t + to)))*pow(tan(al),3) + (8*pow(R0,6) - 16*R*pow(R0,5)*cos((t + to)))*tan(al))/(R*(3*pow(R,6)*pow(tan(al),6) + 8*pow(R0,6)*pow(tan(al),2) + 8*pow(R0,6) + 4*pow(R,6)*cos(2*(t + to))*pow(tan(al),6) + pow(R,6)*cos(4*(t + to))*pow(tan(al),6) + 28*pow(R,2)*pow(R0,4)*pow(tan(al),2) + 44*pow(R,2)*pow(R0,4)*pow(tan(al),4) + 13*pow(R,4)*pow(R0,2)*pow(tan(al),4) + 8*pow(R,2)*pow(R0,4)*pow(tan(al),6) + 24*pow(R,4)*pow(R0,2)*pow(tan(al),6) - 4*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),2) + 12*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),4) + 12*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),4) + 24*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),6) - pow(R,4)*pow(R0,2)*cos(4*(t + to))*pow(tan(al),4) + 32*R*pow(R0,5)*pow(tan(al),2)*cos((t + to)) + 16*R*pow(R0,5)*pow(tan(al),4)*cos((t + to)) + 24*pow(R,5)*R0*pow(tan(al),6)*cos((t + to)) + 8*pow(R,5)*R0*cos(3*(t + to))*pow(tan(al),6) + 64*pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos((t + to)) + 32*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos((t + to))));


  // //      //psi = tau*dss;
  // //      ppsi =  ppsi + vs*0.01;
  // //	      
  // //      ROS_INFO(" t = %f", t);
  // //      ROS_INFO(" to = %f", to);
  // //      ROS_INFO(" pre_s = %f", pre_s);
  // //      ROS_INFO(" tau = %f", tau);
  // //      ROS_INFO(" kappa = %f", kappa);
  // //      ROS_INFO(" dss = %f", dss);
  // //      ROS_INFO(" vs = %f", vs);
  // //      ROS_INFO(" beta = %f", beta);
  // //      ROS_INFO(" ds = %f", ds);

  // //      ROS_INFO(" mode = %f", mode); 

  // //      // new_target_kappa_yaw    = (M_PI * serpenoid_curve.alpha / (2 * serpenoid_curve.l)) * sin(serpenoid_curve.alpha_s);
  // //      //   new_target_kappa_pitch  = 0;	 


  // //      ROS_INFO(" ppsi = %f", ppsi);
  // //      //	  ROS_INFO(" psi_old = %f", psi_old);
  // //      ROS_INFO(" R = %f", R);
  // //      new_target_kappa_pitch  = -kappa * sin(ppsi);
  // //      new_target_kappa_yaw    = kappa * cos(ppsi);

  // //      new_target_tau_pitch = tau * 0.0905;
  // //      new_target_tau_yaw = tau * 0.0905;

  // //      ROS_INFO(" new_target_kappa_pitch = %f", new_target_kappa_pitch);
  // //      ROS_INFO(" new_target_kappa_yaw = %f", new_target_kappa_yaw);
  // //      ROS_INFO(" new_target_tau_pitch = %f", new_target_tau_pitch);	      
  // //      ROS_INFO(" new_target_tau_yaw = %f", new_target_tau_yaw);

  // //      // // new_target_tau_pitch = 0;
  // //      // // new_target_tau_yaw = 0;

  // //      new_target_bias         = bias_;

  // //      //ROS_INFO(" new_target_kappa_yaw = %f", new_target_kappa_yaw);


  // //      /***  先頭デックの最初の要素に現在κを追加 (デックの長さ前より＋１になる) ***/
  // //      hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), new_target_kappa_pitch);
  // //      hold_data.shift_param[1].kappa_hold.insert(hold_data.shift_param[1].kappa_hold.begin(), new_target_kappa_yaw);
  // //      hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), new_target_tau_pitch);
  // //      hold_data.shift_param[1].tau_hold.insert(hold_data.shift_param[1].tau_hold.begin(), new_target_tau_yaw);
  // //      hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), 0);
  // //      hold_data.shift_param[1].psi_hold.insert(hold_data.shift_param[1].psi_hold.begin(), 0);
  // //      hold_data.shift_param[0].bias_hold.insert(hold_data.shift_param[0].bias_hold.begin(), 0);
  // //      hold_data.shift_param[1].bias_hold.insert(hold_data.shift_param[1].bias_hold.begin(), new_target_bias);

  // //      //ROS_INFO("**********  8 OK  *********");
  // //      int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();   // デックの大きさ, 前より＋１になった
  // //      //ROS_INFO(" **hold_data.shift_param[ %d ].kappa_hold is  %d", 0, hold_data.shift_param[0].kappa_hold.size());
  // //      //ROS_INFO(" **hold_data.shift_param[ %d ].kappa_hold is  %d", 1, hold_data.shift_param[1].kappa_hold.size());
  // //      //ROS_INFO(" hold_data.shift_param[ %d ].kappa_hold is  %f", 0, hold_data.shift_param[0].kappa_hold[0]);
  // //      //ROS_INFO(" hold_data.shift_param[ %d ].kappa_hold is  %f", 1, hold_data.shift_param[1].kappa_hold[0]);

  // //      //ROS_INFO("**********  9 OK  *********");
  // //      /***  後続ユニットの目標値をシフトしていれる  (１つ前のデックの最後のものを先頭に入れる)***/
  // //      for(int i=2; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  9.1  *********");
  // //	hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-2].kappa_hold[hold_num-1]);
  // //	//	            hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
  // //	hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i].bias_hold[hold_num-1]);
  // //	hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
  // //	//ROS_INFO(" ***hold_data.shift_param[ %d ].kappa_hold is  %f", i, hold_data.shift_param[i].kappa_hold[0]);
  // //      }

  // //      for(int i=1; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  9.1  *********");
  // //	// hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
  // //	hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
  // //	//    hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i].bias_hold[hold_num-1]);
  // //	//   hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
  // //	//ROS_INFO(" ***hold_data.shift_param[ %d ].kappa_hold is  %f", i, hold_data.shift_param[i].kappa_hold[0]);
  // //      }

  // //      /***  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  ***/
  // //      for(int i=0; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  9.2  *********");
  // //	snake_model_param.kappa.insert(snake_model_param.kappa.begin()+i, hold_data.shift_param[i].kappa_hold[hold_num-1]);
  // //	snake_model_param.tau.insert(snake_model_param.tau.begin()+i, hold_data.shift_param[i].tau_hold[hold_num-1]);
  // //	snake_model_param.bias.insert(snake_model_param.bias.begin()+i, hold_data.shift_param[i].bias_hold[hold_num-1]);
  // //	snake_model_param.psi.insert(snake_model_param.psi.begin()+i, hold_data.shift_param[i].psi_hold[hold_num-1]);

  // //	//ROS_INFO(" snake_model_param.kappa[ %d ] is  %f", i, snake_model_param.kappa[i]);

  // //	//snake_model_param.kappa[i] = hold_data.shift_param[i].kappa_hold[hold_num-1];
  // //	//snake_model_param.tau[i]   = hold_data.shift_param[i].tau_hold[hold_num-1];
  // //	//snake_model_param.bias[i]  = hold_data.shift_param[i].bias_hold[hold_num-1];
  // //	//snake_model_param.psi[i]   = hold_data.shift_param[i].psi_hold[hold_num-1];
  // //	// ROS_INFO("**********  9.3  *********");

  // //	snake_model_param.phi.insert(snake_model_param.phi.begin()+i, 0.0);

  // //	//snake_model_param.phi[i]   = 0;//2*M_PI/2;

  // //	hold_data.shift_param[i].kappa_hold.pop_back();
  // //	hold_data.shift_param[i].tau_hold.pop_back();
  // //	hold_data.shift_param[i].bias_hold.pop_back();
  // //	hold_data.shift_param[i].psi_hold.pop_back();

  // //	//ROS_INFO(" ****snake_model_param.kappa is %f", snake_model_param.kappa[i]);
  // //	//  ROS_INFO("**********  9.4  *********");
  // //      }
  // //      pre_s = pre_s + step;
  // //      //t = t + dth;


  // //      // ROS_INFO("**********  9.5  *********");
  // //      //////////////////////////////////////////////////ROS_INFO(" snake_model_param.kappa[ %d ] is  %f", j, snake_model_param.kappa[j]);
  // //      j++;

  // //    }




  // //    double target_angle = 0; // snake_model_param.bias[i] + 2 * spec.link_length_body() * snake_model_param.kappa[i]*sin(snake_model_param.tau[i] + snake_model_param.psi[i]);
  // //    snake_model_param.angle.resize(spec.num_joint());
  // //    ROS_INFO("**********  10  *********");
  // //    for(int i=0; i<spec.num_joint(); i++){
  // //      //(奇数番目)
  // //      if(i%2){
  // //	//target_angle = snake_model_param.bias[i] + 2 * spec.link_length_body() * snake_model_param.kappa[i]*sin(snake_model_param.tau[i] + snake_model_param.psi[i]);
  // //	target_angle = snake_model_param.kappa[i];

  // //	/////////////////////////////////////////////////ROS_INFO(" target_angle = %f", target_angle);
  // //	snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);

  // //	// snake_model_param.angle[i] = (snake_model_param.bias[i]
  // //	//				+ 2 * spec.link_length_body() * snake_model_param.kappa[i]*sin(snake_model_param.tau[i] + snake_model_param.psi[i]));
  // //		
  // //	snake_model_param.angle.pop_back();
  // //	/////////////////////////////////////////////ROS_INFO("snake_model_param.angle[ %d ] is  %f", i, snake_model_param.angle[i]);
  // //	//ROS_INFO("snake_model_param.angle.size is  %d", snake_model_param.angle.size());


  // //	//(偶数番目)
  // //      }else{
  // //	//target_angle = snake_model_param.bias[i] + 2 * spec.link_length_body() * snake_model_param.kappa[i]*cos(snake_model_param.tau[i] + snake_model_param.psi[i]);
  // //	target_angle = snake_model_param.kappa[i];
  // //	snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
  // //	/////////////////////////////////////////////ROS_INFO(" target_angle = %f", target_angle);
  // //	//snake_model_param.angle[i] =(snake_model_param.bias[i]
  // //	//						+ 2 * spec.link_length_body() * snake_model_param.kappa[i]*cos(snake_model_param.tau[i] + snake_model_param.psi[i]));

  // //	//ROS_INFO("snake_model_param.angle.size is  %f", i, snake_model_param.angle.size());
  // //	snake_model_param.angle.pop_back();
  // //	////////////////////////////////////////////ROS_INFO("snake_model_param.angle[ %d ] is  %f", i, snake_model_param.angle[i]);
  // //	//ROS_INFO("snake_model_param.angle.size is  %d", snake_model_param.angle.size());
  // //      }
  // //    }
  // //  }//mode

























  // //  if(mode==1){


  // //    while(s > (pre_s+step)){

  // //      //ROS_INFO("**********  7 OK  *********");

  // //      /***　先頭UNITのyaw,pitch 軸のkappa を計算する  ***/
  // //      //	      serpenoid_curve.alpha_s = (pre_s * M_PI / (2 * serpenoid_curve.l));

  // //      t = t + dth;
  // //      to = (vs*r*dt*sin(al))/R + beta;


  // //      kappa = sqrt((pow(R,4)*((3*pow(R,6)*pow(tan(al),6))/8 + pow(R0,6)*pow(tan(al),2) + pow(R0,6) + (pow(R,6)*cos(2*(t + to))*pow(tan(al),6))/2 + (pow(R,6)*cos(4*(t + to))*pow(tan(al),6))/8 + (7*pow(R,2)*pow(R0,4)*pow(tan(al),2))/2 + (11*pow(R,2)*pow(R0,4)*pow(tan(al),4))/2 + (13*pow(R,4)*pow(R0,2)*pow(tan(al),4))/8 + pow(R,2)*pow(R0,4)*pow(tan(al),6) + 3*pow(R,4)*pow(R0,2)*pow(tan(al),6) - (pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),2))/2 + (3*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),4))/2 + (3*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),4))/2 + 3*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),6) - (pow(R,4)*pow(R0,2)*cos(4*(t + to))*pow(tan(al),4))/8 + 4*R*pow(R0,5)*pow(tan(al),2)*cos(t + to) + 2*R*pow(R0,5)*pow(tan(al),4)*cos(t + to) + 3*pow(R,5)*R0*pow(tan(al),6)*cos(t + to) + pow(R,5)*R0*cos(3*(t + to))*pow(tan(al),6) + 8*pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos(t + to) + 4*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos(t + to)))/pow(R0,6))/pow(((pow(R,2)*((pow(R,2)*pow(tan(al),2))/2 + pow(R0,2)*pow(tan(al),2) + pow(R0,2) + (pow(R,2)*cos(2*(t + to))*pow(tan(al),2))/2 + 2*R*R0*pow(tan(al),2)*cos((t + to))))/pow(R0,2)),(3/2)); 

  // //      tau = ((- 2*pow(R,5)*R0*(cos(3*(t + to)) + 3*cos((t + to))) - 2*pow(R,4)*R0*(4*R0 + 4*R0*cos(2*(t + to))) - 8*pow(R,3)*pow(R0,3)*cos((t + to)))*pow(tan(al),5) + (8*R*pow(R0,5)*cos((t + to)) + 2*pow(R,3)*R0*(pow(R0,2)*cos(3*(t + to)) - 13*pow(R0,2)*cos((t + to))) - 16*pow(R,2)*pow(R0,4)*cos(2*(t + to)))*pow(tan(al),3) + (8*pow(R0,6) - 16*R*pow(R0,5)*cos((t + to)))*tan(al))/(R*(3*pow(R,6)*pow(tan(al),6) + 8*pow(R0,6)*pow(tan(al),2) + 8*pow(R0,6) + 4*pow(R,6)*cos(2*(t + to))*pow(tan(al),6) + pow(R,6)*cos(4*(t + to))*pow(tan(al),6) + 28*pow(R,2)*pow(R0,4)*pow(tan(al),2) + 44*pow(R,2)*pow(R0,4)*pow(tan(al),4) + 13*pow(R,4)*pow(R0,2)*pow(tan(al),4) + 8*pow(R,2)*pow(R0,4)*pow(tan(al),6) + 24*pow(R,4)*pow(R0,2)*pow(tan(al),6) - 4*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),2) + 12*pow(R,2)*pow(R0,4)*cos(2*(t + to))*pow(tan(al),4) + 12*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),4) + 24*pow(R,4)*pow(R0,2)*cos(2*(t + to))*pow(tan(al),6) - pow(R,4)*pow(R0,2)*cos(4*(t + to))*pow(tan(al),4) + 32*R*pow(R0,5)*pow(tan(al),2)*cos((t + to)) + 16*R*pow(R0,5)*pow(tan(al),4)*cos((t + to)) + 24*pow(R,5)*R0*pow(tan(al),6)*cos((t + to)) + 8*pow(R,5)*R0*cos(3*(t + to))*pow(tan(al),6) + 64*pow(R,3)*pow(R0,3)*pow(tan(al),4)*cos((t + to)) + 32*pow(R,3)*pow(R0,3)*pow(tan(al),6)*cos((t + to))));


  // //      //psi = tau*dss;
  // //      ppsi =  ppsi + vs*0.01;
  // //	      
  // //      ROS_INFO(" t = %f", t);
  // //      ROS_INFO(" to = %f", to);
  // //      ROS_INFO(" pre_s = %f", pre_s);
  // //      ROS_INFO(" tau = %f", tau);
  // //      ROS_INFO(" kappa = %f", kappa);
  // //      ROS_INFO(" dss = %f", dss);
  // //      ROS_INFO(" vs = %f", vs);
  // //      ROS_INFO(" beta = %f", beta);
  // //      ROS_INFO(" ds = %f", ds);

  // //      ROS_INFO(" mode = %f", mode); 

  // //      // new_target_kappa_yaw    = (M_PI * serpenoid_curve.alpha / (2 * serpenoid_curve.l)) * sin(serpenoid_curve.alpha_s);
  // //      //   new_target_kappa_pitch  = 0;	 


  // //      ROS_INFO(" ppsi = %f", ppsi);
  // //      //	  ROS_INFO(" psi_old = %f", psi_old);
  // //      ROS_INFO(" R = %f", R);
  // //      new_target_kappa_pitch  = -kappa * sin(ppsi);
  // //      new_target_kappa_yaw    = kappa * cos(ppsi);

  // //      new_target_tau_pitch = tau * 0.0905;
  // //      new_target_tau_yaw = tau * 0.0905;

  // //      ROS_INFO(" new_target_kappa_pitch = %f", new_target_kappa_pitch);
  // //      ROS_INFO(" new_target_kappa_yaw = %f", new_target_kappa_yaw);
  // //      ROS_INFO(" new_target_tau_pitch = %f", new_target_tau_pitch);	      
  // //      ROS_INFO(" new_target_tau_yaw = %f", new_target_tau_yaw);

  // //      // // new_target_tau_pitch = 0;
  // //      // // new_target_tau_yaw = 0;

  // //      new_target_bias         = bias_;

  // //      //ROS_INFO(" new_target_kappa_yaw = %f", new_target_kappa_yaw);


  // //      /***  先頭デックの最初の要素に現在κを追加 (デックの長さ前より＋１になる) ***/
  // //      hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), new_target_kappa_pitch);
  // //      hold_data.shift_param[1].kappa_hold.insert(hold_data.shift_param[1].kappa_hold.begin(), new_target_kappa_yaw);
  // //      hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), new_target_tau_pitch);
  // //      hold_data.shift_param[1].tau_hold.insert(hold_data.shift_param[1].tau_hold.begin(), new_target_tau_yaw);
  // //      hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), 0);
  // //      hold_data.shift_param[1].psi_hold.insert(hold_data.shift_param[1].psi_hold.begin(), 0);
  // //      hold_data.shift_param[0].bias_hold.insert(hold_data.shift_param[0].bias_hold.begin(), 0);
  // //      hold_data.shift_param[1].bias_hold.insert(hold_data.shift_param[1].bias_hold.begin(), new_target_bias);

  // //      //ROS_INFO("**********  8 OK  *********");
  // //      int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();    //デックの大きさ, 前より＋１になった


  // //      //ROS_INFO("**********  9 OK  *********");
  // //      /***  後続ユニットの目標値をシフトしていれる  (１つ前のデックの最後のものを先頭に入れる)***/
  // //      for(int i=2; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  9.1  *********");
  // //	hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-2].kappa_hold[hold_num-1]);
  // //	//hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
  // //	hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i].bias_hold[hold_num-1]);
  // //	hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);

  // //      }

  // //      for(int i=1; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  9.1  *********");
  // //	// hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
  // //	hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
  // //	//    hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i].bias_hold[hold_num-1]);
  // //	//   hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
  // //	//ROS_INFO(" ***hold_data.shift_param[ %d ].kappa_hold is  %f", i, hold_data.shift_param[i].kappa_hold[0]);
  // //      }


  // //      /***  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  ***/
  // //      for(int i=0; i<spec.num_joint(); i++){
  // //	//ROS_INFO("**********  9.2  *********");
  // //	snake_model_param.kappa.insert(snake_model_param.kappa.begin()+i, hold_data.shift_param[i].kappa_hold[hold_num-1]);
  // //	snake_model_param.tau.insert(snake_model_param.tau.begin()+i, hold_data.shift_param[i].tau_hold[hold_num-1]);
  // //	snake_model_param.bias.insert(snake_model_param.bias.begin()+i, hold_data.shift_param[i].bias_hold[hold_num-1]);
  // //	snake_model_param.psi.insert(snake_model_param.psi.begin()+i, hold_data.shift_param[i].psi_hold[hold_num-1]);

  // //	//ROS_INFO(" snake_model_param.kappa[ %d ] is  %f", i, snake_model_param.kappa[i]);

  // //	//snake_model_param.kappa[i] = hold_data.shift_param[i].kappa_hold[hold_num-1];
  // //	//snake_model_param.tau[i]   = hold_data.shift_param[i].tau_hold[hold_num-1];
  // //	//snake_model_param.bias[i]  = hold_data.shift_param[i].bias_hold[hold_num-1];
  // //	//snake_model_param.psi[i]   = hold_data.shift_param[i].psi_hold[hold_num-1];
  // //	// ROS_INFO("**********  9.3  *********");

  // //	snake_model_param.phi.insert(snake_model_param.phi.begin()+i, 0.0);

  // //	//snake_model_param.phi[i]   = 0;//2*M_PI/2;

  // //	hold_data.shift_param[i].kappa_hold.pop_back();
  // //	hold_data.shift_param[i].tau_hold.pop_back();
  // //	hold_data.shift_param[i].bias_hold.pop_back();
  // //	hold_data.shift_param[i].psi_hold.pop_back();

  // //	//ROS_INFO(" ****snake_model_param.kappa is %f", snake_model_param.kappa[i]);
  // //	//  ROS_INFO("**********  9.4  *********");
  // //      }
  // //      pre_s = pre_s + step;
  // //      //t = t + dth;


  // //      // ROS_INFO("**********  9.5  *********");
  // //      //////////////////////////////////////////////////ROS_INFO(" snake_model_param.kappa[ %d ] is  %f", j, snake_model_param.kappa[j]);
  // //      j++;

  // //    }




  // //    double target_angle = 0; 
  // //    snake_model_param.angle.resize(spec.num_joint());
  // //    ROS_INFO("**********  10  *********");
  // //    for(int i=0; i<spec.num_joint(); i++){
  // //      //(奇数番目)
  // //      if(i%2){
  // //	target_angle = -kappa * sin(i*dss+ppsi);
  // //	//	    target_angle = snake_model_param.kappa[i];
  // //	snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
  // //	snake_model_param.angle.pop_back();


  // //	//(偶数番目)
  // //      }else{

  // //	target_angle = kappa * cos(i*dss+ppsi);
  // //	//	    target_angle =snake_model_param.kappa[i];
  // //	snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);
  // //	snake_model_param.angle.pop_back();
  // //      }
  // //    }
  // //  }//mode

  // //	


















  // //	

}




void HelicalCurveGait::WindingShift() {

}



