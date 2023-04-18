#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <aux_lib/struct_defs.h>
#include <aux_lib/plot_functions.h>
#include <aux_lib/aux_functions.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char **argv) {

        ros::init(argc, argv, "plot_mpc_output");
        ros::NodeHandle nh;

        //define params_alg
        ai4ad::struct_params_alg params_alg_m;
        params_alg_m.dt = 0.1;
        params_alg_m.plan_t = 7;
        params_alg_m.np = uint16_t(std::round(params_alg_m.plan_t / params_alg_m.dt)) + 1;
        params_alg_m.total_np = 4 * params_alg_m.np - 6;
        params_alg_m.tvec =
                Eigen::VectorXd::LinSpaced(params_alg_m.np, 0.0, params_alg_m.plan_t);
        params_alg_m.pos_idxs =
                Eigen::VectorXi::LinSpaced(params_alg_m.np, 0, params_alg_m.np - 1);
        params_alg_m.vel_idxs = Eigen::VectorXi::LinSpaced(
                params_alg_m.np - 1, params_alg_m.np, 2 * params_alg_m.np - 2);
        params_alg_m.acc_idxs = Eigen::VectorXi::LinSpaced(
                params_alg_m.np - 2, 2 * params_alg_m.np - 1, 3 * params_alg_m.np - 4);
        params_alg_m.jrk_idxs = Eigen::VectorXi::LinSpaced(
                params_alg_m.np - 3, 3 * params_alg_m.np - 3, 4 * params_alg_m.np - 7); 
        params_alg_m.v_max = 5.0;
        params_alg_m.v_max_sbc = 5.0;
        params_alg_m.acc_max = 2;
        params_alg_m.acc_min = -2;
        
        //ADDED BY SHRITA - print algo params
        ROS_INFO("Algo params: ntotal %u", params_alg_m.np);
        ROS_INFO("Algo params: plan_t %f", params_alg_m.plan_t);
        
        //initialize current AD state
        ai4ad::AgentStateSimple cur_ad_state;
        ai4ad::StoplineBreak stopalgo_struct;
        stopalgo_struct.NearStopline = false;
        stopalgo_struct.Break = 0;
        stopalgo_struct.stopline_timer = 0;
        stopalgo_struct.max_iter = 25;
        stopalgo_struct.rate = 0.5;
        cur_ad_state.pos = 0; 
        cur_ad_state.vel = 5;
        cur_ad_state.accel = 3;
        
        //initialize vectors for plotting
        std::vector<double> ad_state_pos,ad_state_vel,ad_state_acc,ad_state_jrk, time;  
        int i = 0;     

        //initialize upper and lower position bounds
        Eigen::VectorXd lbpos = Eigen::VectorXd::Zero(params_alg_m.np);
        Eigen::VectorXd ubpos = Eigen::VectorXd::Constant(params_alg_m.np,400);
        bool plan_traj_isvalid = false;

        //initialize params for stopline
        double stopline_dist = 15; //initial dist from stopline 
        stopalgo_struct.stopline_dmin = pow(params_alg_m.v_max,2)/(2*params_alg_m.acc_max)+20;
        stopalgo_struct.stopline_amin = 0.4;
        stopalgo_struct.stopline_tmax = 2;
        stopalgo_struct.stopline_threshold = 0.5;
        
        //initialize qp problem
        // QPProblem* qp_problem_m = new QPProblem(params_alg_m);

        //run loop 
        ros::Rate rate(10);
        while (ros::ok()) {

                //update time
                time.push_back(i*params_alg_m.dt);

                ROS_INFO("Algo params: stopline dist %lf threshold distance %lf", 
                        stopline_dist, stopalgo_struct.stopline_dmin);
                ROS_INFO("Algo params: a_stopline %lf a_threshold %lf", 
                        pow(cur_ad_state.vel,2)/(2*stopline_dist), stopalgo_struct.stopline_amin);
                ROS_INFO("Algo params: cur vel %lf cur acc %lf", cur_ad_state.vel, cur_ad_state.accel);

                stopalgo_struct.a_init = -1;
                stopalgo_struct.j_init = -2;
                //define stop line condition - checkstopline
                CheckStopLine2(stopline_dist,cur_ad_state,stopalgo_struct,params_alg_m);
                ROS_INFO("Breaking action %d", stopalgo_struct.Break);

                //plot limits on velocity and position
                plt::figure(1);
                plt::clf();
                plt::subplot(3,1,1);
                std::vector<double> vtraj(stopalgo_struct.v_traj.data(), 
                        stopalgo_struct.v_traj.data()+stopalgo_struct.v_traj.size());
                plt::plot(vtraj,".-");
                plt::title("velocity");
                plt::subplot(3,1,2);
                std::vector<double> atraj(stopalgo_struct.a_traj.data(), 
                        stopalgo_struct.a_traj.data()+stopalgo_struct.a_traj.size());
                plt::plot(atraj,".-");
                plt::title("acceleration");
                plt::subplot(3,1,3);
                std::vector<double> jtraj(stopalgo_struct.j_traj.data(), 
                        stopalgo_struct.j_traj.data()+stopalgo_struct.j_traj.size());
                plt::plot(jtraj,".-");
                plt::title("jerk");
                plt::pause(0.2);
                // if (stopalgo_struct.Break==0){
                //         plt::show();
                // }
                // plt::show();

                //run dynamics
                cur_ad_state.accel = cur_ad_state.accel + stopalgo_struct.j_traj[0]*params_alg_m.dt; 
                // cur_ad_state.vel = cur_ad_state.vel + stopalgo_struct.a_traj[0]*params_alg_m.dt; 
                cur_ad_state.vel = cur_ad_state.vel + cur_ad_state.accel*params_alg_m.dt;
                cur_ad_state.pos = cur_ad_state.pos + cur_ad_state.vel*params_alg_m.dt;
                stopline_dist = stopline_dist - cur_ad_state.vel*params_alg_m.dt;

                //assign values for plotting
                ad_state_pos.push_back(cur_ad_state.pos);
                ad_state_vel.push_back(cur_ad_state.vel);
                ad_state_acc.push_back(cur_ad_state.accel);
                ad_state_jrk.push_back(stopalgo_struct.j_traj[0]);

                plt::figure(2);
                plt::clf();
                plt::subplot(2,2,1);
                plt::plot(time,ad_state_pos,".-");
                plt::title("position");
                plt::subplot(2,2,2);
                plt::plot(time,ad_state_vel,".-");
                plt::title("velocity");
                plt::subplot(2,2,3);                
                plt::plot(time,ad_state_acc,".-");
                plt::title("acceleration");
                plt::subplot(2,2,4);
                plt::plot(time,ad_state_jrk,".-");
                plt::title("jerk");
                plt::pause(0.2);
                // if (stopalgo_struct.Region!=1){
                //         plt::show();
                // }
                // plt::show();

                ros::spinOnce();
                rate.sleep();
                i++;
        }
        
};
