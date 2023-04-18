#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <aux_lib/struct_defs.h>
#include <aux_lib/plot_functions.h>
#include <aux_lib/aux_functions.h>
#include "OsqpEigen/OsqpEigen.h"
#include <aux_lib/qp_problem.h>
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
        params_alg_m.jrk_max = 50;
        params_alg_m.jrk_min = -50;
        params_alg_m.wa = 0.1;
        params_alg_m.wj = 20.0;
        params_alg_m.wf = 2.0;
        params_alg_m.match_accel = false;
        params_alg_m.use_prev_accel = true;
        params_alg_m.send_time_based_trajectory = true;
        
        //ADDED BY SHRITA - print algo params
        ROS_INFO("Algo params: ntotal %u", params_alg_m.np);
        ROS_INFO("Algo params: plan_t %f", params_alg_m.plan_t);
        ROS_INFO("Algo params: v_max %f v_max_sbc %f", 
                params_alg_m.v_max, params_alg_m.v_max_sbc);
        ROS_INFO("Algo params: acc_min %f acc_max %f", 
                params_alg_m.acc_min, params_alg_m.acc_max);
        ROS_INFO("Algo params: jrk_min %f jrk_max %f", 
                params_alg_m.jrk_min, params_alg_m.jrk_max);
        ROS_INFO("Algo params: pos size %ld", params_alg_m.pos_idxs.size());
        ROS_INFO("Algo params: vel size %ld", params_alg_m.vel_idxs.size());
        ROS_INFO("Algo params: acc size %ld", params_alg_m.acc_idxs.size());
        ROS_INFO("Algo params: jrk size %ld", params_alg_m.jrk_idxs.size());
        
        //initialize current AD state
        ai4ad::AgentStateSimple cur_ad_state;
        ai4ad::StoplineBreak stopalgo_struct;
        stopalgo_struct.NearStopline = false;
        stopalgo_struct.Break = 0;
        stopalgo_struct.stopline_timer = 0;
        stopalgo_struct.a_init = -1;
        stopalgo_struct.j_init = -1;
        stopalgo_struct.max_iter = 20;
        stopalgo_struct.rate = 0.5;
        cur_ad_state.pos = 0; 
        cur_ad_state.vel = 0;
        cur_ad_state.accel = 0;
        
        //initialize vectors for plotting
        std::vector<double> ad_state_pos,ad_state_vel,ad_state_acc,ad_state_jrk, time;  
        int i = 0;     

        //initialize upper and lower position bounds
        Eigen::VectorXd lbpos = Eigen::VectorXd::Zero(params_alg_m.np);
        Eigen::VectorXd ubpos = Eigen::VectorXd::Constant(params_alg_m.np,400);
        bool plan_traj_isvalid = false;
        
        //initialize params for stopline
        double stopline_dist = 15; //initial dist from stopline 
        stopalgo_struct.stopline_dmin = pow(params_alg_m.v_max,2)/(2*params_alg_m.acc_max)+10;
        stopalgo_struct.stopline_amin = 0.4;
        stopalgo_struct.stopline_tmax = 1;
        stopalgo_struct.stopline_threshold = 0.2;
        
        //initialize qp problem
        QPProblem* qp_problem_m = new QPProblem(params_alg_m);

        //run loop 
        ros::Rate rate(10);
        while (ros::ok()) {

                //update time
                time.push_back(i*params_alg_m.dt);

                ROS_INFO("Algo params: stopline dist %lf threshold distance %lf", 
                        stopline_dist, stopalgo_struct.stopline_dmin);
                ROS_INFO("Algo params: a_stopline %lf a_threshold %lf", 
                        pow(cur_ad_state.vel,2)/(2*stopline_dist), stopalgo_struct.stopline_amin);
                ROS_INFO("Algo params: t_stopline %lf t_threshold %lf", 
                        2*stopline_dist/cur_ad_state.vel, stopalgo_struct.stopline_tmax);
                ROS_INFO("Algo params: nstopline %u", 
                        uint16_t(std::round(2*stopline_dist/(cur_ad_state.vel*params_alg_m.dt))));
                ROS_INFO("Algo params: cur vel %lf", cur_ad_state.vel);

                //define stop line condition - checkstopline
                bool near_stopline = CheckStopLine(stopline_dist,cur_ad_state,
                        stopalgo_struct,params_alg_m);
                ROS_INFO("Breaking behaviour %d", near_stopline);

                plt::figure(1);
                plt::clf();
                std::vector<double> vmax(stopalgo_struct.vmax_stopline.data(), 
                        stopalgo_struct.vmax_stopline.data()+stopalgo_struct.vmax_stopline.size());
                plt::plot(vmax,".-");
                plt::title("velocity bounds");
                plt::pause(0.2);

                //update traj start point
                qp_problem_m->UpdateTrajectoryStartPoints(cur_ad_state);

                //solve qp problem
                ai4ad::Trajectory traj_qp;
                QPAuxStruct sqp_struct;
                sqp_struct.qp_var = std::make_unique<OsqpEigen::Solver>();
                plan_traj_isvalid = 
                        qp_problem_m->SolveQPProblem(lbpos,ubpos,sqp_struct,traj_qp,stopalgo_struct);
                ROS_INFO("size vtraj %ld",traj_qp.vel.size());
                stopalgo_struct.v_traj = traj_qp.vel;
                stopalgo_struct.a_traj = traj_qp.accel;
                stopalgo_struct.j_traj = traj_qp.jerk; 
                
                //run dynamics
                cur_ad_state.accel = cur_ad_state.accel + traj_qp.jerk[0]*params_alg_m.dt; 
                cur_ad_state.vel = cur_ad_state.vel + traj_qp.accel[0]*params_alg_m.dt;
                //cur_ad_state.vel = cur_ad_state.vel + cur_ad_state.accel*params_alg_m.dt;
                cur_ad_state.pos = cur_ad_state.pos + cur_ad_state.vel*params_alg_m.dt;
                stopline_dist = stopline_dist - cur_ad_state.vel*params_alg_m.dt;

                //plot limits on velocity and position
                plt::figure(2);
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
                // plt::show();
                plt::pause(0.2);

                //assign values for plotting
                ad_state_pos.push_back(cur_ad_state.pos);
                ad_state_vel.push_back(cur_ad_state.vel);
                ad_state_acc.push_back(cur_ad_state.accel);
                ad_state_jrk.push_back(stopalgo_struct.j_traj[0]);

                plt::figure(3);
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
                // plt::show();
                plt::pause(0.2);

                
                ros::spinOnce();
                rate.sleep();
                i++;
        }
        
};
