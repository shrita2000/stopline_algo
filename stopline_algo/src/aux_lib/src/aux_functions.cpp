/**
 * @file aux_functions.cpp
 * @author Alex Miranda A&ntilde;on (amiranda@honda-ri.com)
 * @brief Implementation of auxiliary functions for MPQP
 * @copyright Honda Research Institute, US (HRI-US)
This software cannot be distributed without the permission HRI-US
 */

#include <aux_lib/aux_functions.h>
#include <aux_lib/struct_defs.h>
#include <math.h>
#include <prediction_ct_vel/geometry_utils.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>

#include <unordered_set>

namespace ai4ad {

//////////////////////////// IDM Functions ////////////////////////////


/**
 * @details
 * Create a full trajectory for the planning horizon using %IDM model.
 *  1. If there are no cars we use a fix default front car distance and front
 * car velocity specified in the algorithm parameters
 *  2. If there is a front car we use that car for the %IDM formulation. The
 * preceding car flag needs to be set to true and preceding car trajectory need
 * to contain valid values for position and velocity over the time horizon
 *  3. If there is a AD car and we are cooperating so we may need to consider it
 * as front car, we do a comparison of the %IDM accelerations given between the
 * actual front and using AD car as front car. We pick the minimum acceleration
 * from the 2
 *
 */

Trajectory DriveCarIDM(const ai4ad::AgentStateSimple &ego_state,
                       const ai4ad::IDM &ego_idm,
                       const struct_params_alg &params_alg,
                       const bool coop_flag, const bool prec_car_flag,
                       const Trajectory &prec_car, const Trajectory &ad_car) {
  Trajectory traj_idm;
  traj_idm.pos.resize(params_alg.np);
  traj_idm.vel.resize(params_alg.np - 1);
  traj_idm.accel.resize(params_alg.np - 2);
  traj_idm.jerk.resize(params_alg.np - 3);

  // Copy current data
  traj_idm.pos[0] = ego_state.pos;
  traj_idm.vel[0] = ego_state.vel;
  traj_idm.accel[0] = ego_state.accel;

  // Verify valid trajectories for front car and AD car when needed
  if (prec_car_flag) {
    if (prec_car.pos.rows() < params_alg.np ||
        prec_car.vel.rows() < (params_alg.np - 1)) {
      ROS_ERROR("IDM called with invalid front car. Exiting");
      exit(1);
    }
  }
  if (coop_flag) {
    if (ad_car.pos.rows() < params_alg.np ||
        ad_car.vel.rows() < (params_alg.np - 1)) {
      ROS_ERROR("IDM called with invalid AD car. Exiting");
      exit(1);
    }
  }

  for (uint16_t t = 1; t < params_alg.np; t++) {
    double pos_old = traj_idm.pos[t - 1];
    double vel_old = traj_idm.vel[t - 1];

    // Accel
    if (t < params_alg.np - 2) {
      double pos_other, vel_other;
      if (prec_car_flag) {
        pos_other = prec_car.pos[t - 1];
        vel_other = prec_car.vel[t - 1];
      } else {
        pos_other = params_alg.dist_no_car;
        vel_other = params_alg.vel_no_car;
      }

      double idm_acc =
          AccelIDM(pos_old, vel_old, pos_other, vel_other, params_alg, ego_idm);
      if (coop_flag) {
        double pos_ad_old = ad_car.pos[t - 1];
        double vel_ad_old = ad_car.vel[t - 1];
        double idm_acc_ad = AccelIDM(pos_old, vel_old, pos_ad_old, vel_ad_old,
                                     params_alg, ego_idm);
        idm_acc = std::min(idm_acc, idm_acc_ad);
      }
      traj_idm.accel[t] = idm_acc;
    }

    // Vel
    if (t < params_alg.np - 1) {
      double acc_old = traj_idm.accel[t - 1];
      traj_idm.vel[t] = vel_old + acc_old * params_alg.dt;
      traj_idm.vel[t] = std::max(0.0, std::min(traj_idm.vel[t], ego_idm.v0));
    }

    // Pos
    traj_idm.pos[t] = pos_old + vel_old * params_alg.dt;

    // Jerk
    if (t < params_alg.np - 3) {
      double acc_old = traj_idm.accel[t - 1];
      traj_idm.jerk[t - 1] = (traj_idm.accel[t] - acc_old) / params_alg.dt;
    }
  }

  return traj_idm;
}

double AccelIDM(const double ego_s, const double ego_v,
                const double front_car_s, const double front_car_v,
                const struct_params_alg &params_alg,
                const ai4ad::IDM &ego_idm) {
  double delta_s = front_car_s - ego_s - params_alg.idm_offset;
  if (delta_s < 0) {
    return -ego_idm.brk_max;
  }
  double delta_v = ego_v - front_car_v;

  double s_star =
      ego_idm.s0 + ego_v * ego_idm.T0 +
      (ego_v * delta_v) / (2.0 * std::sqrt(ego_idm.acc_max * ego_idm.brk_max));

  double acc_estim_idm =
      ego_idm.acc_max *
      (1.0 - std::pow(ego_v / ego_idm.v0, params_alg.idm_delta) -
       std::pow(s_star / delta_s, 2));

  acc_estim_idm = std::max(-ego_idm.brk_max, acc_estim_idm);
  acc_estim_idm = std::min(ego_idm.acc_max, acc_estim_idm);

  return acc_estim_idm;
}

double AccelIDM(const double delta_s, const double ego_v,
                const double front_car_v, const struct_params_alg &params_alg,
                const ai4ad::IDM &ego_idm) {
  double delta_v = ego_v - front_car_v;

  double s_star =
      ego_idm.s0 + ego_v * ego_idm.T0 +
      (ego_v * delta_v) / (2.0 * std::sqrt(ego_idm.acc_max * ego_idm.brk_max));

  double acc_estim_idm =
      ego_idm.acc_max *
      (1.0 - std::pow(ego_v / ego_idm.v0, params_alg.idm_delta) -
       std::pow(s_star / delta_s, 2));

  acc_estim_idm = std::max(-ego_idm.brk_max, acc_estim_idm);
  acc_estim_idm = std::min(ego_idm.acc_max, acc_estim_idm);

  return acc_estim_idm;
}

//////////////////////////// Trajectory creation ////////////////////////////

Trajectory ConstantVel(const ai4ad::AgentStateSimple &ego_state,
                       const struct_params_alg &params_alg) {
  Trajectory traj_ct_vel;
  traj_ct_vel.pos.resize(params_alg.np);
  traj_ct_vel.vel.resize(params_alg.np - 1);
  traj_ct_vel.accel.resize(params_alg.np - 2);
  traj_ct_vel.jerk.resize(params_alg.np - 3);

  // Copy current data
  traj_ct_vel.pos[0] = ego_state.pos;
  double vel_init = ego_state.vel;
  traj_ct_vel.vel.setConstant(vel_init);
  traj_ct_vel.accel.setZero();
  traj_ct_vel.jerk.setZero();

  for (uint16_t t = 1; t < params_alg.np; t++) {
    double pos_old = traj_ct_vel.pos[t - 1];
    // Pos
    traj_ct_vel.pos[t] = pos_old + vel_init * params_alg.dt;
  }

  return traj_ct_vel;
}

/**
 * @details
 *
 * This function generates a very smooth three step braking from the current
 * values of velocity and acceleration to zero or the lowest speed possible in
 * the planning horizon. These are the 3 steps:
 *  1. Constant minimum jerk from the current acceleration to the minimum
 * acceleration
 *  2. Constant minimum acceleration
 *  3. Constant maximum jerk from minimum acceleration to zero acceleration and
 * zero or minimum speed achievable
 *
 * These are the cases where the formulation may skip some of the 3 steps:
 *  - We are already stopped. We send a vector zero speed and zero acceleration
 *  - With minimum jerk we cannot achieve minimum acceleration, we send only
 * minimum jerk trajectory
 *  - Other cases like already minimum acceleration are handled by having the
 * duration of that step last 0 seconds
 *  - With a minimum jerk to minimum acceleration plus a maximum jerk to zero
 * acceleration we already achieve zero speed or lower
 *    - We do not need a constant minimum acceleration step
 *    - If we go below zero speed, we won't be needing to achieve minimum
 * acceleration so we need to find an inflexion point from minimum jerk to
 * maximum jerk. This part requires solving a quadratic equation that may yield
 * no solutions
 */
bool ThreeStepBraking(const double &a0, const double &v0,
                      const struct_params_alg &params_alg,
                      Trajectory &traj_output) {
  // Init variable thar works for already stopped
  traj_output.accel = Eigen::VectorXd::Zero(params_alg.np - 2);
  traj_output.vel = Eigen::VectorXd::Zero(params_alg.np - 1);
  if (v0 <= 1e-03) {
    return true;
  } else {
    double t_minj = std::sqrt(-2.0 * v0 / params_alg.jrk_min);
    if (t_minj > params_alg.plan_t) {
      // Full minimum jerk
      traj_output.accel = a0 + (params_alg.tvec * params_alg.jrk_min).array();
      Eigen::VectorXd aux_vel_init;
      aux_vel_init << traj_output.vel(0), traj_output.accel * params_alg.dt;
      Eigen::VectorXd aux_vel = ai4ad::CumSum(aux_vel_init);
      aux_vel = aux_vel.cwiseMax(0.0);
      traj_output.vel(Eigen::seq(1, Eigen::placeholders::last)) =
          aux_vel(Eigen::seq(1, Eigen::placeholders::last));
      return true;

    } else {
      double t1 = (params_alg.acc_min - a0) / params_alg.jrk_min;
      double v1 =
          v0 + a0 * t1 + 0.5 * params_alg.jrk_min * t1 * t1;  // Debug var
      double delta_t2t3 = -params_alg.acc_min / params_alg.jrk_max;
      double v_t0t3 = v0 + a0 * t1 + 0.5 * params_alg.jrk_min * t1 * t1 +
                      params_alg.acc_min * delta_t2t3 +
                      0.5 * params_alg.jrk_max * delta_t2t3 * delta_t2t3;
      if (v_t0t3 >= 1e-03) {
        // 3 steps at min jerk (until min accel), 0 jerk (ct min accel) and
        // then max jerk (until 0 accel)
        uint16_t ind_t1 =
            uint16_t(std::ceil(t1 * (params_alg.np - 1) / params_alg.plan_t));
        Eigen::VectorXd tvec_part1 = params_alg.tvec(Eigen::seq(0, ind_t1));
        double extra_t1 = double(ind_t1) * params_alg.dt - t1;
        traj_output.accel(Eigen::seq(0, ind_t1)) =
            a0 + (tvec_part1 * params_alg.jrk_min).array();
        traj_output.vel(Eigen::seq(0, ind_t1)) =
            v0 + (tvec_part1 * a0).array() +
            0.5 * tvec_part1.array().square() * params_alg.jrk_min;

        double delta_t1t2 = -v_t0t3 / params_alg.acc_min;
        uint16_t ind_t3;

        if (extra_t1 + delta_t1t2 <= params_alg.dt) {
          traj_output.accel(ind_t1 + 1) =
              params_alg.acc_min +
              (params_alg.dt - extra_t1 + delta_t1t2) * params_alg.jrk_max;
          traj_output.vel(ind_t1 + 1) =
              v1 +
              params_alg.acc_min * (params_alg.dt - extra_t1 + delta_t1t2) +
              0.5 * (params_alg.dt - extra_t1 + delta_t1t2) *
                  params_alg.jrk_max;
          ind_t3 = uint16_t(ceil((t1 + delta_t1t2 + delta_t2t3) *
                                 (params_alg.np - 1) / params_alg.plan_t));
          Eigen::VectorXd tvec_part3 =
              params_alg.tvec(Eigen::seq(0, ind_t3 - ind_t1));
          traj_output.accel(Eigen::seq(ind_t1 + 1, ind_t3)) =
              traj_output.accel(ind_t1 + 1) +
              (tvec_part3 * params_alg.jrk_max).array();
          traj_output.vel(Eigen::seq(ind_t1 + 1, ind_t3)) =
              traj_output.vel(ind_t1 + 1) +
              (traj_output.accel(ind_t1 + 1) * tvec_part3).array() +
              0.5 * tvec_part3.array().square() * params_alg.jrk_max;
        } else {
          traj_output.accel(ind_t1 + 1) = params_alg.acc_min;
          traj_output.vel(ind_t1 + 1) = v1 + params_alg.acc_min * extra_t1;
          uint16_t ind_t2 = uint16_t(std::ceil(
              (t1 + delta_t1t2) * (params_alg.np - 1) / params_alg.plan_t));
          Eigen::VectorXd tvec_part2 =
              params_alg.tvec(Eigen::seq(0, ind_t2 - ind_t1));
          double v2 = v1 + params_alg.acc_min * delta_t1t2;
          traj_output.accel(Eigen::seq(ind_t1 + 1, ind_t2)) =
              Eigen::VectorXd::Constant(ind_t2 - (ind_t1 + 1),
                                        params_alg.acc_min);
          traj_output.vel(Eigen::seq(ind_t1 + 1, ind_t2)) =
              traj_output.vel(ind_t1 + 1) +
              (tvec_part2 * params_alg.acc_min).array();
          double extra_t2 = ind_t2 * params_alg.dt - (t1 + delta_t1t2);
          traj_output.accel(ind_t2 + 1) =
              params_alg.acc_min + extra_t2 * params_alg.jrk_max;
          traj_output.vel(ind_t2 + 1) =
              v2 + params_alg.acc_min * extra_t2 +
              0.5 * extra_t2 * extra_t2 * params_alg.jrk_max;
          ind_t3 = uint16_t(std::ceil((t1 + delta_t1t2 + delta_t2t3) *
                                      (params_alg.np - 1) / params_alg.plan_t));
          Eigen::VectorXd tvec_part3 =
              params_alg.tvec(Eigen::seq(0, ind_t3 - ind_t2));
          double v3 =
              v2 + delta_t2t3 * params_alg.acc_min +
              0.5 * delta_t2t3 * delta_t2t3 * params_alg.jrk_max;  // Debug var
          traj_output.accel(Eigen::seq(ind_t2 + 1, ind_t3)) =
              traj_output.accel(ind_t2 + 1) +
              (tvec_part3 * params_alg.jrk_max).array();
          traj_output.vel(Eigen::seq(ind_t2 + 1, ind_t3)) =
              traj_output.vel(ind_t2 + 1) +
              (traj_output.accel(ind_t2 + 1) * tvec_part3).array() +
              0.5 * tvec_part3.array().square() * params_alg.jrk_max;
        }

        traj_output.accel(Eigen::seq(ind_t3 + 1, Eigen::placeholders::last)) =
            Eigen::VectorXd::Zero(traj_output.accel.size() - (ind_t3 + 1));
        traj_output.vel(Eigen::seq(ind_t3 + 1, Eigen::placeholders::last)) =
            Eigen::VectorXd::Zero(traj_output.vel.size() - (ind_t3 + 1));

        return true;

      } else {
        // Need to compute an inflection from max jerk to min jerk
        double a = 0.5 * params_alg.jrk_min - params_alg.jrk_min *
                                                  params_alg.jrk_min /
                                                  (2.0 * params_alg.jrk_max);
        double b = a0 - a0 * params_alg.jrk_min / params_alg.jrk_max;
        double c = v0 - a0 * a0 / (2 * params_alg.jrk_max);
        if (b > -1e-7 && b < 1e-7) {
          b = 0;
        }
        if (c > -1e-7 && c < 1e-7) {
          c = 0;
        }
        double t1_sol_1 = -b + std::sqrt(b * b - 4.0 * a * c) / (2.0 * a);
        double t1_sol_2 = -b - std::sqrt(b * b - 4.0 * a * c) / (2.0 * a);

        // At least one positive solution
        bool t1_sol_1_valid = t1_sol_1 > 0.0 && t1_sol_1 <= params_alg.plan_t;
        bool t1_sol_2_valid = t1_sol_2 > 0.0 && t1_sol_2 <= params_alg.plan_t;

        double t1;
        if (t1_sol_1_valid && !t1_sol_2_valid) {
          t1 = t1_sol_1;
        } else if (!t1_sol_1_valid && t1_sol_2_valid) {
          t1 = t1_sol_2;
        } else if (!t1_sol_1_valid && !t1_sol_2_valid) {
          return false;
        } else if (t1_sol_1_valid && t1_sol_2_valid) {
          ROS_ERROR("Two solutions in 3 step braking");
          t1 = t1_sol_2;
        } else {
          ROS_FATAL("Impossibe case in 3 step braking. Exiting");
          exit(0);
        }

        // Min jerk till t1 and then max jerk then 0 accel (if needed)
        double a1 = a0 + t1 * params_alg.jrk_min;
        double v1 =
            v0 + a0 * t1 + 0.5 * params_alg.jrk_min * t1 * t1;  // Debug var
        uint16_t ind_t1 =
            uint16_t(std::ceil(t1 * (params_alg.np - 1) / params_alg.plan_t));
        Eigen::VectorXd tvec_part1 = params_alg.tvec(Eigen::seq(0, ind_t1));
        double extra_t1 = ind_t1 * params_alg.dt - t1;
        traj_output.accel(Eigen::seq(0, ind_t1)) =
            a0 + (tvec_part1 * params_alg.jrk_min).array();
        traj_output.accel(ind_t1 + 1) = a1 + extra_t1 * params_alg.jrk_max;
        traj_output.vel(Eigen::seq(0, ind_t1)) =
            v0 + (a0 * tvec_part1).array() +
            0.5 * tvec_part1.array().square() * params_alg.jrk_min;
        traj_output.vel(ind_t1 + 1) =
            v1 + a1 * extra_t1 + 0.5 * params_alg.jrk_max * extra_t1;

        double delta_t1t2 = -a1 / params_alg.jrk_max;
        double a2 = a1 + delta_t1t2 * params_alg.jrk_max;  // Debug var
        double t2 = t1 + delta_t1t2;                       // Debug var
        uint16_t ind_t2 = std::min(
            uint16_t(std::ceil((t1 + delta_t1t2) * (params_alg.np - 1) /
                               params_alg.plan_t)),
            uint16_t(params_alg.acc_idxs.size()));
        Eigen::VectorXd tvec_part2 =
            params_alg.tvec(Eigen::seq(0, ind_t2 - ind_t1));
        traj_output.accel(Eigen::seq(ind_t1 + 1, ind_t2)) =
            traj_output.accel(ind_t1 + 1) +
            (tvec_part2 * params_alg.jrk_max).array();
        traj_output.accel(Eigen::seq(ind_t2 + 1, Eigen::placeholders::last)) =
            Eigen::VectorXd::Zero(traj_output.accel.size() - (ind_t2 + 1));
        traj_output.vel(Eigen::seq(ind_t1 + 1, ind_t2)) =
            traj_output.vel(ind_t1 + 1) +
            (traj_output.accel(ind_t1 + 1) * tvec_part2).array() +
            0.5 * params_alg.jrk_max * tvec_part2.array().square();
        traj_output.vel(Eigen::seq(ind_t2 + 1, Eigen::placeholders::last)) =
            Eigen::VectorXd::Zero(traj_output.vel.size() - (ind_t2 + 1));

        double v2 =
            v1 + delta_t1t2 * a1 +
            0.5 * delta_t1t2 * delta_t1t2 * params_alg.jrk_max;  // Debug var

        return true;
      }
    }
  }
}

/**
 * @details
 *
 * Unless we use the 3 step braking flag these will be braking trajectory will
 * be divided in two sections:
 *  1. Minimum jerk trajectory to minimum acceleration (optional if we don't
 * match acceleration or we match but we are already at minimum acceleration)
 *  2. Constant minimum acceleration to zero or minimum speed achievable in the
 * planning horizon. After speed 0 is achieved we send 0 acceleration directly
 * without smoothly going to zero acceleration as in the 3 step braking
 */
bool BrakingBehavior(const ai4ad::AgentStateSimple &state_input,
                     const ai4ad::struct_params_alg &params_alg,
                     const bool &three_step_braking,
                     ai4ad::Trajectory &traj_output) {
  traj_output.pos = Eigen::VectorXd::Zero(params_alg.np);
  traj_output.vel = Eigen::VectorXd::Zero(params_alg.np - 1);
  traj_output.accel = Eigen::VectorXd::Zero(params_alg.np - 2);

  double acc_min = params_alg.acc_min;
  double jerk_min = params_alg.jrk_min;

  traj_output.pos(0) = state_input.pos;
  traj_output.vel(0) = state_input.vel;
  if (params_alg.match_accel || params_alg.use_prev_accel) {
    traj_output.accel(0) = state_input.accel;
  } else {
    traj_output.accel(0) = acc_min;
  }

  if (three_step_braking) {
    bool is_solution = ai4ad::ThreeStepBraking(
        traj_output.accel(0), traj_output.vel(0), params_alg, traj_output);
    return is_solution;
  }

  Eigen::VectorXd aux_vel_init = Eigen::VectorXd::Zero(params_alg.np - 1);

  // Acceleration
  traj_output.accel = traj_output.accel(0) +
                      jerk_min * params_alg
                                     .tvec(params_alg.pos_idxs(Eigen::seq(
                                         0, Eigen::placeholders::last - 2)))
                                     .array();
  traj_output.accel = traj_output.accel.cwiseMax(acc_min);

  // Velocity
  aux_vel_init << traj_output.vel(0), traj_output.accel * params_alg.dt;
  Eigen::VectorXd aux_vel = ai4ad::CumSum(aux_vel_init);
  std::vector<uint16_t> ind_clamp;
  for (uint16_t i = 1; i < aux_vel.size(); i++) {
    if (aux_vel(i) < 0.0) {
      if (i < aux_vel.size() - 1) {
        ind_clamp.push_back(i);
      }
      aux_vel(i) = 0.0;
    }
  }

  traj_output.vel(Eigen::seq(1, Eigen::placeholders::last)) =
      aux_vel(Eigen::seq(1, Eigen::placeholders::last));
  traj_output.accel(ind_clamp) =
      Eigen::VectorXd::Zero(uint16_t(ind_clamp.size()));

  // Position
  Eigen::VectorXd aux_pos_init = Eigen::VectorXd::Zero(params_alg.np);
  aux_pos_init << traj_output.pos(0), traj_output.vel * params_alg.dt;
  Eigen::VectorXd aux_pos = ai4ad::CumSum(aux_pos_init);

  traj_output.pos(Eigen::seq(1, Eigen::placeholders::last)) =
      aux_pos(Eigen::seq(1, Eigen::placeholders::last));

  return true;
}

/**
 * @details
 * Accelerate to max speed as specified in the algorithm parameters:
 *  1. Maximum jerk trajectory to maximum acceleration (optional if we don't
 * match acceleration or we match but we are already at maximum acceleration)
 *  2. Constant maximum acceleration to max speed or maximum speed achievable in
 * the planning horizon. After arriving to max speed send 0 acceleration
 * directly
 */
void AccelerationBehavior(const ai4ad::AgentStateSimple &state_input,
                          const ai4ad::struct_params_alg &params_alg,
                          ai4ad::Trajectory &traj_output) {
  Eigen::VectorXd pos_max_vec;

  traj_output.pos = Eigen::VectorXd::Zero(params_alg.np);
  traj_output.vel = Eigen::VectorXd::Zero(params_alg.np - 1);
  traj_output.accel = Eigen::VectorXd::Zero(params_alg.np - 2);

  traj_output.pos(0) = state_input.pos;
  traj_output.vel(0) = state_input.vel;
  if (params_alg.match_accel || params_alg.use_prev_accel) {
    traj_output.accel(0) = state_input.accel;
  } else {
    traj_output.accel(0) = params_alg.acc_max;
  }
  // Acceleration
  traj_output.accel =
      traj_output.accel(0) +
      params_alg.jrk_min *
          params_alg.tvec(Eigen::seq(0, Eigen::placeholders::last - 2)).array();
  traj_output.accel = traj_output.accel.cwiseMax(params_alg.acc_max);

  // Velocity
  Eigen::VectorXd aux_vel_init = Eigen::VectorXd::Zero(params_alg.np - 1);
  aux_vel_init << traj_output.vel(0), traj_output.accel * params_alg.dt;
  Eigen::VectorXd aux_vel = ai4ad::CumSum(aux_vel_init);
  std::vector<uint16_t> ind_clamp;
  for (uint16_t i = 1; i < aux_vel.size(); i++) {
    if (aux_vel(i) > params_alg.v_max) {
      if (i < aux_vel.size() - 1) {
        ind_clamp.push_back(i);
      }
      aux_vel(i) = params_alg.v_max;
    }
  }

  traj_output.vel(Eigen::seq(1, Eigen::placeholders::last)) =
      aux_vel(Eigen::seq(1, Eigen::placeholders::last));
  traj_output.accel(ind_clamp) =
      Eigen::VectorXd::Zero(uint16_t(ind_clamp.size()));

  // Position
  Eigen::VectorXd aux_pos_init = Eigen::VectorXd::Zero(params_alg.np);
  aux_pos_init << traj_output.pos(0), traj_output.vel * params_alg.dt;
  Eigen::VectorXd aux_pos = ai4ad::CumSum(aux_pos_init);
  traj_output.pos(Eigen::seq(1, Eigen::placeholders::last)) =
      aux_pos(Eigen::seq(1, Eigen::placeholders::last));
}

/**
   * @brief Update mpc parameters to ensure breaking at stopline
   * - added by shrita
   */
  bool CheckStopLine(double stopline_dist,
                    const ai4ad::AgentStateSimple &cur_ad_state, ai4ad::StoplineBreak 
                    &stopalgo_struct, const ai4ad::struct_params_alg &params_alg) {
    
    //initialize vmax, astop, tstop, nstop 
    stopalgo_struct.vmax_stopline = Eigen::VectorXd::Constant(params_alg.np-1,0.05);
    double v_init = cur_ad_state.vel + cur_ad_state.accel*params_alg.dt + 0.1;
    double t_stopline = 2*stopline_dist/v_init;
    double a_stopline = pow(v_init,2)/(2*stopline_dist);
    uint16_t n_stopline = uint16_t(std::round(t_stopline/params_alg.dt));
    //calculate vmax_stopline bounds
    double t_curr = 0;
    //asign strictly decreasing values to vmax
    stopalgo_struct.vmax_stopline(0)=cur_ad_state.vel;
    stopalgo_struct.vmax_stopline(1)=v_init;
    for(int i=2; i<stopalgo_struct.vmax_stopline.size(); i++){
      if (i<n_stopline+2){
        stopalgo_struct.vmax_stopline(i) = v_init - a_stopline*t_curr;
        t_curr += params_alg.dt;  
    }}
    
    //update stopline_dist_m for imposing pos bounds
    stopalgo_struct.stopline_dist_m = stopline_dist; 
    
    //if in stopline region but not at the line
    if (stopline_dist<stopalgo_struct.stopline_dmin 
      && stopline_dist>stopalgo_struct.stopline_threshold){
      //if astopline becomes very very low, temporarily accelerate
      //after acceleration once threshold value is crossed, resume stopping
      if (stopalgo_struct.NearStopline==false && a_stopline>stopalgo_struct.stopline_amin){
        stopalgo_struct.NearStopline=true;
      }else if (stopalgo_struct.NearStopline==true && a_stopline<0.01){
        stopalgo_struct.NearStopline=false;
      }
    
    //if at stopline, wait for tmax seconds
    }else if (stopline_dist<stopalgo_struct.stopline_threshold && stopline_dist>0 
        && stopalgo_struct.stopline_timer<stopalgo_struct.stopline_tmax*10){
      //cant assign vmax as zero since we will get an infeasible soln
      stopalgo_struct.NearStopline = true;
      stopalgo_struct.stopline_timer++;
      ROS_INFO("Stop Timer %u", stopalgo_struct.stopline_timer);
    
    //otherwise, proceed as usual  
    }else{
      stopalgo_struct.NearStopline = false;
      stopalgo_struct.vmax_stopline = Eigen::VectorXd::Zero(params_alg.np-1);
    }
    return stopalgo_struct.NearStopline;
  }

/**
   * @brief Update trajectory generated for stopline breaking using iterations
   * - added by shrita
   */
  void CheckStopLine2(double stopline_dist,
                    const ai4ad::AgentStateSimple &cur_ad_state, ai4ad::StoplineBreak 
                    &stopalgo_struct, const ai4ad::struct_params_alg &params_alg) {
    
    //initialize astop 
    double a_stopline = pow(cur_ad_state.vel,2)/(2*stopline_dist); 
    
    if(stopline_dist<stopalgo_struct.stopline_threshold && cur_ad_state.vel<0.01){
      stopalgo_struct.Break=0;
    }else{stopalgo_struct.Break=1;}

    if(stopalgo_struct.Break==1){
      //calculate vmax_stopline bounds
      int n_i = stopalgo_struct.max_iter;
      Eigen::VectorXd r = Eigen::VectorXd::Zero(n_i);
      Eigen::VectorXd atgt = Eigen::VectorXd::Zero(n_i);
      Eigen::VectorXd jtgt = Eigen::VectorXd::Zero(n_i);
      Eigen::VectorXd dtgt = Eigen::VectorXd::Zero(n_i);
      r(0) = 1;
      atgt(0) = stopalgo_struct.a_init;
      jtgt(0) = stopalgo_struct.j_init;
      stopalgo_struct.case_no = 0;
      dtgt(0) = CalcDist(atgt(0), jtgt(0), cur_ad_state.vel, cur_ad_state.accel, 
        stopalgo_struct);
      int i = 0;
      double err = 1;

      while (i<n_i-1 && err>0.01 && stopalgo_struct.case_no==0){
      //iterate to get best target acc, jerk using ratio of planned
      //distance travelled dtgt, distance of stopline from car stopline_dist
      r(i+1)=r(i)*pow((dtgt(i)/stopline_dist),stopalgo_struct.rate);
      atgt(i+1) = atgt(0)*r(i+1);
      jtgt(i+1) = jtgt(0)*r(i+1);
      dtgt(i+1) = CalcDist(atgt(i+1), jtgt(i+1), cur_ad_state.vel, cur_ad_state.accel, 
        stopalgo_struct);
      Eigen::Vector3d err_v(atgt(i+1)-atgt(i),jtgt(i+1)-jtgt(i),dtgt(i+1)-stopline_dist);
      err = err_v.norm();
      i=i+1;
      if (stopalgo_struct.case_no>0){
        break;
      }
      }
      ROS_INFO("Iteration no %u",i);
      ROS_INFO("Ratio value %lf",r(i));
      ROS_INFO("Atgt %lf Jtgt %lf",atgt(i),jtgt(i));
      stopalgo_struct.vmax_stopline = CalcTraj(atgt(i), jtgt(i), cur_ad_state.vel, 
          cur_ad_state.accel, params_alg.dt, params_alg.np-1, stopalgo_struct);
      
      //update stopline_dist_m for imposing pos bounds
      stopalgo_struct.stopline_dist_m = stopline_dist;
    }else{
      stopalgo_struct.v_traj = Eigen::VectorXd::Zero(params_alg.np-1);
      stopalgo_struct.a_traj = Eigen::VectorXd::Zero(params_alg.np-1);
      stopalgo_struct.j_traj = Eigen::VectorXd::Zero(params_alg.np-1);
    }
  }

  double CalcDist(double atgt, double jtgt, double v_curr, double a_curr, ai4ad::StoplineBreak 
                    &stopalgo_struct){
    
    //time instants at which j changes, corresponding velocity
    double t1 = (atgt-a_curr)/jtgt;
    double j0 = jtgt;
    if (atgt>a_curr && jtgt<0){
      t1 = -(atgt-a_curr)/jtgt;
      j0 = -jtgt;
    }
    if (t1<0.1){
      t1=0;
    }
    double v1 = v_curr + a_curr*t1 + 0.5*j0*pow(t1,2);
    double t3 = atgt/jtgt;
    double v2 = 0.5*jtgt*pow(t3,2) - atgt*t3;
    double t2 = (v2-v1)/atgt; 
    double dist; 
    
    //There is a case that t2 is negative. In that case, there is no constant acceleration section.
    if (t2<0){  
        if (atgt>a_curr || t1==0){
          //constant positive jerk case
          stopalgo_struct.case_no = 1;
        }else{
          //negative jerk and then positive jerk
          stopalgo_struct.case_no = 2;
        }
        t2 = 0;
    }else{ 
      //negative jerk, zero jerk, positive jerk
      stopalgo_struct.case_no = 0;
    }

    if (stopalgo_struct.case_no==0){
       
      double p1 = v_curr*t1 + a_curr*pow(t1,2)/2 + j0*pow(t1,3)/6;
      double p2 = v1*t2 + atgt*pow(t2,2)/2;
      double p3 = v2*t3 + atgt*pow(t3,2)/2 - jtgt*pow(t3,3)/6;
      dist = p1+p2+p3;
       
    }else if (stopalgo_struct.case_no==1){
           
      //time and jerk
      double tf = abs(2*v_curr/a_curr);
      double jt = abs(pow(a_curr,2)/(2*v_curr));
      if(jt>5){jt=5;}
       
      dist = v_curr*tf + a_curr*pow(tf,2)/2 + jt*pow(tf,3)/6;
       
    }else{ 
       
      //jerk and t1, t3 
      double j0 = (2*pow(atgt,2)-pow(a_curr,2))/(2*v_curr);
      t1 = (a_curr-atgt)/j0;
      if (t1<0.1){
        t1=0;
      }
      if(j0>5){j0=5;}
      t2=0;
      t3 = -atgt/j0;
      t1 = t1*((2*pow(atgt,2)-pow(a_curr,2))/(2*v_curr))/j0;
       
      v1 = v_curr + a_curr*t1 - 0.5*j0*pow(t1,2);
      v2 = -atgt*t3 -0.5*j0*pow(t3,2);

      double p1 = v_curr*t1 + a_curr*pow(t1,2)/2 - j0*pow(t1,3)/6;
      double p2 = 0;
      double p3 = v2*t3 + atgt*pow(t3,2)/2 + j0*pow(t3,3)/6;   
       
      dist = p1+p2+p3;      
    } 

    return dist;

  }

   Eigen::VectorXd CalcTraj(double atgt, double jtgt, double v_curr, 
        double a_curr, float dt, int np, ai4ad::StoplineBreak 
                    &stopalgo_struct){
    //Calculates trajectory for a given target acc and jerk
   //Input: sampling interval dt, current velocity v_curr, current
   //acceleration a_curr, target values agt, jtgt,
   //time instants at which j changes
    double t1 = (atgt-a_curr)/jtgt;
    double j0 = jtgt;
    if (atgt>a_curr && jtgt<0){
      t1 = -(atgt-a_curr)/jtgt;
      j0 = -jtgt;
    }
    if (t1<0.1){
      t1=0;
    }
    double v1 = v_curr + a_curr*t1 + 0.5*j0*pow(t1,2);
    
    double t3 = atgt/jtgt;
    double v2 = 0.5*jtgt*pow(t3,2) - atgt*t3;
    
    double t2 = (v2-v1)/atgt; 
    
    if (t2<0){  
        if ((atgt>a_curr) || t1==0){
          stopalgo_struct.case_no = 1;
        }else{
          stopalgo_struct.case_no = 2;
        }
        t2 = 0;
    }else{ 
      stopalgo_struct.case_no = 0;
    }

    //initialize state
    Eigen::VectorXd v_iter = Eigen::VectorXd::Zero(np);
    Eigen::VectorXd a_iter = Eigen::VectorXd::Zero(np);
    Eigen::VectorXd j_iter = Eigen::VectorXd::Zero(np);

    if (stopalgo_struct.case_no==0){

      ROS_INFO("Case no %u",stopalgo_struct.case_no);
       
      double totalt = t1+t2+t3;
    
      ROS_INFO("t1 %lf v1 %lf j0 %lf",t1,v1,j0);
      ROS_INFO("t2 %lf v2 %lf",t2,v2);
      ROS_INFO("t3 %lf",t3);
      ROS_INFO("Total time %lf",totalt);

      //calculate distance travelled for each period
      double p1 = v_curr*t1 + a_curr*pow(t1,2)/2 + j0*pow(t1,3)/6;
      double p2 = v1*t2 + atgt*pow(t2,2)/2;
      double p3 = v2*t3 + atgt*pow(t3,2)/2 - jtgt*pow(t3,3)/6;
      ROS_INFO("p1 %lf p2 %lf p3 %lf",p1,p2,p3);

      //convert to sample values
      int nt = round(totalt/dt);
      ROS_INFO("Total samples %u",nt);
      double t;

      v_iter(0) = v_curr;

      int n_iter = 0;

      while (n_iter<np){
        t = (n_iter)*dt;
        // ROS_INFO("Time %lf", t);
        if (t<t1){
          v_iter(n_iter) = v_curr + a_curr*t + j0*pow(t,2)/2;
          a_iter(n_iter) = a_curr + j0*t;
          j_iter(n_iter) = j0;
        }else if(t<t1+t2){
          v_iter(n_iter) = v1 + atgt*(t-t1);
          a_iter(n_iter) = atgt;
          j_iter(n_iter) = 0;
        }else if(t<t1+t2+t3){
          double t_t2 = t - t2 - t1;
          v_iter(n_iter) = v2 + atgt*t_t2 - jtgt*pow(t_t2,2)/2;
          a_iter(n_iter) = atgt - jtgt*t_t2;
          j_iter(n_iter) = -jtgt;
        }else{
          v_iter(n_iter) = 0;
          a_iter(n_iter) = 0;
          j_iter(n_iter) = 0;
        }
        // ROS_INFO("Sample no %u %u", n_iter, np);
        // ROS_INFO("Iteration %u Planned acc %lf",n_iter,a_iter(n_iter));
        n_iter = n_iter + 1;
      }
       
    }else if (stopalgo_struct.case_no==1){

      ROS_INFO("Case no %u",stopalgo_struct.case_no);
           
      //time and jerk
      double tf = abs(2*v_curr/a_curr);
      double jt = abs(pow(a_curr,2)/(2*v_curr));
      int nt = round(tf/dt);

      ROS_INFO("Total samples %u",nt);
      double t;
      v_iter(0) = v_curr;
      int n_iter = 0;
        
      while (n_iter<np){    
        t = (n_iter)*dt;
        if(t<tf){
          v_iter(n_iter) = v_curr + a_curr*t + jt*pow(t,2)/2;
          a_iter(n_iter) = a_curr + jt*t;
          j_iter(n_iter) = jt;
        }else{
          v_iter(n_iter) = 0;
          a_iter(n_iter) = 0;
          j_iter(n_iter) = 0;
        }
        n_iter = n_iter + 1;
      }
       
    }else{ 
       
      ROS_INFO("Case no %u",stopalgo_struct.case_no);

      double j0 = (2*pow(atgt,2) - pow(a_curr,2))/(2*v_curr);
      t1 = (a_curr-atgt)/j0;
      if(t1<0.1){
        t1=0;
      }
      t2 = 0;
      t3 = -atgt/j0;

      double totalt = t1+t2+t3;
       
      v1 = v_curr + a_curr*t1 - 0.5*j0*pow(t1,2);
      v2 = -atgt*t3 -0.5*j0*pow(t3,2);
      //these are used later
      double p1 = v_curr*t1 + a_curr*pow(t1,2)/2 - j0*pow(t1,3)/6;
      double p2 = 0;
      double p3 = v2*t3 + atgt*pow(t3,2)/2 + j0*pow(t3,3)/6;   
       
      //convert to sample values
      int nt = round(totalt/dt);
      ROS_INFO("Total samples %u",nt);
      double t;

      v_iter(0) = v_curr;

      int n_iter = 0;

      while (n_iter<np){
        t = (n_iter)*dt;
        // ROS_INFO("Time %lf", t);
        if (t<t1){
          v_iter(n_iter) = v_curr + a_curr*t - j0*pow(t,2)/2;
          a_iter(n_iter) = a_curr - j0*t;
          j_iter(n_iter) = -j0;
        }else if(t<t1+t2+t3){
          double t_t2 = t - t2 - t1;
          v_iter(n_iter) = v2 + atgt*t_t2 + j0*pow(t_t2,2)/2;
          a_iter(n_iter) = atgt + j0*t_t2;
          j_iter(n_iter) = j0;
        }else{
          v_iter(n_iter) = 0;
          a_iter(n_iter) = 0;
          j_iter(n_iter) = 0;
        }
        // ROS_INFO("Sample no %u %u", n_iter, np);
        // ROS_INFO("Iteration %u Planned acc %lf",n_iter,a_iter(n_iter));
        n_iter = n_iter + 1;
      }      
    }
    
    // ROS_INFO("Traj size %lu %lu", 
      // stopalgo_struct.v_traj.size(), v_iter.size());
    stopalgo_struct.v_traj = v_iter;
    // ROS_INFO("Traj size %lu %lu", 
      // stopalgo_struct.a_traj.size(), a_iter.size());
    stopalgo_struct.a_traj = a_iter;
    // ROS_INFO("Traj size %lu %lu", 
    //   stopalgo_struct.j_traj.size(), j_iter.size());
    stopalgo_struct.j_traj = j_iter;

    return v_iter;
   }

////////////////////////// Transformation Functions //////////////////////////

/**
 * @details
 * We transform the prediction in global frame to a local ST map prediction
 * starting at 0 that will be used later on to figure out the bounds for the
 * optimization.
 *  1. We first check if the prediction given is long enough compared to our
 * planning horizon. If it is too short, we extend the prediction with constant
 * velocity following the lane for cars or using the initial heading for
 * pedestrians
 *  2. If the flag for cross path margins is active, we check if the current
 * position of the agent is crossing the path
 *  3. For each point in the planning horizon:
 *    - Extend the global position to a rectangle that includes both dimensions
 * and extra margins
 *    - Check if the rectangle overlaps with the path and get the overlap points
 *    - Transform all overlapping points to a Frenet frame and shift it by the
 * AD ar position so they start in 0
 *    - Get the minimum and max points. These will mark the lower and upper
 * positions of the agent in the ST map (rectangle height)
 *    - If there is no overlap we will set that value to NaN
 *    - We also collect the lower and upper positions after adding the
 * uncertainty when needed
 *    - For each trajectory in the ST map, we annotate the time index when the
 * object starts being visible as start_point and the time index where it stops
 * being visible as end_point, using 0 and max number of points when it is
 * visible in all the planning horizon.
 */
bool IntentionToTrajectoryAgents(
    const ai4ad::struct_params_alg &params_alg,
    const traffic_msgs::Prediction &agt_pred, const uint16_t intention_sel,
    const geometry::Frenet &ad_lane, const AgentStateSimple &ad_state,
    const ClipperLib::Paths &clip_path, ai4ad::TrajectoryAgents &traj_ag,
    const bool is_pedestrian,
    std::shared_ptr<PlotterQCustomPlot> plot_class_ptr) {
  traffic_msgs::IntentionTrajectory traj_int =
      agt_pred.trajectories[intention_sel];

  // If predictions are not long enough, use constant velocity
  uint16_t n_points = uint16_t(traj_int.trajectory_estimated.waypoints.size());
  if (n_points < params_alg.np) {
    ROS_WARN_THROTTLE(0.1,
                      "Prediction not long enough. Using constant velocity");
    geometry::Point2D p_xy_init(
        traj_int.trajectory_estimated.waypoints[n_points - 1]
            .pose.pose.position.x,
        traj_int.trajectory_estimated.waypoints[n_points - 1]
            .pose.pose.position.y);
    double v_init =
        geometry::module(traj_int.trajectory_estimated.waypoints[n_points - 1]
                             .twist.twist.linear.x,
                         traj_int.trajectory_estimated.waypoints[n_points - 1]
                             .twist.twist.linear.y);
    geometry_msgs::TwistStamped twist_val =
        traj_int.trajectory_estimated.waypoints[n_points - 1].twist;
    geometry::Point_Frenet p_sd_init;

    tf::Quaternion q_init(traj_int.trajectory_estimated.waypoints[n_points - 1]
                              .pose.pose.orientation.x,
                          traj_int.trajectory_estimated.waypoints[n_points - 1]
                              .pose.pose.orientation.y,
                          traj_int.trajectory_estimated.waypoints[n_points - 1]
                              .pose.pose.orientation.z,
                          traj_int.trajectory_estimated.waypoints[n_points - 1]
                              .pose.pose.orientation.w);
    tf::Matrix3x3 m_init(q_init);
    double init_roll, init_pitch, init_yaw;
    m_init.getRPY(init_roll, init_pitch, init_yaw);

    traffic_msgs::Waypoint wp_empty;
    wp_empty.pose.pose.position.x = 0.0;
    wp_empty.pose.pose.position.y = 0.0;
    wp_empty.twist.twist.linear.x = 0.0;
    wp_empty.twist.twist.linear.y = 0.0;

    double t = params_alg.dt;
    for (uint32_t k = n_points; k < params_alg.np; ++k) {
      geometry::Point2D p_xy;
      tf::Quaternion q;

      if (is_pedestrian) {
        p_xy = geometry::Point2D(p_xy_init.x + v_init * t * cos(init_yaw),
                                 p_xy_init.y + v_init * t * sin(init_yaw));
        q = q_init;
      }

      traffic_msgs::Waypoint wp;
      wp.pose.pose.position.x = p_xy.x;
      wp.pose.pose.position.y = p_xy.y;
      wp.pose.pose.orientation.x = q.getX();
      wp.pose.pose.orientation.y = q.getY();
      wp.pose.pose.orientation.z = q.getZ();
      wp.pose.pose.orientation.w = q.getW();
      wp.twist = twist_val;

      traj_int.trajectory_estimated.waypoints.push_back(wp);
      traj_int.trajectory_uncertainty.waypoints.push_back(wp_empty);
      t += params_alg.dt;
    }
  }

  bool agent_crossing = false;
  // Check if the agent is crossing the path
  if (params_alg.use_cross_path_margins) {
    for (uint16_t k = 0; k < params_alg.np; k++) {
      geometry::Point2D p_xy(
          traj_int.trajectory_estimated.waypoints[k].pose.pose.position.x,
          traj_int.trajectory_estimated.waypoints[k].pose.pose.position.y);
      ClipperLib::IntPoint p_long;
      if (to_cInt(p_xy.x, p_long.X) && to_cInt(p_xy.y, p_long.Y)) {
        int res = ClipperLib::PointInPolygon(p_long, clip_path[0]);
        if (res != 0) {
          agent_crossing = true;
          break;
        }
      } else {
        ROS_INFO("Conversion problem in the agent crossing check");
      }
    }
  }

  traj_ag.pos.resize(params_alg.np);
  traj_ag.vel.resize(params_alg.np);
  traj_ag.pos_lower.resize(params_alg.np);
  traj_ag.pos_upper.resize(params_alg.np);
  traj_ag.pos_lower_uncty.resize(params_alg.np);
  traj_ag.pos_upper_uncty.resize(params_alg.np);
  traj_ag.agent_id = agt_pred.agent_id;

  bool intersection_vals = false;

  for (uint16_t k = 0; k < params_alg.np; k++) {
    geometry::Point2D p_xy_lane(
        traj_int.trajectory_estimated.waypoints[k].pose.pose.position.x,
        traj_int.trajectory_estimated.waypoints[k].pose.pose.position.y);
    tf::Quaternion q;
    q.setX(traj_int.trajectory_estimated.waypoints[k].pose.pose.orientation.x);
    q.setY(traj_int.trajectory_estimated.waypoints[k].pose.pose.orientation.y);
    q.setZ(traj_int.trajectory_estimated.waypoints[k].pose.pose.orientation.z);
    q.setW(traj_int.trajectory_estimated.waypoints[k].pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // TO DO - DEAL WITH UNCERTAINTY
    double v_agt = std::hypot(
        traj_int.trajectory_estimated.waypoints[k].twist.twist.linear.x,
        traj_int.trajectory_estimated.waypoints[k].twist.twist.linear.y);

    double v_margin;
    v_margin = std::max(ad_state.vel, v_agt);

    std::vector<geometry::Point2D> agt_pts;
    double margin_agent_lat_front = 0.0, margin_agent_lat_back,
           margin_agent_front = 0.0, margin_agent_back = 0.0;
    if (is_pedestrian) {
      if (agent_crossing && params_alg.use_cross_path_margins) {
        margin_agent_front = params_alg.margin_pedestrian_cross_path;
        margin_agent_back = params_alg.margin_pedestrian_cross_path;
        margin_agent_lat_front = params_alg.margin_lat_ped_cross_path;
        margin_agent_lat_back = params_alg.margin_lat_ped_cross_path;
      } else {
        margin_agent_front = params_alg.margin_pedestrian;
        margin_agent_back = params_alg.margin_pedestrian;
        margin_agent_lat_front = params_alg.margin_lat_ped;
        margin_agent_lat_back = params_alg.margin_lat_ped;
      }
    } else {
      if (agent_crossing && params_alg.use_cross_path_margins) {
        margin_agent_back = (params_alg.margin_car_cross_path +
                             params_alg.margin_vel_cross_path * v_margin) /
                            params_alg.margin_rear_car_factor_cross_path;
        margin_agent_front = (params_alg.margin_car_cross_path +
                              params_alg.margin_vel_cross_path * v_margin);
        margin_agent_lat_front = params_alg.margin_car_lat_cross_path;
        margin_agent_lat_back = params_alg.margin_car_lat_cross_path;
      } else {
        margin_agent_back =
            (params_alg.margin_car + params_alg.margin_vel * v_margin) /
            params_alg.margin_rear_car_factor;
        margin_agent_front =
            (params_alg.margin_car + params_alg.margin_vel * v_margin);
        margin_agent_lat_front = params_alg.margin_car_lat;
        margin_agent_lat_back = params_alg.margin_car_lat;
      }
    }

    double width_agt = double(agt_pred.width),
           length_agt = double(agt_pred.length);
    agt_pts.push_back(geometry::localToGlobal(
        p_xy_lane, yaw,
        geometry::Point2D(length_agt / 2.0 + margin_agent_front,
                          -width_agt / 2.0 - margin_agent_lat_front)));
    agt_pts.push_back(geometry::localToGlobal(
        p_xy_lane, yaw,
        geometry::Point2D(length_agt / 2.0 + margin_agent_front,
                          width_agt / 2.0 + margin_agent_lat_front)));
    agt_pts.push_back(geometry::localToGlobal(
        p_xy_lane, yaw,
        geometry::Point2D(-length_agt / 2.0 - margin_agent_back,
                          width_agt / 2.0 + margin_agent_lat_back)));
    agt_pts.push_back(geometry::localToGlobal(
        p_xy_lane, yaw,
        geometry::Point2D(-length_agt / 2.0 - margin_agent_back,
                          -width_agt / 2.0 - margin_agent_lat_back)));

    double min_veh = 1e10, max_veh = -1e10;
    geometry::Point_Frenet p_sd;
    bool overlap = false;

    // Create a polygon for the car
    ClipperLib::Clipper clipper;
    ClipperLib::Paths clip_car(1);
    for (uint16_t k = 0; k < agt_pts.size(); k++) {
      ClipperLib::IntPoint p_long;
      if (to_cInt(agt_pts[k].x, p_long.X) && to_cInt(agt_pts[k].y, p_long.Y)) {
        clip_car[0].push_back(p_long);
      }
    }

    // Find intersection between car and path
    clipper.AddPaths(clip_path, ClipperLib::ptSubject, true);
    clipper.AddPaths(clip_car, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clipper.Execute(ClipperLib::ctIntersection, solution,
                    ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    overlap = solution.size() != 0;

    // Figure out s values for the intersecting points
    if (overlap) {
      for (uint16_t i = 0; i < solution[0].size(); i++) {
        geometry::Point2D p_xy_path(to_double(solution[0][i].X),
                                    to_double(solution[0][i].Y));
        geometry::Point_Frenet p_sd_path;
        ad_lane.ToFrenet(p_xy_path, p_sd_path);
        min_veh = std::min(p_sd_path.s - ad_state.pos, min_veh);
        max_veh = std::max(p_sd_path.s - ad_state.pos, max_veh);
      }
    }

    bool plot_grid = false;
    if (plot_grid) {
      plot_class_ptr->UpdatePathOverlap(clip_car, clip_path, solution);
    }

    double p_lower = 0.0, p_lower_uncty = 0.0, p_upper = 0.0,
           p_upper_uncty = 0.0;
    bool invalid_number;
    if (overlap) {
      p_lower = min_veh;
      p_lower_uncty = min_veh;
      p_upper = max_veh;
      p_upper_uncty = max_veh;
      p_sd.s = (p_lower + p_upper) / 2.0;
      invalid_number = p_upper < 0;
    } else {
      invalid_number = true;
    }

    if (invalid_number) {
      traj_ag.pos(k) = std::nan("");
      traj_ag.vel(k) = std::nan("");
      traj_ag.pos_lower(k) = std::nan("");
      traj_ag.pos_upper(k) = std::nan("");
      traj_ag.pos_lower_uncty(k) = std::nan("");
      traj_ag.pos_upper_uncty(k) = std::nan("");
    } else {
      intersection_vals = true;
      traj_ag.pos(k) = p_sd.s;
      traj_ag.vel(k) = geometry::module(
          traj_int.trajectory_estimated.waypoints[k].twist.twist.linear.x,
          traj_int.trajectory_estimated.waypoints[k].twist.twist.linear.y);
      traj_ag.pos_lower(k) = p_lower;
      traj_ag.pos_upper(k) = p_upper;
      traj_ag.pos_lower_uncty(k) = p_lower_uncty;
      traj_ag.pos_upper_uncty(k) = p_upper_uncty;
    }
  }

  bool prev_invalid = true;
  std::vector<uint16_t> start_pts;
  std::vector<uint16_t> end_pts;
  std::vector<uint16_t> diff_pts;
  for (uint16_t k = 0; k < params_alg.np; k++) {
    if (std::isnan(traj_ag.pos(k))) {
      if (!prev_invalid) {
        end_pts.push_back(k - 1);
        diff_pts.push_back(end_pts.back() - start_pts.back());
      }
      prev_invalid = true;
    } else {
      if (prev_invalid || k == 0) {
        start_pts.push_back(k);
      }
      prev_invalid = false;
    }
  }

  if (!prev_invalid) {
    end_pts.push_back(params_alg.np - 1);
    diff_pts.push_back(end_pts.back() - start_pts.back());
  }

  if (start_pts.size() == 1) {
    traj_ag.start_point = start_pts[0];
    traj_ag.end_point = end_pts[0];
  } else if (start_pts.size() > 1) {
    // TODO - Quick check for close points instead of just discarding
    std::vector<uint16_t>::iterator result =
        std::max_element(diff_pts.begin(), diff_pts.end());
    uint16_t max_ind = std::distance(diff_pts.begin(), result);
    ROS_INFO("Removing spurious data for agent: %d", traj_ag.agent_id);
    for (uint16_t i = 0; i < start_pts.size(); i++) {
      if (i == max_ind) {
        traj_ag.start_point = start_pts[i];
        traj_ag.end_point = end_pts[i];
        continue;
      }
      traj_ag.pos(Eigen::seq(start_pts[i], end_pts[i])) =
          Eigen::VectorXd::Constant(end_pts[i] - start_pts[i] + 1,
                                    std::nan(""));
      traj_ag.vel(Eigen::seq(start_pts[i], end_pts[i])) =
          Eigen::VectorXd::Constant(end_pts[i] - start_pts[i] + 1,
                                    std::nan(""));
      traj_ag.pos_lower(Eigen::seq(start_pts[i], end_pts[i])) =
          Eigen::VectorXd::Constant(end_pts[i] - start_pts[i] + 1,
                                    std::nan(""));
      traj_ag.pos_upper(Eigen::seq(start_pts[i], end_pts[i])) =
          Eigen::VectorXd::Constant(end_pts[i] - start_pts[i] + 1,
                                    std::nan(""));
      traj_ag.pos_lower_uncty(Eigen::seq(start_pts[i], end_pts[i])) =
          Eigen::VectorXd::Constant(end_pts[i] - start_pts[i] + 1,
                                    std::nan(""));
      traj_ag.pos_upper_uncty(Eigen::seq(start_pts[i], end_pts[i])) =
          Eigen::VectorXd::Constant(end_pts[i] - start_pts[i] + 1,
                                    std::nan(""));
    }
  }

  return intersection_vals;
}

/**
 * @details
 *
 * For the transformation from the 1D planned trajectory to a global space
 * trajectory we will be projecting the positions onto the ego lane in 2D.
 *  1. If we are not using time based, we verify use the max of the minimum gap
 * specified in the algorithm parameters and the time based trajectory
 *  2. If we want to interpolate from the current lateral distance to the center
 * lane back to the center lane, we figure out what the distance represents in
 * terms of indexes
 *  3. For points before the interpolation (from the beginning if
 * interp_back_path is set to 0.0), we interpolate from the initial offset to
 * zero offser over time. For points after the interpolation we just send the
 * center line 4- We use Frenet frame transformations to get the global route 5-
 * We tranform also the velocity from local 1D longitudinal only to global space
 * using the road heading
 */
void TrajectorytoWaypoints(const struct_params_alg &params_alg,
                           const ai4ad::Trajectory &traj_ag,
                           const geometry::Frenet &center_line,
                           const geometry::Point_Frenet &ad_frenet,
                           traffic_msgs::WaypointArray &wpts) {
  uint16_t n_points = uint16_t(traj_ag.pos.rows());

  Eigen::VectorXd pos_new = traj_ag.pos;
  if (!params_alg.send_time_based_trajectory) {
    Eigen::VectorXd min_separated = Eigen::VectorXd::LinSpaced(
        params_alg.np, 0.0, params_alg.min_gap * (params_alg.np - 1));
    pos_new = pos_new.cwiseMax(min_separated);
    // pos_new = min_separated;
  }

  uint16_t final_interp_idx;
  if (params_alg.interp_back_path <= 0.0) {
    final_interp_idx = 0;
  } else {
    auto it = std::lower_bound(pos_new.begin(), pos_new.end(),
                               params_alg.interp_back_path);
    if (*it >= pos_new(n_points - 1)) {
      final_interp_idx = pos_new.rows();
    } else {
      final_interp_idx = uint16_t(it - pos_new.begin()) - 1;
    }
  }

  for (uint16_t k = 0; k < n_points; k++) {
    traffic_msgs::Waypoint wp;

    // Position
    geometry::Point2D p_xy;
    double theta;

    double s_val = ad_frenet.s + pos_new(k);

    if (k < final_interp_idx) {
      double d_val =
          ad_frenet.d - double(k) * ad_frenet.d / double(final_interp_idx);
      center_line.ToCartesian(geometry::Point_Frenet(s_val, d_val), p_xy,
                              theta);
    } else {
      center_line.ToCartesian(geometry::Point_Frenet(s_val, 0.0), p_xy, theta);
    }
    wp.pose.pose.position.x = p_xy.x;
    wp.pose.pose.position.y = p_xy.y;

    // Orientation
    tf::Quaternion q_aux;
    q_aux.setRPY(0.0, 0.0, theta);
    geometry_msgs::Quaternion q;
    tf::quaternionTFToMsg(q_aux, q);
    wp.pose.pose.orientation = q;

    // Velocity
    double vel;
    if (k == n_points - 1) {
      vel = traj_ag.vel(Eigen::placeholders::last);
    } else {
      vel = traj_ag.vel[k];
    }
    wp.twist.twist.linear.x = vel * cos(theta);
    wp.twist.twist.linear.y = vel * sin(theta);

    // Accel (just debugging)
    double accel;
    if (k >= n_points - 2) {
      accel = traj_ag.accel(Eigen::placeholders::last);
    } else {
      accel = traj_ag.accel[k];
    }

    wpts.waypoints.push_back(wp);
  }
}

////////////////////////// Bounds computation  //////////////////////////
/**
 * @details
 *
 * We follow these steps to compute the optimization bounds:
 * 1. Create a vector of occupied cells from the agents trajectories
 * 2. Run ST Cell Planner to get the bounds
 * 3. Transform from cells to Eigen Vectors to be used on the optimization
 */
void TrafficBounds(const struct_params_alg &params_alg,
                   const std::vector<ai4ad::TrajectoryAgentsType> &agents_trajs,
                   std::vector<Eigen::VectorXd> &lb_pos_vec,
                   std::vector<Eigen::VectorXd> &ub_pos_vec,
                   std::vector<std::string> &keys_trajs_vec) {
  // For all agents, create the vector of occupies STCells
  std::vector<std::vector<ai4ad::STCell>> occupied_cells;
  occupied_cells.resize(params_alg.np);

  // Create occupied cells for each agent
  for (ai4ad::TrajectoryAgentsType traj : agents_trajs) {
    for (uint16_t t = traj.traj_agent.start_point;
         t <= traj.traj_agent.end_point; t++) {
      STCell st_cell(
          traj.traj_agent.pos_lower_uncty(t) - params_alg.ego_length / 2.0,
          traj.traj_agent.pos_upper_uncty(t) + params_alg.ego_length / 2.0,
          traj.traj_agent.agent_id, traj.traj_agent.agent_id);
      occupied_cells[t].push_back(st_cell);
    }
  }

  // Use ST Cell planner to get the lower and upper bounds
  ai4ad::STCellPlanner st_cell_planner(occupied_cells,
                                       2 * params_alg.plan_t * params_alg.v_max,
                                       params_alg.tvec);
  std::vector<std::vector<STCell>> candidate_plans =
      st_cell_planner.SearchCandidatePlans();

  // Transform from ST cells to Eigen vectors for the optimization
  uint16_t p = 0;

  for (std::vector<ai4ad::STCell> plan : candidate_plans) {
    // Create a set for all unique ids in lb and ub
    std::unordered_set<uint32_t> lb_ids;
    std::unordered_set<uint32_t> ub_ids;
    Eigen::VectorXd lb_pos(params_alg.np);
    Eigen::VectorXd ub_pos(params_alg.np);
    for (uint16_t t = 0; t < params_alg.np; t++) {
      lb_pos(t) = plan[t].min_s;
      ub_pos(t) = plan[t].max_s;
      lb_ids.insert(plan[t].min_s_agt_id);
      ub_ids.insert(plan[t].max_s_agt_id);
    }
    lb_pos_vec.push_back(lb_pos);
    ub_pos_vec.push_back(ub_pos);
    p++;

    // No car is specified by the uint32_t max value. We will use _ to mark
    // there is no car. Otherwise we eliminate it
    std::uint32_t invalid_id = std::numeric_limits<uint32_t>::max();
    std::string lb_str = "", ub_str = "";
    if (lb_ids.size() == 1 && *lb_ids.begin() == invalid_id) {
      lb_str = "_";
    } else {
      auto it = lb_ids.find(invalid_id);
      if (it != lb_ids.end()) {
        lb_ids.erase(invalid_id);
      }
    }
    if (ub_ids.size() == 1 && *ub_ids.begin() == invalid_id) {
      ub_str = "_";
    } else {
      auto it = ub_ids.find(invalid_id);
      if (it != ub_ids.end()) {
        ub_ids.erase(invalid_id);
      }
    }

    // Put all the ids of cars in front sorted in a string
    if (lb_str != "_") {
      std::vector<uint32_t> lb_ids_aux;
      lb_ids_aux.assign(lb_ids.begin(), lb_ids.end());
      sort(lb_ids_aux.begin(), lb_ids_aux.end());
      for (uint16_t i = 0; i < lb_ids_aux.size(); i++) {
        lb_str += std::to_string(lb_ids_aux[i]);
        if (i < lb_ids_aux.size() - 1) {
          lb_str += "/";
        }
      }
    }
    if (ub_str != "_") {
      std::vector<uint32_t> ub_ids_aux;
      ub_ids_aux.assign(ub_ids.begin(), ub_ids.end());
      sort(ub_ids_aux.begin(), ub_ids_aux.end());
      for (uint16_t i = 0; i < ub_ids_aux.size(); i++) {
        ub_str += std::to_string(ub_ids_aux[i]);
        if (i < ub_ids_aux.size() - 1) {
          ub_str += "/";
        }
      }
    }
    // keys_trajs_vec.push_back(
    //     std::to_string(ros::Time::now().toNSec() + 1e9 * double(p)));
    keys_trajs_vec.push_back(std::string(lb_str + "-" + ub_str));
  }
}

//////////////////////////// Helper functions ////////////////////////////

Eigen::VectorXd CumSum(const Eigen::VectorXd &v) {
  Eigen::VectorXd v_cs = v;
  v_cs(0) = v(0);
  for (int i = 1; i < v_cs.size(); ++i) v_cs(i) += v_cs(i - 1);
  return v_cs;
}

double normpdf(double x, double mu, double sig) {
  return (ONE_OVER_SQRT_2PI / sig) * std::exp(-0.5 * (x - mu) * (x - mu) / sig);
}

void ParallelizePathClipper(const std::vector<geometry::Point2D> &in_path,
                            const double delta,
                            ClipperLib::Paths &clipper_path) {
  // Set the points in the clipper format
  ClipperLib::Path center_path;
  for (uint32_t i = 0; i < in_path.size(); i++) {
    ClipperLib::IntPoint p_long;
    if (to_cInt(in_path[i].x, p_long.X) && to_cInt(in_path[i].y, p_long.Y)) {
      center_path.push_back(p_long);
    } else {
      return;
    }
  }

  // Do the offset
  ClipperLib::ClipperOffset co;
  co.AddPath(center_path, ClipperLib::JoinType::jtSquare,
             ClipperLib::EndType::etOpenButt);
  clipper_path.clear();
  co.Execute(clipper_path, delta * SCALE_LONG);
}

}  // namespace ai4ad
