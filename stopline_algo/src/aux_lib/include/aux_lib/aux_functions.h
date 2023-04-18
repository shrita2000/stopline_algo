/**
 * @file aux_functions.h
 * @author Alex Miranda A&ntilde;on (amiranda@honda-ri.com)
 * @brief Declaration of auxiliary functions for MPQP
 * @copyright Honda Research Institute, US (HRI-US)
This software cannot be distributed without the permission HRI-US
 */

#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H

#include <aux_lib/plot_functions.h>
#include <aux_lib/st_cell_planner.h>
#include <aux_lib/struct_defs.h>
#include <traffic_msgs/Prediction.h>

#include <aux_lib/clipper.hpp>

namespace ai4ad {

//////////////////////////// IDM Functions ////////////////////////////

/**
 * @brief Use Intelligent Driver Model (%IDM) to driver the car
 *
 * @param ego_state Current ego state
 * @param ego_idm Current %IDM parameters
 * @param params_alg Algorithm parameters
 * @param coop_flag Is ego car cooperating with AD vehicle
 * @param prec_car_flag Is there a preceding car
 * @param prec_car Preceding car predicted trajectory, could be empty if no
 * preceding car
 * @param ad_car AD car predicted trajectory, could be empty if not cooperating
 * @return Trajectory -> Planned trajectory
 *
 * @note Here ego means current car that we are planning for, while AD means the
 * Autonomous Driving vehicle. For driving %IDM as the AD car, just set coop
 * flag to false and use AD as ego_state
 */
Trajectory DriveCarIDM(const ai4ad::AgentStateSimple &ego_state,
                       const ai4ad::IDM &ego_idm,
                       const struct_params_alg &params_alg,
                       const bool coop_flag, const bool prec_car_flag,
                       const Trajectory &prec_car, const Trajectory &ad_car);

/**
 * @brief Compute single acceleration for %IDM
 *
 * @param ego_s Ego car position
 * @param ego_v Ego car velocity
 * @param front_car_s Front car position
 * @param front_car_v Front car velocity
 * @param params_alg Algorithm parameters
 * @param ego_idm Ego %IDM parameters
 * @return double -> Planned %IDM acceleration
 * @note The acceleration is clamped to the limits specified by the %IDM
 * parameters
 */
double AccelIDM(const double ego_s, const double ego_v,
                const double front_car_s, const double front_car_v,
                const struct_params_alg &params_alg, const ai4ad::IDM &ego_idm);

/**
 * @brief Compute single acceleration for %IDM
 *
 * @param delta_s Headway between ego and front car
 * @param ego_v Ego car velocity
 * @param front_car_v Front car velocity
 * @param params_alg Algorithm parameters
 * @param ego_idm Ego %IDM parameters
 * @return double -> Planned %IDM acceleration
 * @note The acceleration is clamped to the limits specified by the %IDM
 * parameters
 */
double AccelIDM(const double delta_s, const double ego_v,
                const double front_car_v, const struct_params_alg &params_alg,
                const ai4ad::IDM &ego_idm);

//////////////////////////// Trajectory creation ////////////////////////////

/**
 * @brief Create a constant velocity trajectory
 *
 * @param ego_state Current ego state
 * @param params_alg Algorithm parameters
 * @return Trajectory -> Planned trajectory
 */

Trajectory ConstantVel(const ai4ad::AgentStateSimple &ego_state,
                       const struct_params_alg &params_alg);

/**
 * @brief Three step braking (min jerk, const max decel, max jerk until 0 accel)
 *
 * @param[in] a0 Starting acceleration
 * @param[in] v0 Starting velocity
 * @param[in] params_alg Algorithm parameters
 * @param[out] traj_output Planned trajectory
 * @return true -> We found a valid braking trajectory
 * @return false -> No solutions found
 */
bool ThreeStepBraking(const double &a0, const double &v0,
                      const struct_params_alg &params_alg,
                      Trajectory &traj_output);

/**
 * @brief Plan a braking trajectory
 *
 * @param[in] state_input Current ego state
 * @param[in] params_alg Algorithm parameters
 * @param[in] three_step_braking Flag to check if we want to run a three step
 * braking, may give no solution
 * @param[out] traj_output Planned trajectory
 * @return true -> Trajectory returned is valid
 * @return false -> invalid
 */
bool BrakingBehavior(const ai4ad::AgentStateSimple &state_input,
                     const ai4ad::struct_params_alg &params_alg,
                     const bool &three_step_braking,
                     ai4ad::Trajectory &traj_output);

/**
 * @brief Plan a max jerk trajectory to max speed
 *
 * @param[in] state_input Current ego state
 * @param[in] params_alg Algorithm parameters
 * @param[out] traj_output Planned trajectory
 */
void AccelerationBehavior(const ai4ad::AgentStateSimple &state_input,
                          const ai4ad::struct_params_alg &params_alg,
                          ai4ad::Trajectory &traj_output);

bool CheckStopLine(double stopline_dist,
                    const ai4ad::AgentStateSimple &cur_ad_state, ai4ad::StoplineBreak 
                    &stopalgo_struct, const ai4ad::struct_params_alg &params_alg); 

void CheckStopLine2(double stopline_dist,
                    const ai4ad::AgentStateSimple &cur_ad_state, ai4ad::StoplineBreak 
                    &stopalgo_struct, const ai4ad::struct_params_alg &params_alg);

double CalcDist(double atgt, double jtgt, double v_curr, double a_curr, ai4ad::StoplineBreak 
                    &stopalgo_struct);

Eigen::VectorXd CalcTraj(double atgt, double jtgt, double v_curr, 
        double a_curr, float dt, int np, ai4ad::StoplineBreak 
                    &stopalgo_struct);

////////////////////////// Transformation Functions //////////////////////////

/**
 * @brief Get the ST map for a specific intention from a prediction message
 *
 * @param[in] params_alg Algorithm parameters
 * @param[in] agt_pred Prediction message in global space
 * @param[in] intention_sel Intention to use for the prediction message
 * @param[in] ad_lane  Frenet frame variable for the AD lane
 * @param[in] ad_state AD current state
 * @param[in] clip_path AD Path used to check for overlap
 * @param[out] traj_ag Trajectory in the ST frame
 * @param[in] is_pedestrian
 * @return true ->There is overlap with AD path
 * @return false -> No overlap with AD path
 */
bool IntentionToTrajectoryAgents(
    const ai4ad::struct_params_alg &params_alg,
    const traffic_msgs::Prediction &agt_pred, const uint16_t intention_sel,
    const geometry::Frenet &ad_lane, const ai4ad::AgentStateSimple &ad_state,
    const ClipperLib::Paths &clip_path, ai4ad::TrajectoryAgents &traj_ag,
    const bool is_pedestrian,
    std::shared_ptr<PlotterQCustomPlot> plot_class_ptr);

/**
 * @brief Tranform a 1D trajectory to waypoints to send to gatekeeper
 *
 * @param[in] params_alg Algorithm parameter
 * @param[in] traj_ag 1D planned trajectory
 * @param[in] center_line Path to onto which project the trajectory (global
 * space)
 * @param[in] ad_frenet AD current state in Frenet frame
 * @param[out] wpts Returned array of waypoints in global space
 */
void TrajectorytoWaypoints(const struct_params_alg &params_alg,
                           const ai4ad::Trajectory &traj_ag,
                           const geometry::Frenet &center_line,
                           const geometry::Point_Frenet &ad_frenet,
                           traffic_msgs::WaypointArray &wpts);

////////////////////////// Bounds computation  //////////////////////////

/**
 * @brief Compute the vector of traffic bounds that will be used for the
 * optimization
 *
 * @param[in] params_alg params_alg Algorithm parameters
 * @param[in] ad_cur ad_cur AD current state
 * @param[in] agents_trajs Trajectories for all agents
 * @param[out] lb_pos_vec Returned vector of lower bounds
 * @param[out] ub_pos_vec Returned vector of upper bounds
 * @param[out] keys_trajs_vec Returned vector of key strings naming the gaps
 */
void TrafficBounds(const struct_params_alg &params_alg,
                   const std::vector<ai4ad::TrajectoryAgentsType> &agents_trajs,
                   std::vector<Eigen::VectorXd> &lb_pos_vec,
                   std::vector<Eigen::VectorXd> &ub_pos_vec,
                   std::vector<std::string> &keys_trajs_vec);

//////////////////////////// Helper functions ////////////////////////////////

/**
 * @brief Sign function
 *
 * @tparam T
 * @param val Value
 * @return int -> Sign of value, -1, 0, 1
 */
template <typename T>
int signfcn(T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief Similar to MATLAB, computes the cumulative sum of the elements of v
 *
 * @param v Input vector
 * @return Eigen::VectorXd -> Cumulative sum vector
 */
Eigen::VectorXd CumSum(const Eigen::VectorXd &v);

/**
 * @brief Const double for: \f$ \frac{1}{2\pi} \f$
 */
const double ONE_OVER_SQRT_2PI = 0.39894228040143267793994605993438;

/**
 * @brief Compute the probability density function (pdf) of the standard normal
 * distribution
 *
 * @param x Value at which to evaluate
 * @param mu Mean of the normal distribution
 * @param sig Standard deviation of the normal distribution
 * @return double -> Pdf value
 */
double normpdf(double x, double mu, double sig);

/**
 * @brief Compute a clipper path that will be used to check for overlap
 *
 * @param[in] in_path Vector of 2D points representing the path
 * @param[in] delta Width to expand to the right and to the left
 * @param[out] clipper_path Clipper path returned
 *
 * @note For Clipper math we use long integers so there is a need to transform
 * from doubles to ClipperLib::cInt to create the path
 */
void ParallelizePathClipper(const std::vector<geometry::Point2D> &in_path,
                            const double delta,
                            ClipperLib::Paths &clipper_path);

/**
 \hideinitializer
 * @brief Scale used to use to convert from double to long integer (1e-04 m)
 */
double const SCALE_LONG = 10000.0;

// representable range
/**
 \hideinitializer
 * @brief Min value representable
 */
double const MIN_VALUE_LONG =
    std::numeric_limits<ClipperLib::cInt>::min() / SCALE_LONG;

/**
 \hideinitializer
 * @brief Max value representable
 */
double const MAX_VALUE_LONG =
    std::numeric_limits<ClipperLib::cInt>::max() / SCALE_LONG;

/**
 * @brief Conversion from double to long integer
 *
 * @param[in] v Input double value
 * @param[out] v_long Output long integer value
 * @return true -> Valid conversion
 * @return false -> Invalid conversion
 */
inline bool to_cInt(double v, ClipperLib::cInt &v_long) {
  if (v < 0) {
    if (v < MIN_VALUE_LONG) {
      std::printf("v: %0.7F, v_min: %0.7F\n", v, MIN_VALUE_LONG);
      v_long = 0;
      std::cerr << "Out of limits conversion to long" << std::endl;
      return false;
    } else {
      v_long = static_cast<ClipperLib::cInt>(v * SCALE_LONG - 0.5);
      return true;
    }
  } else {
    if (v > MAX_VALUE_LONG) {
      std::printf("v: %0.7F, v_max: %0.7F\n", v, MAX_VALUE_LONG);
      v_long = 0;
      std::cerr << "Out of limits conversion to long" << std::endl;
      return false;
    } else {
      v_long = static_cast<ClipperLib::cInt>(v * SCALE_LONG + 0.5);
      return true;
    }
  }
}

/**
 * @brief Conversion from long integer to double
 *
 * @param v Input long integer value
 * @return double -> Output value
 */
inline double to_double(ClipperLib::cInt &v) {
  return static_cast<double>(v / SCALE_LONG);
}

}  // namespace ai4ad

#endif
