/**
 * @file struct_defs.h
 * @author Alex Miranda A&ntilde;on (amiranda@honda-ri.com)
 * @brief Auxiliary structures used for MPQP
 * @copyright Honda Research Institute, US (HRI-US)
This software cannot be distributed without the permission HRI-US
 */

#ifndef STRUCT_DEFS_H
#define STRUCT_DEFS_H

#include <prediction_ct_vel/frenet_frame.h>
#include <prediction_ct_vel/geometry_utils.h>
#include <traffic_msgs/VehicleState.h>

#include <eigen3/Eigen/Core>
#include <unordered_map>
#include <vector>

namespace ai4ad {

////////////////////////////////////////////////////////////////////////////////
// Algorithm settings
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Structure that contains all the parameters for MPQP algorithm
 */
struct struct_params_alg {
  double dt;            /**< Time increment plan (s) */
  double plan_t;        /**< Plan horizon length (s) */
  uint16_t np;          /**< Number of points used to define the position */
  uint16_t total_np;    /**< Number of points use to define the total state. */
  Eigen::VectorXd tvec; /**< Time vector from 0 to plan_t */

  Eigen::ArrayXi
      pos_idxs; /**< Vector of indexes for position in the state vector */
  Eigen::ArrayXi
      vel_idxs; /**< Vector of indexes for velocity in the state vector */
  Eigen::ArrayXi
      acc_idxs; /**< Vector of indexes for acceleration in the state vector */
  Eigen::ArrayXi
      jrk_idxs; /**< Vector of indexes for jerk in the state vector */

  double idm_delta;   /**< IDM delta parameter */
  double idm_offset;  /**< IDM offset/length parameter */
  double idm_T0;      /**< IDM T0 (time headway) parameter (s) */
  double idm_s0;      /**< IDM s0 (space margin) parameter (m) */
  double dist_no_car; /**< IDM front car distance if no car detected (m) */
  double vel_no_car;  /**< IDM front car velocity if no car detected (m/s) */

  double v_max;     /**< Speed limit (m/s) */
  double v_max_sbc; /**< Speed limit based on curvature (m/s) */
  double acc_min;   /**< Minimum acceleration limit (m/s<SUP>2</SUP>) */
  double acc_max;   /**< Maximum acceleration limit (m/s<SUP>2</SUP>) */
  double jrk_min;   /**< Minimum jerk limit (m/s<SUP>3</SUP>) */
  double jrk_max;   /**< Maximum jerk limit (m/s<SUP>3</SUP>) */

  double wa; /**< Weight for acceleration */
  double wj; /**< Weight for jerk */
  double wf; /**< Weight for maximizing displacement */

  double ego_length;     /** Ego car length (m)  */
  double ego_width;      /**< Ego car width (m) */
  double margin_car;     /**< Extra margin for cars (front and back) (m) */
  double margin_car_lat; /**< Extra margin for cars (lateral) (m) */
  double margin_vel; /**< Extra margin based on velocity (s), i.e, 2 m/s is 1
                        extra m margin */
  double margin_rear_car_factor; /**< Reduce the rear margin by this factor (1.0
                                    no reduction, 2.0 half */
  double margin_pedestrian; /**< Extra margin for pedestrians (front and back)
                               (m) */
  double margin_lat_ped;    /**< Extra margin for pedestrians (lateral) (m) */

  bool use_cross_path_margins;  /**< Activate specific margins when the car
                                   actually crosses ego path */
  double margin_car_cross_path; /**< Extra margin for cars (front and back) (m)
                                   - CROSS*/
  double margin_car_lat_cross_path; /**< Extra margin for cars (lateral) (m) -
                                       CROSS */
  double margin_vel_cross_path; /**< Extra margin based on velocity (s), i.e, 2
                                   m/s is 1 extra m margin - CROSS*/
  double margin_rear_car_factor_cross_path; /**< Reduce the rear margin by this
                                               factor (1.0 no reduction, 2.0
                                               half - CROSS */
  double margin_pedestrian_cross_path; /**< Extra margin for pedestrians (front
                                          and back) (m) */
  double margin_lat_ped_cross_path; /**< Extra margin for pedestrians (lateral)
                                       (m) - CROSS */

  bool match_accel;    /**< Match initial acceleration */
  bool use_prev_accel; /**< Use previous acceleration from optimizer. Assumes
                          match accel is false */
  bool send_time_based_trajectory; /**< Send time based trajectories as the
                                      exact result out of the optimization */
  double min_gap;          /**< Min gap for space based trajectory (m) */
  double interp_back_path; /**< How many meters to return to path (m) */

  
};

////////////////////////////////////////////////////////////////////////////////
// Sensor and Road data structures
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Data structure for data can be inferred from sensor data
 */
struct SensorData {
  double accel_estim;   /**< Estimated acceleration for current vehicle
                           (m/s<SUP>2</SUP>)*/
  double ego_vel;       /**< Velocity of the current vehicle (m/s) */
  double preceding_vel; /**< Velocity of the vehicle in front of the current
                           vehicle (m/s) */
  double space_headway; /**< Distace betwen current vehicle and front vehicle
                           (m) */
};

/**
 * @brief Data structure with all the IDM parameters
 */
struct IDM {
  double acc_max; /**< IDM max acceleration (m/s<SUP>2</SUP>) */
  double brk_max; /**< IDM max braking (m/s<SUP>2</SUP>) */
  double v0;      /**< IDM reference speed (m/s) */
  double s0;      /**< IDM s0 (space margin) (m) */
  double T0;      /**< IDM T0 (time headway) (s) */
};

/**
 * @brief Data structure with parameters for Stopline Breaking algo 
 * - added by shrita
 */
struct StoplineBreak {
  Eigen::VectorXd vmax_stopline; /**<Upper bound on velocity for stop line*/
  bool NearStopline; /**<Bool for whether car is in stopline region*/
  bool Break;
  double stopline_dist_m; /**<Distance between car and stopline*/
  int stopline_timer; /**<Timer for stopline wait*/
  int max_iter;
  double a_init;
  double j_init;
  float rate;
  Eigen::VectorXd v_traj; /**<Upper bound on velocity for stop line*/
  Eigen::VectorXd a_traj; /**<Upper bound on velocity for stop line*/
  Eigen::VectorXd j_traj; /**<Upper bound on velocity for stop line*/

  double stopline_dmin; /**<Min distance from stop line at which 
                                      decceleration should begin*/
  double stopline_amin; /**<Min acc for stopping behaviour*/ 
  double stopline_tmax; /**<Max time for stopping behaviour*/
  double stopline_threshold; /**<Threshold distance for stopping and starting timer*/
  int case_no;
};

/**
 * @brief Data structure for the estimated vehicle parameters for prediction
 */
struct VehiclesEstim {
  IDM idm_vals;         /**< IDM parameters estimated */
  double p_cooperative; /**< Probabiluty the car is cooperative, 0.0 not
                           cooperative, 1.0 cooperative */
  uint32_t lifetime_id; /**< Id for the agent */
  bool estimated; /**< Boolean that specifies if the values have been estimated
                     yet */
};

/**
 * @brief Data structure that contains the vehicle information for a lane
 */
struct VehiclesLaneData {
  std::vector<traffic_msgs::VehicleState>
      sorted_vehicles; /**< Vector of vehicle states sorted from front to back
                          in the direction of traffict*/
  int32_t id;          /**< Lane id */
};

/**
 * @brief Data structure for the lane information
 */
struct LaneInfo {
  geometry::Frenet frenet_path; /**< Frenet path variable */
  std::vector<double>
      sbc_path;     /**< Vector of speed values based on curvatgure */
  float lane_width; /**< Lane width (m) */
};

/**
 * @brief Id used for the AV ego lane
 */
static const int32_t kADLane = 0;

/**
 * @brief Id used as the target lane in a cross traffic scenario
 */
static const int32_t kTargetCrossTraffic = 100;

/**
 * @brief Id for of an auxiliary ghost vehicle used for stopping in some
 * scenarios
 */
static const uint32_t kGhostID = 123456789;

////////////////////////////////////////////////////////////////////////////////
// Trajectory structure
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Enum describing agents' types
 */
enum TypeAgent {
  /// Car
  kCar,
  /// Pedestrian
  kPedestrian
};

/**
 * @brief Data structure for a trajectory
 */
struct Trajectory {
  Eigen::VectorXd pos;    /**< Position vector (m) */
  Eigen::VectorXd vel;    /**< Velocity vector (m/s) */
  Eigen::VectorXd accel;  /**< Acceleration vector (m/s<SUP>2</SUP>) */
  Eigen::VectorXd jerk;   /**< Jerk vector (m/s<SUP>3</SUP>) */
  Eigen::VectorXd margin; /**< Margin vector (m) */
  uint32_t agent_id;      /**< Agent id */
};

/**
 * @brief Additional data structure for agents
 */
struct TrajectoryAgents : Trajectory {
  uint16_t start_point; /**< Starting index where the agent shows in ST map */
  uint16_t end_point;   /**< Final index where the agent shows in ST map */
  Eigen::VectorXd
      pos_lower; /**< Lower position vector including margins and dimensions*/
  Eigen::VectorXd
      pos_upper; /**< Upper position vector including margins and dimensions*/
  Eigen::VectorXd pos_lower_uncty; /**< Lower position vector including margins,
                                      dimensions and uncertainty */
  Eigen::VectorXd pos_upper_uncty; /**< Upper position vector including margins,
                                      dimensions and uncertainty */
};

/**
 * @brief Data structure for a simplified agent state
 */
struct AgentStateSimple {
  double pos;   /** Current position (m) */
  double vel;   /** Current velocity (m/s) */
  double accel; /** Current acceleration (m/s<SUP>2</SUP>) */
};

/**
 * @brief Agents trajectories
 */
struct TrajectoryAgentsType {
  ai4ad::TrajectoryAgents traj_agent; /** Trajectory values */
  TypeAgent type_agent;               /** Type of agent */
};

}  // namespace ai4ad

#endif
