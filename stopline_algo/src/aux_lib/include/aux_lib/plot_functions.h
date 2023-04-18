/**
 * @file plot_functions.h
 * @author Alex Miranda A&ntilde;on (amiranda@honda-ri.com)
 * @brief Declarations for the visualizations of MPQP
 * @copyright Honda Research Institute, US (HRI-US)
This software cannot be distributed without the permission HRI-US
 */

#ifndef PLOT_LIB_H
#define PLOT_LIB_H

#include <QCustomPlot/qcustomplot.h>
#include <aux_lib/struct_defs.h>
#include <ros/ros.h>
#include <signal.h>
#include <traffic_msgs/PlannedTrajectory.h>

#include <QMainWindow>
#include <QTimer>
#include <aux_lib/clipper.hpp>
#include <memory>
#include <thread>

namespace ai4ad {

/**
 * @brief Auxuliary structure to help for plotting the behaviors
 */
struct AuxTrajPlotStruct {
  Eigen::VectorXd plot_pos; /**< 1D planned position in the local space */
  Eigen::VectorXd plot_vel; /**< 1D planned velocity in the local space */

  //ADDED BY SHRITA 
  Eigen::VectorXd plot_acc; /**< 1D planned acceleration in the local space */
  Eigen::VectorXd plot_jrk; /**< 1D planned jerk in the local space */
  
  
  traffic_msgs::PlannedTrajectory plan_traj; /**< Planned trajectory message in
                                                the global space and validity */
};

/**
 * @brief Class to create an online QT plotter for the ST Map and velocity
 * profiles
 */
class PlotterQCustomPlot {
 public:
  /**
   * @brief Construct a new PlotterQCustomPlot object
   *
   * @param params_alg Algorithm parameters
   * @param argc Argument count needed for QApplication
   * @param argv Argument vector needed for QApplications
   */
  PlotterQCustomPlot(struct_params_alg &params_alg, int argc, char **argv,
                     bool need_main_window = true);

  /**
   * @brief Update current agent data, AD state and planned trajectories
   * @param agents_trajs Agents trajectories in ST map
   * @param agents_best Index to select best trajectory in
   * multimodal setting
   * @param ad_trajs Planned trajectories for AD
   * @param ad_cur_state Current state for AD
   */
  void UpdateData(
      const std::vector<AuxTrajPlotStruct> &ad_trajs,
      const ai4ad::AgentStateSimple &ad_cur_state);

  /**
   * @brief Update the best trajectory index from all planned trajectories
   * @param best_traj_idx Best trajectory index
   */
  void UpdateBest(const int16_t best_traj_idx) {
    best_traj_idx_m = best_traj_idx;
  }

  /**
   * @brief Updata algorithm parameters
   * @param params_alg New algorithm parameters
   * @return true -> If the plot has been initalized and can be updated
   * @return false -> If the plot has not been created so we cannot update yet
   */
  bool UpdateParams(const struct_params_alg &params_alg) {
    if (!plot_initialized_m) return false;
    params_alg_m = params_alg;
    st_map_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    st_map_plot_m->axis(QCPAxis::atLeft)
        ->setRange(-1.0, params_alg_m.v_max * params_alg_m.plan_t);
    vel_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    vel_plot_m->axis(QCPAxis::atLeft)->setRange(0.0, params_alg_m.v_max);

    //ADDED BY SHRITA updates acc, jerk graph based on params
    acc_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    acc_plot_m->axis(QCPAxis::atLeft)
        ->setRange(params_alg_m.acc_min, params_alg_m.acc_max);
    jrk_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    jrk_plot_m->axis(QCPAxis::atLeft)
        ->setRange(params_alg_m.jrk_min, params_alg_m.jrk_max);

    return true;
  }

  /**
   * @brief Signal handler to stop plotter application
   * @param signum Signal id
   */
  void SigHandler(int signum) {
    timer_m->stop();
    if (secondary_initialized) {
      sec_window_m->close();
    }
    if (need_main_window_m) {
      main_window_m->close();
    }
    application_m->quit();
    sleep(1);
    application_m->exit();
    application_m.release();
    exit(EXIT_SUCCESS);
  }

  void QuitApplication() { stop_application_m = true; }

  /**
   * @brief Uses to plot the car, the path and the solution from the
   * clipper library
   *
   * @param car Clipper path that represents the car
   * @param path Clipper path that represents the ego car
   * @param sol Clipper path that represents the solution of the overlap if any
   */
  void UpdatePathOverlap(const ClipperLib::Paths &car,
                         const ClipperLib::Paths &path,
                         const ClipperLib::Paths &sol);

  /**
   * @brief Uses qcustomplot to plot all the agents in the ST map and loops
   * through all the possible combinations of lower bounds and upper bounds
   * found
   *
   * @param agents_trajs Agents trajectories in ST map
   * @param agents_best Index to select best trajectory in multimodal setting
   * @param ub_vec Upper bound
   * @param lb_vec Lower bound
   */
  void UpdateDataAndBounds(
      const std::vector<Eigen::VectorXd> &ub_vec,
      const std::vector<Eigen::VectorXd> &lb_vec);

 private:
  // VARIABLES
  // Qt variables
  bool need_main_window_m; /**< Flag to visualize main window */
  bool stop_application_m; /**< Stop application cleanly without signal */
  std::unique_ptr<QApplication>
      application_m; /**< QT application unique pointer*/
  std::unique_ptr<std::thread>
      application_thread_m; /**< Dettached thread for the QT application */
  std::unique_ptr<QMainWindow>
      main_window_m; /**< QT MainWindow unique pointer */
  std::unique_ptr<QCustomPlot>
      custom_plot_m; /**< Unique pointer for QCustomPlot variable */
  std::unique_ptr<QCPAxisRect>
      st_map_plot_m; /**< Top rectangles to plot ST map */
  std::unique_ptr<QCPAxisRect>
      vel_plot_m; /**< Bottom rectangle to plot velocity profiles */

  //ADDED BY SHRITA declare acc,jerk plot
  std::unique_ptr<QCPAxisRect>
      acc_plot_m; /**< Bottom rectangle to plot acc profiles */ 
  std::unique_ptr<QCPAxisRect>
      jrk_plot_m; /**< Bottom rectangle to plot jrk profiles */ 

  std::unique_ptr<QTimer> timer_m; /**< QT timer to update plot */
  std::vector<QColor>
      color_palette_m; /**< QT color palette for fixed color patterns */

  // Secondary plot
  std::unique_ptr<QMainWindow>
      sec_window_m; /**< QT MainWindow unique pointer for secondary */
  std::unique_ptr<QCustomPlot>
      custom_plot_sec_m;      /**< Unique pointer for QCustomPlot variable for
                                 secondary*/
  bool secondary_initialized; /**< Flag to check if secondary has been
                                 initialized*/
  ClipperLib::Paths car_clipper_m;
  ClipperLib::Paths path_clipper_m;
  ClipperLib::Paths sol_clipper_m;
  std::vector<Eigen::VectorXd> ub_vec_m;
  std::vector<Eigen::VectorXd> lb_vec_m;
  bool data_bounds_m = false;
  bool data_path_overlap_m = false;

  // Initialization params
  struct_params_alg &params_alg_m; /**< Algorithm parameters */
  int argc_m;                      /**< Argument count copy */
  char **argv_m;                   /**< Argument vector copy */

  // Data variables
  bool data_updated_m = false; /**< Flag to see if the data has been updated */
  bool
      plot_initialized_m; /**< Flag to check if the plot has been initialized */
  std::vector<std::vector<ai4ad::TrajectoryAgentsType>>
      agents_trajs_m; /**< Variable to keep the latest trajetories for agents */
  std::vector<uint16_t> agents_best_m; /**< Variable to keep the best indexes
                                          for agent trajectories */
  std::vector<AuxTrajPlotStruct>
      ad_trajs_m; /**< Variable to keep the AD planned trajectories */
  ai4ad::AgentStateSimple
      ad_cur_state_m;      /**< Variable to keep AD current state */
  int16_t best_traj_idx_m; /**< Variable to keep the best index from all AD
                              planned trajectories */

  // Plot function
  /**
   * @brief Start the new QApplication in a second thread and prepare the plot
   * format
   */
  void InitThread();

  /**
   * @brief Visualize the most recent data
   */
  void PlotData();

  void PlotSecondary() {
    PlotPathOverlap();
    PlotDataAndBounds();
  }

  void PlotPathOverlap();

  void PlotDataAndBounds();
};

/**
 * @brief Wrapper function for signal handler set as a global to remember the
 * object
 */
// extern PlotterQCustomPlot *signal_handler_aux;
extern std::unique_ptr<PlotterQCustomPlot> signal_handler_aux;

}  // namespace ai4ad

#endif
