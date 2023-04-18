/**
 * @file plot_functions.cpp
 * @author Alex Miranda A&ntilde;on (amiranda@honda-ri.com)
 * @brief Implementation of the visualizations of MPQP
 * @copyright Honda Research Institute, US (HRI-US)
This software cannot be distributed without the permission HRI-US
 */

#include <aux_lib/aux_functions.h>
#include <aux_lib/plot_functions.h>

#include <QDebug>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QMetaEnum>
#include <QScreen>

namespace ai4ad {

// PlotterQCustomPlot *signal_handler_aux;
std::unique_ptr<PlotterQCustomPlot> signal_handler_aux;

/**
 * @brief Signal handler wrapper
 * @param signum
 */
void SigHandlerWrapper(int signum) { signal_handler_aux->SigHandler(signum); }

/**
 * @details During construction, this functions saves the parameters and creates
 * a new thread that will initialize all the necessary QT components. This
 * thread will be detached and kept independent from main thread
 */
PlotterQCustomPlot::PlotterQCustomPlot(struct_params_alg &params_alg, int argc,
                                       char **argv, bool need_main_window)
    : params_alg_m(params_alg), argc_m(argc), argv_m(argv) {
  need_main_window_m = need_main_window;
  stop_application_m = false;
  data_updated_m = false;
  plot_initialized_m = false;
  secondary_initialized = false;
  best_traj_idx_m = -1;
  application_thread_m = std::unique_ptr<std::thread>(
      new std::thread(&PlotterQCustomPlot::InitThread, this));
  application_thread_m->detach();
}

/**
 * @details
- Create a QApplication and a Qt MainWindow for the visualization
- Generate a QCustomPlot (external library) that will handle the plotting of the
desired graphs
- Set the size of the windows and the layout of the plots
- Set the axis for the different plots
- Create a color palette that will be used for the different valid AD
trajectories
  - Start the application, start a 0.1 second timer for plotting and set up all
the  signal handlers for correct destruction
 */
void PlotterQCustomPlot::InitThread() {
  application_m =
      std::unique_ptr<QApplication>(new QApplication(argc_m, argv_m));

  if (need_main_window_m) {
    // custom_plot_m = new QCustomPlot();
    custom_plot_m = std::unique_ptr<QCustomPlot>(new QCustomPlot());

    main_window_m = std::unique_ptr<QMainWindow>(new QMainWindow());

    // setup customPlot as central widget of window
    main_window_m->setCentralWidget(custom_plot_m.get());
    main_window_m->setGeometry(100, 100, 500, 700);

    // Set up the plots
    custom_plot_m->plotLayout()->clear();

    // Create 2 different plots for ST map and velocity
    st_map_plot_m =
        std::unique_ptr<QCPAxisRect>(new QCPAxisRect(custom_plot_m.get()));
    vel_plot_m =
        std::unique_ptr<QCPAxisRect>(new QCPAxisRect(custom_plot_m.get()));
    
    //ADDED BY SHRITA creates custom plot for acc, jerk
    acc_plot_m =
        std::unique_ptr<QCPAxisRect>(new QCPAxisRect(custom_plot_m.get()));
    jrk_plot_m =
        std::unique_ptr<QCPAxisRect>(new QCPAxisRect(custom_plot_m.get()));

    //adds new element to plot layout row wise
    custom_plot_m->plotLayout()->addElement(0, 0, st_map_plot_m.get());
    custom_plot_m->plotLayout()->addElement(1, 0, vel_plot_m.get());
    
    //ADDED BY SHRITA adds acc, jerk plot element
    custom_plot_m->plotLayout()->addElement(2, 0, acc_plot_m.get());
    custom_plot_m->plotLayout()->addElement(3, 0, jrk_plot_m.get());
    
    //sets row stretch factor for each row
    custom_plot_m->plotLayout()->setRowStretchFactor(0, 2);
    custom_plot_m->plotLayout()->setRowStretchFactor(1, 1);

    //ADDED BY SHRITA set row stretch factor for acc,jrk plot
    custom_plot_m->plotLayout()->setRowStretchFactor(2, 1);
    custom_plot_m->plotLayout()->setRowStretchFactor(3, 1);

    // Axis for the ST map
    st_map_plot_m->axis(QCPAxis::atBottom)->setLabel("t [s]");
    st_map_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    st_map_plot_m->axis(QCPAxis::atBottom)->setSubTicks(false);
    st_map_plot_m->axis(QCPAxis::atLeft)->setLabel("s [m]");
    st_map_plot_m->axis(QCPAxis::atLeft)
        ->setRange(-1.0, params_alg_m.v_max * params_alg_m.plan_t);
    st_map_plot_m->axis(QCPAxis::atLeft)->setSubTicks(false);

    // Axis for the velocity plot
    vel_plot_m->axis(QCPAxis::atBottom)->setLabel("t [s]");
    vel_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    vel_plot_m->axis(QCPAxis::atBottom)->setSubTicks(false);
    vel_plot_m->axis(QCPAxis::atLeft)->setLabel("v [m/s]");
    vel_plot_m->axis(QCPAxis::atLeft)->setRange(0.0, params_alg_m.v_max);
    vel_plot_m->axis(QCPAxis::atLeft)->setSubTicks(false);
    
    //ADDED BY SHRITA axis for additional acc plot
    acc_plot_m->axis(QCPAxis::atBottom)->setLabel("t [s]");
    acc_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    acc_plot_m->axis(QCPAxis::atBottom)->setSubTicks(false);
    acc_plot_m->axis(QCPAxis::atLeft)->setLabel("a [m/s2]");
    acc_plot_m->axis(QCPAxis::atLeft)
        ->setRange(params_alg_m.acc_min, params_alg_m.acc_max);
    acc_plot_m->axis(QCPAxis::atLeft)->setSubTicks(false);

    //ADDED BY SHRITA axis for additional jrk plot
    jrk_plot_m->axis(QCPAxis::atBottom)->setLabel("t [s]");
    jrk_plot_m->axis(QCPAxis::atBottom)->setRange(0.0, params_alg_m.plan_t);
    jrk_plot_m->axis(QCPAxis::atBottom)->setSubTicks(false);
    jrk_plot_m->axis(QCPAxis::atLeft)->setLabel("j [m/s3]");
    jrk_plot_m->axis(QCPAxis::atLeft)
        ->setRange(params_alg_m.jrk_min, params_alg_m.jrk_max);
    jrk_plot_m->axis(QCPAxis::atLeft)->setSubTicks(false);

    // Create color palette
    color_palette_m = {Qt::blue,        Qt::cyan,      Qt::yellow,
                       Qt::black,       Qt::darkBlue,  Qt::darkCyan,
                       Qt::darkMagenta, Qt::darkYellow};

    sleep(1);

    main_window_m->show();

    plot_initialized_m = true;
  }

  signal(SIGTERM, SigHandlerWrapper);
  signal(SIGABRT, SigHandlerWrapper);
  signal(SIGINT, SigHandlerWrapper);
  signal(SIGKILL, SigHandlerWrapper);

  timer_m = std::unique_ptr<QTimer>(new QTimer());
  QObject::connect(timer_m.get(), &QTimer::timeout, [=]() {
    if (!stop_application_m) {
      PlotData();
      PlotSecondary();
    } else {
      SigHandler(0);
    }
  });

  timer_m->start(100);

  application_m->exec();
}

void PlotterQCustomPlot::UpdateData(
    const std::vector<AuxTrajPlotStruct> &ad_trajs,
    const AgentStateSimple &ad_cur_state) {
  ad_trajs_m = ad_trajs;
  ad_cur_state_m = ad_cur_state;
  data_updated_m = true;
}

/**
 * @details
  - Plot the ST map in the top rectangle
    1. Plot target lane trajectories and the agents' ids in black (uncertainty
 in yellow). For multimodal cases, we will plot in solid lines the best
 probability trajectory and the other ones in dashed lines
    2. Plot cross lane trajectories and the agents' ids in dark blue for cars
 and red for pedestrians (uncertainty in cyan). For multimodal cases, we will
 plot in solid lines the best probability trajectory and the other ones in
 dashed lines
    3. Plot the AD planned positions in the ST map. We use the color palette for
 consistent colors. Green will always be the selected trajectory sent to
 controls
  - Plot the AD planned velocity profiles in the bottom rectangle. We use the
 color palette for consistent colors. Green will always be the selected
 trajectory sent to controls
 */
void PlotterQCustomPlot::PlotData() {
  if (!data_updated_m || !plot_initialized_m) return;

  auto start = std::chrono::high_resolution_clock::now();

  // Time vector, common to all plots
  uint32_t n_points = uint32_t(params_alg_m.tvec.size() * 2);
  int n_points_int = int(n_points);
  Eigen::VectorXd x(n_points);
  x << params_alg_m.tvec, params_alg_m.tvec.reverse();
  QVector<double> x_qt(2 * params_alg_m.np);
  memcpy(x_qt.data(), x.data(), sizeof(double) * n_points);

  custom_plot_m->clearPlottables();
  custom_plot_m->clearItems();
  uint16_t count_graph = 0;

  int font_size = 7;
  double offset_text = -0.5;

  // TRAJECTORY PLOTTING
  bool any_traj_valid = false;
  uint16_t trajs_plot = 0;
  for (uint16_t k = 0; k < ad_trajs_m.size(); k++) {
    if (ad_trajs_m[k].plan_traj.is_valid) {
      any_traj_valid = true;
      if (k != best_traj_idx_m) {
        trajs_plot++;
      }
    }
  }

  QVector<double> t_qt(params_alg_m.np);
  memcpy(t_qt.data(), params_alg_m.tvec.data(),
         sizeof(double) * params_alg_m.np);
  uint16_t count_not_best = 0;
  for (uint16_t k = 0; k < ad_trajs_m.size(); k++) {
    if (ad_trajs_m[k].plan_traj.is_valid) {
      // Color and width for lines
      QPen pen_line;
      if (k == best_traj_idx_m) {
        pen_line = QPen(Qt::green, 4);  // selected traj
      } else {
        pen_line = QPen(color_palette_m[(trajs_plot - 1 - count_not_best) %
                                        color_palette_m.size()],
                        2);
        count_not_best++;
      }

      // Position
      QVector<double> traj_qt_pos(params_alg_m.np);
      memcpy(traj_qt_pos.data(), ad_trajs_m[k].plot_pos.data(),
             sizeof(double) * params_alg_m.np);
      custom_plot_m->addGraph(st_map_plot_m->axis(QCPAxis::atBottom),
                              st_map_plot_m->axis(QCPAxis::atLeft));
      custom_plot_m->graph(count_graph)->setPen(pen_line);
      custom_plot_m->graph(count_graph)->setData(t_qt, traj_qt_pos);
      count_graph++;

      // Velocity
      QVector<double> traj_qt_vel(params_alg_m.np - 1);
      memcpy(traj_qt_vel.data(), ad_trajs_m[k].plot_vel.data(),
             sizeof(double) * (params_alg_m.np - 1));
      traj_qt_vel.append(traj_qt_vel.back());
      custom_plot_m->addGraph(vel_plot_m->axis(QCPAxis::atBottom),
                              vel_plot_m->axis(QCPAxis::atLeft));
      custom_plot_m->graph(count_graph)->setPen(pen_line);
      custom_plot_m->graph(count_graph)->setData(t_qt, traj_qt_vel);
      count_graph++;

      //ADDED BY SHRITA to print acc, jrk, mar sizes
      //ROS_INFO("Acc: %0.9ld", ad_trajs_m[k].plot_acc.size());
      //ROS_INFO("Jrk: %0.9ld", ad_trajs_m[k].plot_jrk.size());

      //ADDED BY SHRITA Acceleration
      QVector<double> traj_qt_acc(params_alg_m.np - 2);
      memcpy(traj_qt_acc.data(), ad_trajs_m[k].plot_acc.data(),
             sizeof(double) * (params_alg_m.np - 2));
      traj_qt_acc.append(traj_qt_acc.back());
      traj_qt_acc.append(traj_qt_acc.back());
      custom_plot_m->addGraph(acc_plot_m->axis(QCPAxis::atBottom),
                              acc_plot_m->axis(QCPAxis::atLeft));
      custom_plot_m->graph(count_graph)->setPen(pen_line);
      custom_plot_m->graph(count_graph)->setData(t_qt, traj_qt_acc);
      count_graph++; 

      //ADDED BY SHRITA Jerk
      QVector<double> traj_qt_jrk(params_alg_m.np - 3);
      memcpy(traj_qt_acc.data(), ad_trajs_m[k].plot_acc.data(),
             sizeof(double) * (params_alg_m.np - 3));
      traj_qt_jrk.append(traj_qt_jrk.back());
      traj_qt_jrk.append(traj_qt_jrk.back());
      traj_qt_jrk.append(traj_qt_jrk.back());
      custom_plot_m->addGraph(jrk_plot_m->axis(QCPAxis::atBottom),
                              jrk_plot_m->axis(QCPAxis::atLeft));
      custom_plot_m->graph(count_graph)->setPen(pen_line);
      custom_plot_m->graph(count_graph)->setData(t_qt, traj_qt_jrk);
      count_graph++; 
    }
  }

  custom_plot_m->replot();
  data_updated_m = false;

  auto end = std::chrono::high_resolution_clock::now();
  double time_taken =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
          .count() *
      1e-09;
  if (time_taken > 0.05) {
    std::printf("Time elapsed plotting - plot func: %0.9F\n", time_taken);
  }
}

void PlotterQCustomPlot::PlotPathOverlap() {
  if (!data_path_overlap_m) return;

  if (!secondary_initialized) {
    sec_window_m = std::unique_ptr<QMainWindow>(new QMainWindow());
    custom_plot_sec_m = std::unique_ptr<QCustomPlot>(new QCustomPlot());

    // setup customPlot as central widget of window
    sec_window_m->setCentralWidget(custom_plot_sec_m.get());
    sec_window_m->setGeometry(400, 100, 1000, 1000);

    sec_window_m->show();
    secondary_initialized = true;
  } else {
    custom_plot_sec_m->clearPlottables();
    custom_plot_sec_m->clearItems();
  }

  QCPCurve *curve_plot_path, *curve_plot_car, *curve_plot_sol;

  // Path
  Eigen::VectorXd x_p(path_clipper_m[0].size()), y_p(path_clipper_m[0].size());
  for (uint16_t k = 0; k < path_clipper_m[0].size(); k++) {
    x_p[k] = path_clipper_m[0][k].X;
    y_p[k] = path_clipper_m[0][k].Y;
  }
  QVector<double> x_p_qt(path_clipper_m[0].size()),
      y_p_qt(path_clipper_m[0].size());
  memcpy(x_p_qt.data(), x_p.data(), sizeof(double) * path_clipper_m[0].size());
  memcpy(y_p_qt.data(), y_p.data(), sizeof(double) * path_clipper_m[0].size());

  curve_plot_path =
      new QCPCurve(custom_plot_sec_m->xAxis, custom_plot_sec_m->yAxis);
  QPen path_pen;
  path_pen.setColor(Qt::black);
  path_pen.setStyle(Qt::DotLine);
  curve_plot_path->setPen(path_pen);
  curve_plot_path->setData(x_p_qt, y_p_qt);

  // Car
  double min_x = std::numeric_limits<double>::max(),
         min_y = std::numeric_limits<double>::max(),
         max_x = std::numeric_limits<double>::lowest(),
         max_y = std::numeric_limits<double>::lowest();
  Eigen::VectorXd x_c(5), y_c(5);
  for (uint16_t k = 0; k < 4; k++) {
    x_c[k] = car_clipper_m[0][k].X;
    y_c[k] = car_clipper_m[0][k].Y;
    min_x = std::min(min_x, x_c[k]);
    min_y = std::min(min_y, y_c[k]);
    max_x = std::max(max_x, x_c[k]);
    max_y = std::max(max_y, y_c[k]);
  }
  x_c[4] = x_c[0];
  y_c[4] = y_c[0];
  curve_plot_car =
      new QCPCurve(custom_plot_sec_m->xAxis, custom_plot_sec_m->yAxis);
  curve_plot_car->setPen(QPen(Qt::blue));
  QVector<double> x_c_qt(5), y_c_qt(5);
  memcpy(x_c_qt.data(), x_c.data(), sizeof(double) * 5);
  memcpy(y_c_qt.data(), y_c.data(), sizeof(double) * 5);
  curve_plot_car->setData(x_c_qt, y_c_qt);

  // Sol
  if (sol_clipper_m.size() > 0) {
    Eigen::VectorXd x_int(sol_clipper_m[0].size()),
        y_int(sol_clipper_m[0].size());
    for (uint16_t k = 0; k < sol_clipper_m[0].size(); k++) {
      x_int[k] = sol_clipper_m[0][k].X;
      y_int[k] = sol_clipper_m[0][k].Y;
    }
    curve_plot_sol =
        new QCPCurve(custom_plot_sec_m->xAxis, custom_plot_sec_m->yAxis);
    curve_plot_sol->setPen(QPen(Qt::magenta));
    QVector<double> x_int_qt(sol_clipper_m[0].size()),
        y_int_qt(sol_clipper_m[0].size());
    memcpy(x_int_qt.data(), x_int.data(),
           sizeof(double) * sol_clipper_m[0].size());
    memcpy(y_int_qt.data(), y_int.data(),
           sizeof(double) * sol_clipper_m[0].size());
    curve_plot_sol->setData(x_int_qt, y_int_qt);
  }

  double dim_max = std::max(max_x - min_x, max_y - min_y);
  custom_plot_sec_m->xAxis->setRange(min_x - dim_max, max_x + dim_max);
  custom_plot_sec_m->yAxis->setRange(min_y - dim_max, max_y + dim_max);

  custom_plot_sec_m->replot();

  data_path_overlap_m = false;
}

void PlotterQCustomPlot::PlotDataAndBounds() {
  if (!data_bounds_m) {
    return;
  }

  if (!secondary_initialized) {
    sec_window_m = std::unique_ptr<QMainWindow>(new QMainWindow());
    custom_plot_sec_m = std::unique_ptr<QCustomPlot>(new QCustomPlot());

    // setup customPlot as central widget of window
    sec_window_m->setCentralWidget(custom_plot_sec_m.get());
    sec_window_m->setGeometry(400, 100, 1000, 1000);

    sec_window_m->show();
    secondary_initialized = true;

    timer_m->stop();
  }

  custom_plot_sec_m->xAxis->setLabel("t [s]");
  custom_plot_sec_m->xAxis->setRange(0.0, params_alg_m.plan_t);
  custom_plot_sec_m->xAxis->setSubTicks(false);
  custom_plot_sec_m->yAxis->setLabel("s [m]");
  custom_plot_sec_m->yAxis->setRange(-1.0,
                                     params_alg_m.v_max * params_alg_m.plan_t);
  custom_plot_sec_m->yAxis->setSubTicks(false);

  // Time vector, common to all plots
  uint32_t n_points = uint32_t(params_alg_m.tvec.size() * 2);
  int n_points_int = int(n_points);
  Eigen::VectorXd x(n_points);
  x << params_alg_m.tvec, params_alg_m.tvec.reverse();
  QVector<double> x_qt(2 * params_alg_m.np);
  memcpy(x_qt.data(), x.data(), sizeof(double) * n_points);
  
  custom_plot_sec_m->replot(QCustomPlot::RefreshPriority::rpImmediateRefresh);
  application_m->processEvents();
  sleep(2);

  // BOUNDS
  QVector<double> tvec_qt(params_alg_m.np);
  memcpy(tvec_qt.data(), params_alg_m.tvec.data(),
         sizeof(double) * params_alg_m.np);

  QCPCurve *curve_plot_lb =
      new QCPCurve(custom_plot_sec_m->xAxis, custom_plot_sec_m->yAxis);
  QCPCurve *curve_plot_ub =
      new QCPCurve(custom_plot_sec_m->xAxis, custom_plot_sec_m->yAxis);

  for (uint16_t k = 0; k < lb_vec_m.size(); k++) {
    curve_plot_lb->setPen(QPen(Qt::black, 2));
    curve_plot_ub->setPen(QPen(Qt::red, 2));

    QVector<double> lb_qt(params_alg_m.np);
    memcpy(lb_qt.data(), lb_vec_m[k].data(), sizeof(double) * params_alg_m.np);
    QVector<double> ub_qt(params_alg_m.np);
    memcpy(ub_qt.data(), ub_vec_m[k].data(), sizeof(double) * params_alg_m.np);

    curve_plot_lb->setData(tvec_qt, lb_qt);
    curve_plot_ub->setData(tvec_qt, ub_qt);

    custom_plot_sec_m->replot(QCustomPlot::RefreshPriority::rpImmediateRefresh);
    application_m->processEvents();

    sleep(4);
  }

  data_bounds_m = false;

  timer_m->start(100);
}

void PlotterQCustomPlot::UpdatePathOverlap(const ClipperLib::Paths &car,
                                           const ClipperLib::Paths &path,
                                           const ClipperLib::Paths &sol) {
  car_clipper_m = car;
  path_clipper_m = path;
  sol_clipper_m = sol;

  data_path_overlap_m = true;
}

void PlotterQCustomPlot::UpdateDataAndBounds(
    const std::vector<Eigen::VectorXd> &ub_vec,
    const std::vector<Eigen::VectorXd> &lb_vec) {
  ub_vec_m = ub_vec;
  lb_vec_m = lb_vec;

  data_bounds_m = true;
}

}  // namespace ai4ad
