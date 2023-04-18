#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <aux_lib/struct_defs.h>
#include <osqp/osqp.h>
#include <stopline_test/qp_problem.h>

QPProblem::QPProblem(const ai4ad::struct_params_alg params_alg) {
  params_alg_m = params_alg;
  InitProblem(params_alg);
}

void QPProblem::InitProblem(const ai4ad::struct_params_alg params_alg) {

  // Fix bounds
  lb_m = Eigen::VectorXd(params_alg.total_np);
  lb_m << Eigen::VectorXd::Constant(params_alg.np,
                                    -std::numeric_limits<double>::epsilon()),
      Eigen::VectorXd::Zero(params_alg.np - 1),
      Eigen::VectorXd::Constant(params_alg.np - 2, params_alg.acc_min),
      Eigen::VectorXd::Constant(params_alg.np - 3, params_alg.jrk_min);

  ub_m = Eigen::VectorXd(params_alg.total_np);
  ub_m << Eigen::VectorXd::Constant(params_alg.np, 400.0),
      Eigen::VectorXd::Constant(params_alg.np - 1, params_alg.v_max_sbc),
      Eigen::VectorXd::Constant(params_alg.np - 2, params_alg.acc_max),
      Eigen::VectorXd::Constant(params_alg.np - 3, params_alg.jrk_max);

  // QUADRATIC (H) AND LINEAR TERMS (f) FOR OPTIMIZATION
  double wa = params_alg.wa / std::max(std::abs(params_alg.acc_min),
                                       std::abs(params_alg.acc_max));
  double wj = params_alg.wj / std::max(std::abs(params_alg.jrk_min),
                                       std::abs(params_alg.jrk_max));
  double wf = params_alg.wf;

  // Obstacle free
  std::vector<Trip_d> H_trip(params_alg.total_np);
  for (uint16_t k = 0; k < params_alg.total_np; ++k) {
    if (k >= params_alg.pos_idxs(0) &&
        k <= params_alg.pos_idxs(Eigen::placeholders::last)) {
      H_trip[k] = Trip_d(k, k, 0.0);
    }

    if (k >= params_alg.vel_idxs(0) &&
        k <= params_alg.vel_idxs(Eigen::placeholders::last)) {
      H_trip[k] = Trip_d(k, k, 0.0);
    }

    if (k >= params_alg.acc_idxs(0) &&
        k <= params_alg.acc_idxs(Eigen::placeholders::last)) {
      H_trip[k] = Trip_d(k, k, wa);
    }

    if (k >= params_alg.jrk_idxs(0) &&
        k <= params_alg.jrk_idxs(Eigen::placeholders::last)) {
      H_trip[k] = Trip_d(k, k, wj);
    }
  }

  H_free_m.resize(params_alg.total_np, params_alg.total_np);
  H_free_m.setFromTriplets(H_trip.begin(), H_trip.end());
  H_free_m.makeCompressed();

  Eigen::VectorXd f = Eigen::VectorXd::Zero(params_alg.total_np);
  f(params_alg.pos_idxs(0)) = wf;
  f(params_alg.pos_idxs(Eigen::placeholders::last)) = -wf;
  f_free_m = f;

  // Constraints QP optimization initialization
  Eigen::MatrixXd Aeq =
      Eigen::MatrixXd::Zero(2 * params_alg.total_np, params_alg.total_np);
  Aeq(params_alg.pos_idxs(0), params_alg.pos_idxs(0)) = 1;  // init pos
  Aeq(params_alg.vel_idxs(0), params_alg.vel_idxs(0)) = 1;  // init vel
  if (params_alg.match_accel || params_alg.use_prev_accel) {
    Aeq(params_alg.acc_idxs(0), params_alg.acc_idxs(0)) = 1;  // init acc
  }
  for (int i = 1; i < params_alg.np; i++) {
    // position
    Aeq(params_alg.pos_idxs(i), params_alg.pos_idxs(i - 1)) = -1.0;
    Aeq(params_alg.pos_idxs(i), params_alg.pos_idxs(i)) = 1.0;
    Aeq(params_alg.pos_idxs(i), params_alg.vel_idxs(i - 1)) = -params_alg.dt;

    if (i <= params_alg.np - 2) {
      // speed
      Aeq(params_alg.vel_idxs(i), params_alg.vel_idxs(i - 1)) = -1.0;
      Aeq(params_alg.vel_idxs(i), params_alg.vel_idxs(i)) = 1.0;
      Aeq(params_alg.vel_idxs(i), params_alg.acc_idxs(i - 1)) = -params_alg.dt;
    }

    if (i <= params_alg.np - 3) {
      // acceleration
      Aeq(params_alg.acc_idxs(i), params_alg.acc_idxs(i - 1)) = -1.0;
      Aeq(params_alg.acc_idxs(i), params_alg.acc_idxs(i)) = 1.0;
      Aeq(params_alg.acc_idxs(i), params_alg.jrk_idxs(i - 1)) = -params_alg.dt;
    }
  }

  Aeq.block(params_alg.total_np, 0, params_alg.total_np, params_alg.total_np) =
      Eigen::MatrixXd::Identity(params_alg.total_np, params_alg.total_np);

  Aeq_free_m = Aeq.sparseView();
  Aeq_free_m.makeCompressed(); 
}

bool QPProblem::SolveQPProblem(const Eigen::VectorXd &lb_pos,
                               const Eigen::VectorXd &ub_pos,
                               ai4ad::Trajectory &traj_optim,
                               ai4ad::StoplineBreak &stopalgo_struct) {
  Eigen::SparseMatrix<double, Eigen::RowMajor> H;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq;
  Eigen::VectorXd f;

  Eigen::VectorXd lb_state = lb_m;
  Eigen::VectorXd ub_state = ub_m;

  // Save the initial state
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(params_alg_m.total_np);
  beq(params_alg_m.pos_idxs(0)) = ad_sol_m.pos;
  beq(params_alg_m.vel_idxs(0)) = ad_sol_m.vel;
  if (params_alg_m.match_accel || params_alg_m.use_prev_accel) {
    beq(params_alg_m.acc_idxs(0)) = ad_sol_m.accel;
  }

  //Set final H,A,f,ub,lb    
  H = H_free_m;
  Aeq = Aeq_free_m;
  f = f_free_m;
  ub_state(params_alg_m.pos_idxs) = ub_pos;
  lb_state(params_alg_m.pos_idxs) = lb_pos;

  //Set special bounds on v for Stopline case - added by shrita
  if(stopalgo_struct.NearStopline){
    ROS_INFO("Debug v_ubsize %ld", ub_state(params_alg_m.vel_idxs).size());
    for(int i=0; i<params_alg_m.np; i++){
      if(ub_state(params_alg_m.pos_idxs(i))>stopalgo_struct.stopline_dist_m){
        ub_state(params_alg_m.pos_idxs(i))=stopalgo_struct.stopline_dist_m;
      }
      if(i<params_alg_m.np-1){
        ub_state(params_alg_m.vel_idxs(i))=stopalgo_struct.vmax_stopline(i);
      }
    }
    ROS_INFO("Debug vmax size %ld", stopalgo_struct.vmax_stopline.size());
  } 

  // Joint bounds
  Eigen::VectorXd lb_full(params_alg_m.total_np * 2),
      ub_full(params_alg_m.total_np * 2);
  lb_full << beq.array(), lb_state;
  ub_full << beq.array(), ub_state;

  double tolerance_sol = 1e-02;

  // OSQP Settings
  sqp_struct.qp_var->settings()->setVerbosity(true);
  sqp_struct.qp_var->settings()->setMaxIteration(
      5 * (params_alg_m.total_np + 2 * params_alg_m.total_np));  // 5*(nV+nC);
  sqp_struct.qp_var->settings()->setTimeLimit(0.07);
  // sqp_struct.qp_var->settings()->setTimeLimit(0);
  sqp_struct.qp_var->settings()->setWarmStart(true);
  sqp_struct.qp_var->settings()->setAbsoluteTolerance(1e-05);
  sqp_struct.qp_var->settings()->setRelativeTolerance(1e-05);

  // OSQP variables
  sqp_struct.qp_var->data()->setNumberOfVariables(params_alg_m.total_np);
  sqp_struct.qp_var->data()->setNumberOfConstraints(2 *
                                                      params_alg_m.total_np);
  bool valid_hess = sqp_struct.qp_var->data()->setHessianMatrix(H);
  bool valid_grad = sqp_struct.qp_var->data()->setGradient(f);
  bool valid_const =
      sqp_struct.qp_var->data()->setLinearConstraintsMatrix(Aeq);
  bool valid_bounds = sqp_struct.qp_var->data()->setBounds(lb_full, ub_full);

  // One time initialization
  bool valid_init = sqp_struct.qp_var->initSolver();

  if (!valid_hess || !valid_grad || !valid_const || !valid_bounds ||
      !valid_init) {
    ROS_WARN("Incorrect data setting QP problem. Cannot run optimizer");
    return false;
  }

  // Run the solver
  OsqpEigen::ErrorExitFlag ret;
  ret = sqp_struct.qp_var->solveProblem();
  OsqpEigen::Status status = sqp_struct.qp_var->getStatus();

  bool valid_problem = ret == OsqpEigen::ErrorExitFlag::NoError;
  bool valid_status = status == OsqpEigen::Status::Solved ||
                      status == OsqpEigen::Status::SolvedInaccurate ||
                      status == OsqpEigen::Status::MaxIterReached ||
                      status == OsqpEigen::Status::TimeLimitReached;

  if (!valid_problem || !valid_status) {
    return false;
  } else {
    // Save the data onto the solution vector to retrieve later
    Eigen::VectorXd aux_sol = sqp_struct.qp_var->getSolution();
    traj_optim.pos = aux_sol(params_alg_m.pos_idxs);
    traj_optim.vel = aux_sol(params_alg_m.vel_idxs);
    traj_optim.accel = aux_sol(params_alg_m.acc_idxs);
    traj_optim.jerk = aux_sol(params_alg_m.jrk_idxs);

    // Problem verification
    bool pos_lims_init, vel_lims_init, acc_lims_init, Aeq_x, low_limit,
        upp_limit, good_sol;

    pos_lims_init =
        std::abs(beq(params_alg_m.pos_idxs(0)) -
                 aux_sol(params_alg_m.pos_idxs(0))) <= tolerance_sol;

    vel_lims_init =
        std::abs(beq(params_alg_m.vel_idxs(0)) -
                 aux_sol(params_alg_m.vel_idxs(0))) <= tolerance_sol;

    if (params_alg_m.match_accel || params_alg_m.use_prev_accel) {
      acc_lims_init =
          std::abs(beq(params_alg_m.acc_idxs(0)) -
                   aux_sol(params_alg_m.acc_idxs(0))) <= tolerance_sol;
    } else {
      acc_lims_init = true;
    }

    Eigen::VectorXd aux_Aeq_b =
        Aeq.block(0, 0, params_alg_m.total_np, params_alg_m.total_np) *
            aux_sol -
        beq;
    Aeq_x = (aux_Aeq_b.cwiseAbs()).maxCoeff() <= tolerance_sol;

    upp_limit = (aux_sol - ub_state).maxCoeff() <= tolerance_sol;

    low_limit = (aux_sol - lb_state).minCoeff() >= -tolerance_sol;

    good_sol = pos_lims_init && vel_lims_init && acc_lims_init && Aeq_x &&
               low_limit && upp_limit;

    if (good_sol) {
      if (traj_optim.pos.size() < params_alg_m.np) {
        ROS_ERROR("Solution does not have enough points");
        return false;
      }
      return true;
    } else {
      return false;
    }
  }
}