/**
 * @file st_cell_planner.h
 * @brief The class of STCellPlanner.
 */

#ifndef ST_CELL_PLANNER_H
#define ST_CELL_PLANNER_H

#include <aux_lib/st_utils.h>

namespace ai4ad {

/**
 * @class STCellPlanner
 * @brief Implementation of the core planner in the STGraph. It will generate a
 * several candidate paths in the STGraph for the optimizer.
 */
class STCellPlanner {
 public:
  STCellPlanner();

  /**
   *
   * @brief Construct a STCell Planner with given occupied areas on a ST-Graph.
   * @param occupied_cells: All occupied cells before sorting them
   * @param s_range: Specify the size of the ST-Graph in s axis.
   * @param timeline: The time vector
   */
  STCellPlanner(const std::vector<std::vector<STCell>> &occupied_cells,
                const double &s_range, const Eigen::VectorXd &timeline)
      : occupied_cells_(occupied_cells),
        s_range_(s_range),
        timeline_(timeline) {}

  /**
   * @brief Search for all viable paths connecting the origin to the right end
   *        using Breadth-First-Search algorithm.
   * @return A vector of STCell series.
   */
  std::vector<std::vector<STCell>> SearchCandidatePlans();

 private:
  double s_range_;
  Eigen::VectorXd timeline_;
  std::vector<std::vector<STCell>> occupied_cells_;
  std::vector<std::vector<STCell>> viable_cells_;

  /**
   * @brief Generate occupied st_cells for each time interval from a given
   * st_graph
   * @return void
   */
  void ComputeOccupiedSTCells();

  //
  /**
   * @brief Generate viable st_cells for each time interval from a given
   * st_graph.
   * @return void
   */
  void ComputeViableSTCells();
};

}  // namespace ai4ad

#endif  // ST_CELL_PLANNER_H
