/**
 * @file st_cell_planner.cpp
 * @brief STCellPlanner class definition.
 **/

#include <aux_lib/st_cell_planner.h>

namespace ai4ad {

bool compareSTCell(STCell a, STCell b) {
  if (a.min_s == b.min_s) return a.max_s < b.max_s;
  return a.min_s < b.min_s;
}

bool overlap(STCell a, STCell b) {
  if (a.max_s < b.min_s || a.min_s > b.max_s) {
    return false;
  }
  return true;
}

// generate occupied st_cells for each time interval from a given st_graph
void STCellPlanner::ComputeOccupiedSTCells() {
  for (int i = 0; i < occupied_cells_.size(); i++) {
    std::vector<STCell> cells = occupied_cells_[i];
    if (cells.empty()) continue;
    sort(cells.begin(), cells.end(), compareSTCell);
    std::vector<STCell> new_cells;
    new_cells.push_back(cells[0]);
    for (int i = 1; i < cells.size(); i++) {
      if (cells[i].min_s <= new_cells.back().max_s) {
        if (cells[i].max_s >= new_cells.back().max_s) {
          new_cells.back().max_s = cells[i].max_s;
          new_cells.back().max_s_agt_id = cells[i].max_s_agt_id;
        }
      } else {
        new_cells.push_back(cells[i]);
      }
    }
    occupied_cells_[i] = new_cells;
  }
}

// generate viable st_cells for each time interval from a given st_graph
void STCellPlanner::ComputeViableSTCells() {
  viable_cells_.resize(timeline_.size());
  uint32_t s_range_id = std::numeric_limits<uint32_t>::max();
  for (int i = 0; i < viable_cells_.size(); i++) {
    double prev_border = -std::numeric_limits<double>::epsilon();
    uint32_t prev_id = std::numeric_limits<uint32_t>::max();
    std::vector<STCell> cells = occupied_cells_[i];
    sort(cells.begin(), cells.end(), compareSTCell);
    for (STCell cell : cells) {
      viable_cells_[i].push_back(
          STCell(prev_border, cell.min_s, prev_id, cell.min_s_agt_id));
      prev_border = cell.max_s;
      prev_id = cell.max_s_agt_id;
    }
    viable_cells_[i].push_back(
        STCell(prev_border, s_range_, prev_id, s_range_id));
  }
}

// core function
// BFS
std::vector<std::vector<STCell>> STCellPlanner::SearchCandidatePlans() {
  // Search for viable paths connecting viable cells
  std::vector<std::vector<STCell>> candidate_plans;

  ComputeOccupiedSTCells();

  ComputeViableSTCells();

  if (viable_cells_[0].size() > 1) viable_cells_[0] = {viable_cells_[0][0]};
  candidate_plans.push_back(viable_cells_[0]);
  for (int i = 1; i < viable_cells_.size(); i++) {
    std::vector<std::vector<STCell>> temp;
    for (std::vector<STCell> plan : candidate_plans) {
      STCell prev_cell = plan.back();

      std::vector<STCell> cells = viable_cells_[i];
      sort(cells.begin(), cells.end(), compareSTCell);
      for (STCell cell : cells) {
        STCell aux_cell = cell;
        if (overlap(prev_cell, cell)) {
          if (prev_cell.min_s > cell.min_s) {
            aux_cell.min_s = prev_cell.min_s;
            aux_cell.min_s_agt_id = prev_cell.min_s_agt_id;
          }
          plan.push_back(aux_cell);
          temp.push_back(plan);
          plan.pop_back();
        }
      }
    }
    candidate_plans = temp;
  }

  std::vector<std::vector<STCell>> new_candidate_plans;
  for (int i = 0; i < candidate_plans.size(); i++) {
    std::vector<STCell> plan = candidate_plans[i];
    bool valid = true;
    for (int j = 0; j < plan.size(); j++) {
      if (plan[j].max_s - plan[j].min_s < 0.001) {
        valid = false;
        break;
      }
    }
    if (valid) new_candidate_plans.push_back(plan);
  }

  return new_candidate_plans;
}

}  // namespace ai4ad
