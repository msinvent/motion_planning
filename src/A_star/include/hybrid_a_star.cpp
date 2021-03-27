#include <math.h>
#include <iostream>
#include <vector>
#include "hybrid_a_star.h"
#include <queue>
#include <fstream>


// Initializes HBF
ASTAR::ASTAR() {}

ASTAR::~ASTAR() {}

int ASTAR::theta_to_stack_number(double theta){
  // Takes an angle (in radians) and returns which "stack" in the 3D
  //   configuration space this angle corresponds to. Angles near 0 go in the
  //   lower stacks while angles near 2 * pi go in the higher stacks.
  double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
  int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI)))
                   % NUM_THETA_CELLS;

  return stack_number;
}

int ASTAR::idx(double float_num) {
  // Returns the index into the grid for continuous position. So if x is 3.621,
  //   then this would return 3 to indicate that 3.621 corresponds to array
  //   index 3.
  return int(floor(float_num));
}

double ASTAR::heuristic(ASTAR::maze_s &state, const std::vector<int>& goal) const{
  return (state.x-goal[0])*(state.x-goal[0]) + (state.y-goal[1])*(state.y-goal[1]);
}

double ASTAR::heuristic(ASTAR::maze_s &state1, ASTAR::maze_s &state2) const{
  return (state1.x-state2.x)*(state1.x-state2.x) + (state1.y-state2.y)*(state1.y-state2.y);
  // return 0.0;
}

vector<ASTAR::maze_s> ASTAR::expand(ASTAR::maze_s &state, const std::vector<int>& goal) {
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;

  int g2 = g+1;
  vector<ASTAR::maze_s> next_states;

  for(double delta_i = minSteering; delta_i < maxSteering; delta_i+=steeringResolution) {
    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;

    if(theta2 < 0) {
      theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta2);
    double y2 = y + SPEED * sin(theta2);
    
    ASTAR::maze_s state2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    // state2.g = g + heuristic(state, state2);
    state2.g = g + 1;
    // state2.f = g2 + heuristic(state2, goal) - 1.0*abs(delta_i); // zig-zag path
    state2.f = state2.g + heuristic(state2, goal) + weight_direction*abs(delta_i);
    next_states.push_back(state2);
  }

  std::cout<<"Expanded "<<next_states.size()<<" elements\n";

  return next_states;
}

vector< ASTAR::maze_s> ASTAR::reconstruct_path(
  vector<vector<vector<ASTAR::maze_s>>> &came_from, vector<double> &start,
  ASTAR::maze_s &final) {

  vector<maze_s> path = {final};

  int stack = theta_to_stack_number(final.theta);

  maze_s current = came_from[stack][idx(final.x)][idx(final.y)];

  stack = theta_to_stack_number(current.theta);

  double x = current.x;
  double y = current.y;

  while(x != start[0] || y != start[1]) {
    path.push_back(current);
    current = came_from[stack][idx(x)][idx(y)];
    x = current.x;
    y = current.y;
    stack = theta_to_stack_number(current.theta);
  }

  return path;
}

ASTAR::maze_path ASTAR::search(vector< vector<int> > &grid, vector<double> &start,
                           vector<int> &goal) {

  std::ofstream mycout("astarsearch.txt");

  // Working Implementation of breadth first search. Does NOT use a heuristic
  //   and as a result this is pretty inefficient. Try modifying this algorithm
  //   into hybrid A* by adding heuristics appropriately.

  /**
   * TODO: Add heuristics and convert this function into hybrid A*
   */
  vector<vector<vector<int>>> closed(
    NUM_THETA_CELLS, vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
  vector<vector<vector<maze_s>>> came_from(
    NUM_THETA_CELLS, vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;

  maze_s state;
  state.g = g;
  state.f = g;
  state.x = start[0];
  state.y = start[1];
  state.theta = theta;

  closed[stack][idx(state.x)][idx(state.y)] = 1;
  came_from[stack][idx(state.x)][idx(state.y)] = state;
  int total_closed = 1;
  // vector<maze_s> opened = {state};
  std::priority_queue<maze_s, std::vector<maze_s>, std::greater<maze_s>> opened;
  opened.push(state);


  static bool printOnce{true};
  if(printOnce){
    mycout<<"X"<<","<<"Y"<<","<<"theta"<<","<<"g"<<","<<"f"<<"\n";
    printOnce = false;
  }
  mycout<<state.x<<","<<state.y<<","<<state.theta<<","<<state.g<<","<<state.f<<"\n";

  bool finished = false;
  while(!opened.empty()) {
    maze_s current = opened.top(); //grab first elment
    opened.pop();

    int x = current.x;
    int y = current.y;

    if(idx(x) == goal[0] && idx(y) == goal[1]) {
      std::cout << "found path to goal in " << total_closed << " expansions"
                << std::endl;
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;

      return path;
    }

    vector<maze_s> next_state = expand(current, goal);

    for(int i = 0; i < next_state.size(); ++i) {
      int g2 = next_state[i].g;
      double f2 = next_state[i].f;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;

      if((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
        // invalid cell
        continue;
      }

      int stack2 = theta_to_stack_number(theta2);

      if(closed[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0) {
        opened.push(next_state[i]);
        mycout<<next_state[i].x<<","<<next_state[i].y<<","<<next_state[i].theta<<","
        <<next_state[i].g<<","<<next_state[i].f<<"\n";
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        ++total_closed;
      }
    }
  }

  std::cout << "no valid path." << std::endl;
  ASTAR::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;

  return path;
}
