#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include <vector>

using std::vector;

class ASTAR {
 public:
  // Constructor
  ASTAR();

  // Destructor
  virtual ~ASTAR();

  // HBF structs
  struct maze_s {
    int g;  // cost to reach from start g(0) = g(-1) + 1
    double f;  // number maintain priority queue (g + h(x,y))
    double x;
    double y;
    double theta;
    bool operator>(const maze_s& rhs) const{
      return this->f > rhs.f;
    }
  };

  struct maze_path {
    vector<vector<vector<int>>> closed;
    vector<vector<vector<maze_s>>> came_from;
    maze_s final;
  };

  // HBF functions
  int theta_to_stack_number(double theta);

  int idx(double float_num);

  double heuristic(maze_s &state, const std::vector<int>& goal) const;
  double heuristic(maze_s &state1, maze_s &state2) const;

  vector<maze_s> expand(maze_s &state, const std::vector<int>& goal);

  vector<maze_s> reconstruct_path(vector<vector<vector<maze_s>>> &came_from,
                                  vector<double> &start, ASTAR::maze_s &final);

  maze_path search(vector<vector<int>> &grid, vector<double> &start,
                   vector<int> &goal);

 private:
  const int NUM_THETA_CELLS = 180;
  const double SPEED = 0.8;
  const double LENGTH = 0.5;
  const double maxSteering = 15.0;
  const double minSteering = -15.0;
  const double steeringResolution = 1.0;
  const double weight_direction = 10.0;
};

#endif  // HYBRID_A_STAR_H_
