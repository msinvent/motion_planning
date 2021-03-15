#include <iostream>
#include <vector>
#include "include/hybrid_a_star.h"
#include <fstream>

// Sets up maze grid
int X = 1;
int _ = 0;

/**
 * TODO: You can change up the grid maze to test different expansions.
 */
vector<vector<int>> GRID = {
  {_,X,X,_,_,_,_,_,_,_,X,X,_,_,_,_,},
  {_,X,X,_,_,_,_,_,_,X,X,_,_,_,_,_,},
  {_,X,X,_,_,_,_,_,X,X,_,_,_,_,_,_,},
  {_,X,X,_,_,_,_,X,X,_,_,_,X,X,X,_,},
  {_,X,X,_,_,_,X,X,_,_,_,X,X,X,_,_,},
  {_,X,X,_,_,X,X,_,_,_,X,X,X,_,_,_,},
  {_,X,X,_,X,X,_,_,_,X,X,X,_,_,_,_,},
  {_,X,X,X,X,_,_,_,X,X,X,_,_,_,_,_,},
  {_,X,X,X,_,_,_,X,X,X,_,_,_,_,_,_,},
  {_,X,X,_,_,_,X,X,X,_,_,X,X,X,X,X,},
  {_,X,_,_,_,X,X,X,_,_,X,X,X,X,X,X,},
  {_,_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,},
  {_,_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,},
  {_,_,X,X,X,_,_,X,X,X,X,X,X,X,X,X,},
  {_,X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,},
  {X,X,X,_,_,_,_,_,_,_,_,_,_,_,_,_,}};

vector<double> START = {0.0,0.0,0.0};
vector<int> GOAL = {(int)GRID.size()-1, (int)GRID[0].size()-1};

int main() {
  std::ofstream myFile("debugOutput.txt");
  myFile << "Finding path through grid:" << "\n";
  myFile<<GRID.size()<<","<<GRID[0].size()<<"\n";

  // Creates an Empty Maze and for testing the number of expansions with it
  for(std::size_t i = 0; i < GRID.size(); ++i) {
	myFile << GRID[i][0];
    for(std::size_t j = 1; j < GRID[0].size(); ++j) {
    	myFile << "," << GRID[i][j];
    }
    myFile << "\n";
  }

  HBF hbf = HBF();

  HBF::maze_path get_path = hbf.search(GRID,START,GOAL);

  vector<HBF::maze_s> show_path = hbf.reconstruct_path(get_path.came_from,
                                                       START, get_path.final);

  myFile << "show path from start to finish" << "\n";
  for(int i = show_path.size()-1; i >= 0; --i) {
      HBF::maze_s step = show_path[i];
      myFile << "##### step " << step.g << " #####" << "\n";
      myFile << "x " << step.x << "\n";
      myFile << "y " << step.y << "\n";
      myFile << "theta " << step.theta << "\n";
  }

  return 0;
}
