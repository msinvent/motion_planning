#include <iostream>
#include <vector>
#include "include/hybrid_a_star.h"
#include <fstream>

// TODO : this code need to be modified

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
  
  std::cout << "Finding path through grid:" << "\n";
  
  std::cout<<GRID.size()<<","<<GRID[0].size()<<"\n";

  // Creates an Empty Maze and for testing the number of expansions with it
  for(std::size_t i = 0; i < GRID.size(); ++i) {
	
  std::cout << GRID[i][0];
    for(std::size_t j = 1; j < GRID[0].size(); ++j) {
    	
      std::cout << "," << GRID[i][j];
    }
    
    std::cout << "\n";
  }

  ASTAR astar = ASTAR();

  ASTAR::maze_path get_path = astar.search(GRID,START,GOAL);

  vector<ASTAR::maze_s> show_path = astar.reconstruct_path(get_path.came_from,
                                                       START, get_path.final);

  
  // std::cout << "show path from start to finish" << "\n";
  // for(int i = show_path.size()-1; i >= 0; --i) {
  //     ASTAR::maze_s step = show_path[i];
      
  //     std::cout << "##### step " << step.g << " #####" << "\n";
      
  //     std::cout << "x " << step.x << "\n";
      
  //     std::cout << "y " << step.y << "\n";
      
  //     std::cout << "theta " << step.theta << "\n";
  // }

  return 0;
}
