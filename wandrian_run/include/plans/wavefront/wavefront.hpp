/*
 * wavefront.hpp
 *
 *  Created on: Dec 9, 2015
 *      Author: haininh
 */

#ifndef INCLUDE_PLANS_WAVEFRONT_WAVEFRONT_HPP_
#define INCLUDE_PLANS_WAVEFRONT_WAVEFRONT_HPP_

#include <iostream>       // std::cout
#include <stack>           // std::list
#include <queue>          // std::queue
#include "../../common/environment.hpp"
#include "../base_plan.hpp"
#include "cell.hpp"

#define VISITED		-1
#define UNVISITED	0
#define OBSTACLE	1
#define X			1
#define GOAL		2

// edit this first
#define MAP_X 8
#define MAP_Y 8

 using namespace wandrian::common;

 namespace wandrian {
  namespace plans {
    namespace wavefront {

      class Wavefront: public BasePlan {
      public:
        Wavefront();
        ~Wavefront();
        void initialize(Cell, Cell, double);
        void cover();

      protected:
        bool go_to(PointPtr, bool);
        bool go_by_step(Cell, Cell);

      private:
        std::vector < std::vector<int > > WORLD_MAP;
        std::queue<Cell> path;
        Cell start;
        Cell goal;
        double robot_size;

        void wave_fill();
        bool check_coordinate(Cell);
        int get_cell_value(Cell);

        void path_planning();
        Cell get_next_hop(Cell);
      };

      typedef boost::shared_ptr<Wavefront> WavefrontPtr;
    }
  }
}




#endif /* INCLUDE_PLANS_WAVEFRONT_WAVEFRONT_HPP_ */
