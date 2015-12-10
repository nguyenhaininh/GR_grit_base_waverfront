/*
 * cell.hpp
 *
 *  Created on: Dec 10, 2015
 *      Author: haininh
 */

#ifndef INCLUDE_PLANS_WAVEFRONT_CELL_HPP_
#define INCLUDE_PLANS_WAVEFRONT_CELL_HPP_

#include <stdlib.h>
#include <cmath>
#include <limits>
#include <boost/shared_ptr.hpp>

namespace wandrian {
namespace plans {
namespace wavefront {
struct Cell {

  int x, y;

  Cell(int, int);
  Cell(const Cell&);

};

typedef boost::shared_ptr<Cell const> CellConstPtr;
typedef boost::shared_ptr<Cell> CellPtr;

inline bool operator!=(const Cell &p1, const Cell &p2) {
  return !(p1 == p2);
}

inline bool operator==(const Cell &p1, const Cell &p2) {
  return (p1.x == p2.x) &&(p1.y == p2.y);
}

struct CellComp {
  bool operator()(CellConstPtr p1, CellConstPtr p2) const {
    return *p1 < *p2;
  }
};

}
}
}


#endif /* INCLUDE_PLANS_WAVEFRONT_CELL_HPP_ */
