#pragma once
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

namespace wandrian {
	namespace plans {
		namespace wavefront {
			struct Cell {
				int x, y;
			};

			inline bool operator==(const Cell &p1, const Cell &p2) {
				return (p1.x == p2.x) && (p1.y == p2.y);
			}

			inline bool operator!=(const Cell &p1, const Cell &p2) {
				return !(p1 == p2);
			}
		}
	}
}

#endif /* INCLUDE_PLANS_WAVEFRONT_CELL_HPP_ */
