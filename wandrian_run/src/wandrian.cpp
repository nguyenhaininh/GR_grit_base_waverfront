/*
 * wandrian.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: sontd
 */

#include "../include/wandrian.hpp"
#include "../include/plans/wavefront/wavefront.hpp"

#define CLOCKWISE true
#define COUNTERCLOCKWISE false

using namespace wandrian::plans::wavefront;

// TODO: Choose relevant epsilon values
#define EPS_ORI_TO_ROTATE 0.06
#define EPS_ORI_TO_MOVE 4 * EPS_ORI_TO_ROTATE
#define EPS_POS 0.06

namespace wandrian {

bool Wandrian::initialize() {
  return core.initialize();
}

void Wandrian::spin() {
  core.set_behavior_run(boost::bind(&Wandrian::wandrian_run, this));
  core.spin();
}

void Wandrian::wandrian_run() {
  if (core.get_plan_name() == "wavefront") {
    WavefrontPtr wavefront = WavefrontPtr(new Wavefront());
    Cell start, goal;
    start.x = 0; start.y = 0;
    goal.x = 7; goal.y = 7;
    wavefront->initialize(start, goal, core.get_robot_size());
    wavefront->set_behavior_go_to(
        boost::bind(&Wandrian::wavefront_go_to, this, _1, _2));
    return wavefront->cover();
  }
}

bool Wandrian::wavefront_go_to(PointPtr position, bool flexibly) {
  return go_to(position, flexibly);
}

bool Wandrian::spiral_stc_see_obstacle(VectorPtr orientation, double step) {
  // TODO: Correctly check whether obstacle is near or not
  double angle = *orientation ^ *core.get_current_orientation();
  return
      (std::abs(angle) <= M_PI_4) ?
          core.get_obstacles()[IN_FRONT] :
          ((angle > M_PI_4) ?
              core.get_obstacles()[AT_LEFT_SIDE] :
              core.get_obstacles()[AT_RIGHT_SIDE]);
}

bool Wandrian::go_to(PointPtr new_position, bool flexibly) {
  bool forward;
  forward = rotate_to(new_position, flexibly);
  go(forward);
  while (true) {
    // Check current_position + k * current_orientation == new_position
    Vector direction_vector = (*new_position - *core.get_current_position())
        / (*new_position % *core.get_current_position());
    if (forward ?
        (!(std::abs(direction_vector.x - core.get_current_orientation()->x)
            < EPS_ORI_TO_MOVE
            && std::abs(direction_vector.y - core.get_current_orientation()->y)
                < EPS_ORI_TO_MOVE)) :
        (!(std::abs(direction_vector.x + core.get_current_orientation()->x)
            < EPS_ORI_TO_MOVE
            && std::abs(direction_vector.y + core.get_current_orientation()->y)
                < EPS_ORI_TO_MOVE))) {
      core.stop();
      forward = rotate_to(new_position, flexibly);
      go(forward);
    }

    if (!core.get_obstacles()[IN_FRONT]) {
      if (std::abs(new_position->x - core.get_current_position()->x) < EPS_POS
          && std::abs(new_position->y - core.get_current_position()->y)
              < EPS_POS) {
        core.stop();
        break;
      }
    } else {
      // Obstacle
      core.stop();
      return false;
    }
  }
  return true;
}

bool Wandrian::rotate_to(PointPtr new_position, bool flexibly) {
  VectorPtr new_orientation = VectorPtr(
      new Vector(
          (*new_position - *core.get_current_position())
              / (*new_position % *core.get_current_position())));
  double angle = *new_orientation ^ *core.get_current_orientation();

  bool will_move_forward = !flexibly ? true : std::abs(angle) < M_PI_2;
  if (angle > EPS_ORI_TO_ROTATE)
    rotate(will_move_forward ? COUNTERCLOCKWISE : CLOCKWISE);
  else if (angle < -EPS_ORI_TO_ROTATE)
    rotate(will_move_forward ? CLOCKWISE : COUNTERCLOCKWISE);
  while (true) {
    if (will_move_forward ?
        (std::abs(new_orientation->x - core.get_current_orientation()->x)
            < EPS_ORI_TO_ROTATE
            && std::abs(new_orientation->y - core.get_current_orientation()->y)
                < EPS_ORI_TO_ROTATE) :
        (std::abs(new_orientation->x + core.get_current_orientation()->x)
            < EPS_ORI_TO_ROTATE
            && std::abs(new_orientation->y + core.get_current_orientation()->y)
                < EPS_ORI_TO_ROTATE)) {
      core.stop();
      break;
    }
  }
  return will_move_forward;
}

void Wandrian::go(bool forward) {
  double linear_vel_step = core.get_linear_velocity_step();
  core.set_linear_velocity(forward ? linear_vel_step : -linear_vel_step);
}

void Wandrian::rotate(bool clockwise) {
  double angular_vel_step = core.get_angular_velocity_step();
  core.set_angular_velocity(clockwise ? -angular_vel_step : angular_vel_step);
}

}
