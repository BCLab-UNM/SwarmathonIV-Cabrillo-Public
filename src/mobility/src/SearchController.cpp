#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  wanderDist = 2;
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search() {
  geometry_msgs::Pose2D goalLocation;

  //select new heading from Gaussian distribution around current heading
  //goalLocation.theta = rng->gaussian(currentLocation.theta, 0.25);
  goalLocation.theta = rng->gaussian(0, 0.25);

  //select new position 50 cm from current location
  //goalLocation.x = currentLocation.x + (wanderDist * cos(goalLocation.theta));
  //oalLocation.y = currentLocation.y + (wanderDist * sin(goalLocation.theta));

  goalLocation.x = wanderDist * cos(goalLocation.theta);
  goalLocation.y = wanderDist * sin(goalLocation.theta);


  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  //double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);
  //double remainingGoalDist = hypot(oldGoalLocation.x, oldGoalLocation.y);
  //this of course assumes random walk continuation. Change for diffrent search methods.

  //newGoalLocation.theta = oldGoalLocation.theta;
  //newGoalLocation.x = currentLocation.x + (wanderDist * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  //newGoalLocation.y = currentLocation.y + (wanderDist * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
  //newGoalLocation.x = (wanderDist * cos(oldGoalLocation.theta));
  //newGoalLocation.y = (wanderDist * sin(oldGoalLocation.theta));
  return search();

}
