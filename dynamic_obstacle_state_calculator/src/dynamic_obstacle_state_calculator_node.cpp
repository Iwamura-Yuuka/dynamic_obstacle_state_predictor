#include "dynamic_obstacle_state_calculator/dynamic_obstacle_state_calculator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dynamic_obstacle_state_calculator");
  DynamicObstacleStateCalculator dynamicobstaclestatecalculator;
  dynamicobstaclestatecalculator.process();

  return 0;
}