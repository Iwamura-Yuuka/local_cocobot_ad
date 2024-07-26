#include "glocal_path_planner/glocal_path_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "glocal_path_planner");
  GlocalPathPlanner glocal_path_planner;
  glocal_path_planner.process();

  return 0;
}