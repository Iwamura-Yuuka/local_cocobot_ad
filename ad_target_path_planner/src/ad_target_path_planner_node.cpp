#include "ad_target_path_planner/ad_target_path_planner.h"

//メイン関数
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ad_target_path_planner");
  AdTargetPathPlanner adtargetpathplanner;
  adtargetpathplanner.process();

  return 0;
}