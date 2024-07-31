#include "spaciss_obs_map_creator/spaciss_obs_map_creator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "spaciss_obs_map_creator");
  SpacissObsMapCreator spacissobsmapcreator;
  spacissobsmapcreator.process();

  return 0;
}