#include "spaciss_obs_map_creator/spaciss_obs_map_creator.h"

SpacissObsMapCreator::SpacissObsMapCreator():private_nh_("~")
{
  // param
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("obs_map_frame", obs_map_frame_, {"base_footprint"});
  private_nh_.param("map_size", map_size_, {10.0});
  private_nh_.param("map_reso", map_reso_, {0.1});
  private_nh_.param("margin", margin_, {1.0});
  
  // subscriber
  sub_people_states_ = nh_.subscribe("/transformed_people_states", 1, &SpacissObsMapCreator::people_states_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_obs_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/raw_obs_map", 1);

  // --- 基本設定（障害物マップ） ---
  // header
  obs_map_.header.frame_id = obs_map_frame_;
  // info
  obs_map_.info.resolution = map_reso_;
  obs_map_.info.width      = int(round(map_size_/map_reso_));
  obs_map_.info.height     = int(round(map_size_/map_reso_));
  obs_map_.info.origin.position.x = -map_size_/2.0;
  obs_map_.info.origin.position.y = -map_size_/2.0;
  // data
  obs_map_.data.reserve(obs_map_.info.width * obs_map_.info.height);
}

// 歩行者情報のコールバック関数
void SpacissObsMapCreator::people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg)
{
  while(people_states_.size() > 0)
  {
    // people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    people_states_.pop();
  }

  people_states_.emplace(msg);
  flag_people_states_ = true;
}

// マップの初期化(すべて「未知」にする)
void SpacissObsMapCreator::init_map(nav_msgs::OccupancyGrid& map)
{
  map.data.clear();

  // マップサイズを計算
  const int size = map.info.width * map.info.height;

  for(int i=0; i<size; i++)
  {
    map.data.push_back(-1);  //「未知」にする
  }
}

// マップ内の場合、trueを返す
bool SpacissObsMapCreator::is_in_map(nav_msgs::OccupancyGrid& map, const double x, const double y)
{
  const int index_x = int(floor((x - map.info.origin.position.x) / map.info.resolution));
  const int index_y = int(floor((y - map.info.origin.position.y) / map.info.resolution));

  if((index_x >= 0) && (index_x < map.info.width) && (index_y >= 0) && (index_y < map.info.height))
    return true;
  else
    return false;
}

// 座標からグリッドのインデックスを返す
int SpacissObsMapCreator::xy_to_grid_index(nav_msgs::OccupancyGrid& map, const double x, const double y)
{
  const int index_x = int(floor((x - map.info.origin.position.x) / map.info.resolution));
  const int index_y = int(floor((y - map.info.origin.position.y) / map.info.resolution));

  return index_x + (index_y * map.info.width);
}

// 歩行者位置までの距離を計算
double SpacissObsMapCreator::calc_distance(const double person_x, const double person_y, const double x, const double y)
{
  const double dx = x - person_x;
  const double dy = y - person_y;

  return hypot(dx, dy);
}

// マップにコストを割り当てる
void SpacissObsMapCreator::assign_cost_for_person_cost_map(nav_msgs::OccupancyGrid& map, const double x, const double y, const double cost)
{
  const int grid_index = xy_to_grid_index(map, x, y);

  // すでに割り当てられているコストより大きければ更新
  if(map.data[grid_index] < cost)
    map.data[grid_index] = cost;
}

// 歩行者の周りの衝突半径分を占有にする
void SpacissObsMapCreator::expand_obstacle(const double person_x, const double person_y)
{
  // 探索を開始する座標を計算
  const double start_x = person_x - margin_;  // x座標
  const double start_y = person_y - margin_;  // y座標

  // 探索を終了する座標を計算
  const double end_x = person_x + margin_;    // x座標
  const double end_y = person_y + margin_;    // y座標

  for(double x=start_x; x<=end_x; x+=map_reso_)
  {
    for(double y=start_y; y<=end_y; y+=map_reso_)
    {
      const double dist = calc_distance(person_x, person_y, x, y);

      // 衝突半径内であれば占有にする
      if(dist <= margin_)
      {
        if(is_in_map(obs_map_, x, y))
        {
          assign_cost_for_person_cost_map(obs_map_, x, y, 100);
        }
      }
    }
  }
}

// localmapを作成
void SpacissObsMapCreator::create_obs_map()
{
  // マップの初期化
  init_map(obs_map_);

  // 歩行者情報の取得
  const auto people = people_states_.front();

  // 歩行者情報の取得
  for(const auto& person : people->people_states)
  {
    // 歩行者の周りの衝突半径分を占有にする
    expand_obstacle(person.pose.position.x, person.pose.position.y);
  }

  pub_obs_map_.publish(obs_map_);
}

//メイン文で実行する関数
void SpacissObsMapCreator::process()
{
  ros::Rate loop_rate(hz_);

  while(ros::ok())
  {
    if(flag_people_states_)
    {
      create_obs_map();
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_people_states_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}