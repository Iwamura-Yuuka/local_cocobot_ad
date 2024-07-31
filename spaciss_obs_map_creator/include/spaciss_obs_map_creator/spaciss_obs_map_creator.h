#ifndef SPACISS_OBS_MAP_CREATOR_H
#define SPACISS_OBS_MAP_CREATOR_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

class SpacissObsMapCreator
{
  public:
    SpacissObsMapCreator();
    void process();

  private:
    // コールバック関数
    void people_states_callback(const pedestrian_msgs::PeopleStatesConstPtr& msg);

    // 引数あり関数
    void init_map(nav_msgs::OccupancyGrid& map);                                                                            // マップの初期化(すべて「未知」にする)
    bool is_in_map(nav_msgs::OccupancyGrid& map, const double x, const double y);                                           // マップ内の場合、trueを返す
    int xy_to_grid_index(nav_msgs::OccupancyGrid& map, const double x, const double y);                                     // 座標からグリッドのインデックスを返す
    double calc_distance(const double person_x, const double person_y, const double x, const double y);                     // 歩行者位置までの距離を計算
    void assign_cost_for_person_cost_map(nav_msgs::OccupancyGrid& map, const double x, const double y, const double cost);  // マップにコストを割り当てる
    void expand_obstacle(const double person_x, const double person_y);                                                     // 歩行者の周りの衝突半径分を占有にする

    // 引数なし関数
    void create_obs_map();  // localmapを作成

    // yamlファイルで設定可能な変数
    int hz_;                     // ループ周波数 [Hz]
    std::string obs_map_frame_;  // 障害物マップのframe_id
    double map_size_;            // マップの一辺の長さ [m]
    double map_reso_;            // マップの解像度 [m/cell]
    double margin_;              // 歩行者の周りの衝突半径 [m]

    // msgの受け取り判定用
    bool flag_people_states_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_people_states_;

    // Publisher
    ros::Publisher pub_obs_map_;

    // 各種オブジェクト
    std::queue<pedestrian_msgs::PeopleStatesConstPtr> people_states_;  // 歩行者情報
    nav_msgs::OccupancyGrid obs_map_;                                  // 障害物マップ
};

#endif  // SPACISS_OBS_MAP_CREATOR_H