#ifndef GLOCAL_PATH_PLANNER_H
#define GLOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

// ===== 構造体 =====
struct Node {
  int index_x;
  int index_y;
  int parent_index_x;
  int parent_index_y;
  int cost_map_value;
  double cost;
};

struct Motion {
  double x;
  double y;
  double cost;
};

// ===== クラス =====
class GlocalPathPlanner
{
public:
  GlocalPathPlanner();
  void process();

private:
  // コールバック関数
  void density_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
  void local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg); 

  // 引数あり関数
  double normalize_angle(double theta);            // 適切な角度(-M_PI ~ M_PI)を返す

  // 引数なし関数
  bool is_in_map();                               // local_goalがマップ内の場合、trueを返す
  double calc_direction();                       // local_goalの方位を計算
  void calc_target_goal();                        // target_goal（マップ内のlocal_goal）を計算
  Node get_goal_node();                              // ゴールノードを取得
  void create_glocal_path();                       // glocal_pathを生成
  

  // yamlファイルで設定可能な変数
  bool visualize_target_goal_;  // target_goalを可視化するかの設定用
  int hz_;                      // ループ周波数 [Hz]
  std::string node_frame_;      // 生成するノードのframe_id
  std::string path_frame_;      // 生成するpathのframe_id
  std::string goal_frame_;      // local_goalのframe_id

  // msgの受け取り判定用
  bool flag_density_map_;
  bool flag_local_goal_;

  // ノード関連
  Node start_node_;               // スタートノード
  Node goal_node_;                // ゴールノード
  std::vector<Node> open_set_;    // Openリスト
  std::vector<Node> closed_set_;  // Closeリスト 

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_density_map_;
  ros::Subscriber sub_local_goal_;

  // Publisher
  ros::Publisher pub_glocal_path_;
  ros::Publisher pub_target_goal_;

  // tf
  tf2_ros::Buffer tf_buffer_;

  // 各種オブジェクト
  nav_msgs::OccupancyGrid density_map_;      // density_map
  geometry_msgs::PointStamped local_goal_;   // local_goal
  geometry_msgs::PointStamped target_goal_;  // target_goal
};

#endif  // GLOCAL_PATH_PLANNER_H