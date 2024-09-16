#ifndef AD_TARGET_PATH_PLANNER_H
#define AD_TARGET_PATH_PLANNER_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

// ===== 構造体 =====
struct State
{
  double x;  // [m]
  double y;  // [m]
  double yaw; // [rad]
};

// ===== クラス =====
class AdTargetPathPlanner
{
public:
  AdTargetPathPlanner();
  void process();

private:
  // コールバック関数
  void cost_map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
  void local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg);
  void glocal_path_callback(const nav_msgs::PathConstPtr& msg);

  // 引数あり関数
  bool is_goal_check(const double x, const double y);                                  // pathの終端がlocal_goalから一定距離内に到達したか確認
  bool is_finish_check(const double x, const double y);                                // pathの終端がロボットから一定距離離れたか確認
  double calc_evaluation(const std::vector<State>& traj);                              // 評価関数を計算する
  double calc_heading_eval(const std::vector<State>& traj);                            // heading（1項目）の評価関数を計算する
  double calc_glocal_heading_eval(const std::vector<State>& traj);                     // glocal_heading（2項目）の評価関数を計算する
  double calc_cost_map_eval(const std::vector<State>& traj);                           // cost_map（3項目）の評価関数を計算する
  double normalize_angle(double theta);                                                // 適切な角度(-M_PI ~ M_PI)を返す
  int xy_to_grid_index(const double x, const double y);                                // 座標からグリッドのインデックスを返す
  void search_node(const double max_rad, std::vector<State>& nodes);                   // 次のノードを探索
  void create_path(const double max_rad);                                              // 目標軌道を生成
  void transform_node_to_path(const std::vector<State>& nodes, nav_msgs::Path& path);  // ノード情報からパスを生成

  // 引数なし関数
  double calc_rad_with_steer();       // 機構的制約内で旋回可能な角度を計算(ステアあり)
  double calc_rad_no_steer();         // 機構的制約内で旋回可能な角度を計算(ステアなし)
  double calc_dist_to_glocal_goal();  // glocal_goalまでの距離を計算
  void update_glocal_goal();          // glocal_goalを更新

  // yamlファイルで設定可能な変数
  int hz_;                             // ループ周波数 [Hz]
  std::string node_frame_;             // 生成するノードのframe_id
  std::string path_frame_;             // 生成するpathのframe_id
  std::string goal_frame_;             // local_goalのframe_id
  double goal_tolerance_;              // local_goal_に対する許容誤差 [m]
  double finish_dist_;                 // ロボットからノード先端までの距離 [m]
  double dist_to_update_glocal_goal_;  // glocal_goalを更新する距離 [m]
  double max_vel_;                     // 最高並進速度 [m/s]
  double max_yawrate_;                 // 最高旋回速度 [rad/s]
  double max_speed_;                   // タイヤの最高回転速度 [m/s]
  double speed_reso_;                  // 速度を探索する際の刻み幅 [m/s]
  double max_steer_angle_;             // ステア角の最大値 [deg]
  double steer_angle_reso_;            // ステア角を探索するときの刻み幅 [deg]
  double tread_;                       // ccvのトレッド [m]
  double path_reso_;                   // 生成するpathの刻み幅 [m]
  double theta_reso_;                  // 候補となるpathを生成する際の方位の刻み幅 [rad]
  int search_step_;                    // 何ステップ先まで評価値を計算するか
  double weight_heading_;              // 評価関数1項目　重みづけ定数
  double weight_glocal_heading_;       // 評価関数2項目　重みづけ定数
  double weight_cost_map_;             // 評価関数2項目　重みづけ定数
  double min_cost_;                    // 割り当てるコストの最小値

  // その他の変数
  double tmp_x_ = 0.0;         // 1ステップ前のロボットのx座標格納用
  double tmp_y_ = 0.0;         // 1ステップ前のロボットのy座標格納用
  double tmp_yaw_ = 0.0;       // 1ステップ前のロボットのyaw角格納用
  int glocal_path_index_ = 0;  // glocal_pathのインデックス

  // msgの受け取り判定用
  bool flag_cost_map_ = false;
  bool flag_local_goal_ = false;
  bool flag_glocal_path_ = false;

  // local_goalから一定距離内に到達したかの確認用
  bool flag_goal_check_ = false;

  // 座標変換の判定用
  bool flag_frame_change_ = false;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_cost_map_;
  ros::Subscriber sub_local_goal_;
  ros::Subscriber sub_glocal_path_;

  // Publisher
  ros::Publisher pub_target_path_;

  // tf
  tf2_ros::Buffer tf_buffer_;

  // 各種オブジェクト
  nav_msgs::OccupancyGrid cost_map_;         // コストマップ
  geometry_msgs::PointStamped local_goal_;   // local_goal
  nav_msgs::Path glocal_path_;               // glocal_path
  geometry_msgs::PointStamped glocal_goal_;  // glocal_goal
};

#endif // AD_TARGET_PATH_PLANNER_H