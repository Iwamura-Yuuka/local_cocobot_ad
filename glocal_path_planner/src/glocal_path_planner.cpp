# include "glocal_path_planner/glocal_path_planner.h"

GlocalPathPlanner::GlocalPathPlanner():private_nh_("~")
{
  // param
  private_nh_.param("visualize_for_debug", visualize_for_debug_, {false});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("path_frame", path_frame_, {"base_footprint"});
  private_nh_.param("goal_frame", goal_frame_, {"odom"});
  private_nh_.param("goal_tolerance", goal_tolerance_, {0.5});
  private_nh_.param("density_map_weight", density_map_weight_, {1.0});

  // subscriber
  sub_density_map_ = nh_.subscribe("/density_map", 1, &GlocalPathPlanner::density_map_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_local_goal_ = nh_.subscribe("/local_goal", 1, &GlocalPathPlanner::local_goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_glocal_path_ = nh_.advertise<nav_msgs::Path>("/glocal_path", 1);

  // debug
  if(visualize_for_debug_)
  {
    pub_target_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/target_goal", 1);
    pub_current_node_ = nh_.advertise<geometry_msgs::PointStamped>("/current_node", 1);
  }

  // path
  glocal_path_.header.frame_id = path_frame_;
  glocal_path_.poses.reserve(1000);
  current_node_.header.frame_id = path_frame_;
}

// density_mapのコールバック関数
void GlocalPathPlanner::density_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  density_map_ = *msg;
  flag_density_map_ = true;
  ROS_INFO_STREAM("density_map is received.");
}

// local_goalのコールバック関数
void GlocalPathPlanner::local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform(path_frame_, goal_frame_, ros::Time(0));
    flag_local_goal_ = true;
    ROS_INFO_STREAM("local_goal is received.");
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    flag_local_goal_ = false;
    ROS_INFO_STREAM("local_goal is not received.");
    return;
  }

  tf2::doTransform(*msg, local_goal_, transform);
}

// local_goalがマップ内の場合、trueを返す
bool GlocalPathPlanner::is_in_map()
{
  const int index_x = int(floor((local_goal_.point.x - density_map_.info.origin.position.x) / density_map_.info.resolution));
  const int index_y = int(floor((local_goal_.point.y - density_map_.info.origin.position.y) / density_map_.info.resolution));

  if((index_x >= 0) && (index_x < density_map_.info.width) && (index_y >= 0) && (index_y < density_map_.info.height))
    return true;
  else
    return false;
}

// 距離を計算
double GlocalPathPlanner::calc_distance(const double x1, const double y1, const double x2, const double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;

  return hypot(dx, dy);
}

// target_goal（マップ内のlocal_goal）を計算
void GlocalPathPlanner::calc_target_goal()
{
  if(is_in_map())
  {
    target_goal_ = local_goal_;
  }
  else
  {
    target_goal_.header.stamp = local_goal_.header.stamp;
    target_goal_.header.frame_id = local_goal_.header.frame_id;

    double target_goal_x = density_map_.info.origin.position.x + (density_map_.info.resolution / 2.0);
    double target_goal_y = density_map_.info.origin.position.y + (density_map_.info.resolution / 2.0);

    double min_dist = 100.0;  // 適当に大きい値で初期化

    // local_goalに最も近いtarget_goalを計算
    for(int i=0; i<density_map_.info.width; i++)
    {
      for(int j=0; j<density_map_.info.height; j++)
      {
        const double dist = calc_distance(local_goal_.point.x, local_goal_.point.y, target_goal_x, target_goal_y);

        if(dist < min_dist)
        {
          min_dist = dist;
          target_goal_.point.x = target_goal_x;
          target_goal_.point.y = target_goal_y;
        }

        target_goal_y += density_map_.info.resolution;
      }

      target_goal_x += density_map_.info.resolution;
      target_goal_y = density_map_.info.origin.position.y + (density_map_.info.resolution / 2.0);
    }
  }

  if(visualize_for_debug_)
    pub_target_goal_.publish(target_goal_);
}

// ノードが障害物上にあるかの判定
bool GlocalPathPlanner::is_in_obs(const Node node)
{
  const int index = node.index_x + (node.index_y * density_map_.info.width);

  // indexのコストが100（占有）の場合，trueを返す
  return density_map_.data[index] == 100;
}

// ゴールノードを取得
Node GlocalPathPlanner::get_goal_node()
{
  Node goal_node;

  // target_goal（ゴールノード計算用）の計算
  calc_target_goal();

  // ゴールノードの設定
  goal_node.index_x = int(floor((target_goal_.point.x - density_map_.info.origin.position.x) / density_map_.info.resolution));
  goal_node.index_y = int(floor((target_goal_.point.y - density_map_.info.origin.position.y) / density_map_.info.resolution));

  // ゴールノードが障害物上にあるかの判定
  flag_goal_node_in_obs_ = is_in_obs(goal_node);

  return goal_node;
}

// ヒューリスティック値を計算
double GlocalPathPlanner::calc_heuristic(const Node node)
{
  // ヒューリスティックの重み
  const double w = 1.0;

  // 2点間のユークリッド距離
  const double dx = double(node.index_x - goal_node_.index_x);
  const double dy = double(node.index_y - goal_node_.index_y);
  const double dist = hypot(dx, dy);

  return w * dist;
}

// スタートノードの場合trueを返す
bool GlocalPathPlanner::is_start(const Node node)
{
  return is_same_node(node, start_node_);
}

// ゴールノードの場合trueを返す
bool GlocalPathPlanner::is_goal(const Node node)
{
  // ゴールノードが障害物上にある場合はゴール手前でゴールとする
  if(flag_goal_node_in_obs_)
  {
    const double dx = (node.index_x - goal_node_.index_x) * density_map_.info.resolution;
    const double dy = (node.index_y - goal_node_.index_y) * density_map_.info.resolution;
    const double dist = hypot(dx, dy);

    return dist < goal_tolerance_;
  }
  else  // それ以外はゴールノードまで計算する
  {
    return is_same_node(node, goal_node_);
  }
}

// 2つが同じノードの場合trueを返す
bool GlocalPathPlanner::is_same_node(const Node node1, const Node node2)
{
  if(node1.index_x == node2.index_x && node1.index_y == node2.index_y)
    return true;
  else
    return false;
}

// Openリスト内で最もコストの小さいノードを取得
Node GlocalPathPlanner::select_current_node()
{
  Node current_node = open_set_[0];
  double min_cost = open_set_[0].cost;

  for(const auto& open_node : open_set_)
  {
    if(open_node.cost < min_cost)
    {
      min_cost = open_node.cost;
      current_node = open_node;
    } 
  }

  return current_node;
}

// ノードからポーズを計算
geometry_msgs::PoseStamped GlocalPathPlanner::calc_pose(const Node node)
{
  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = node.index_x * density_map_.info.resolution + density_map_.info.origin.position.x;
  pose.pose.position.y = node.index_y * density_map_.info.resolution + density_map_.info.origin.position.y;

  return pose;
}

// Closeリストの特定のノードが親ノードか判断
bool GlocalPathPlanner::is_parent(int closed_node_index, Node node)
{
  bool is_same_x = closed_set_[closed_node_index].index_x == node.parent_index_x;
  bool is_same_y = closed_set_[closed_node_index].index_y == node.parent_index_y;

  return is_same_x && is_same_y;
}

// ノードからパスを生成
void GlocalPathPlanner::create_path(Node current_node)
{
  nav_msgs::Path path;
  path.poses.push_back(calc_pose(current_node));

  // ノードリストの最後からスタートまで再探索
  while(!is_start(current_node))
  {
    for(int i=0; i<closed_set_.size(); i++)
    {
      if(is_parent(i, current_node))
      {
        current_node = closed_set_[i];
        path.poses.push_back(calc_pose(current_node));
        break;
      }

      if(i == closed_set_.size() - 1)
        ROS_ERROR_STREAM("parent node is not found.");
    }
  }

  reverse(path.poses.begin(), path.poses.end());
  glocal_path_.poses.insert(glocal_path_.poses.end(), path.poses.begin(), path.poses.end());
}

// 指定したリストに含まれるか検索
int GlocalPathPlanner::search_node_from_set(const Node node, const std::vector<Node>& set)
{
  for(int i=0; i<set.size(); i++)
  {
    if(is_same_node(node, set[i]))
      return i;  // インデックスを返す
  }

  return -1;  // 含まれていなかった場合は-1を返す
}

// set1からset2にノードを移動
// ノードをOpenリストからCloseリストに移動するときに使用
void GlocalPathPlanner::transfer_node(const Node node, std::vector<Node>& set1, std::vector<Node>& set2)
{
  // リスト1からノードを探す
  const int set1_node_index = search_node_from_set(node, set1);

  // リスト1にノードが含まれていない場合はエラー
  if(set1_node_index == -1)
    ROS_ERROR_STREAM("node is not found.");

  // リスト1からノードを削除
  set1.erase(set1.begin() + set1_node_index);

  // リスト2にノードを追加
  set2.push_back(node);
}

// 動作を作成
Motion GlocalPathPlanner::get_motion(const int dx, const int dy, const double cost)
{
  if((1 < abs(dx)) || (1 < abs(dy)))
    ROS_ERROR_STREAM("dx or dy is invalid.");

  Motion motion;
  motion.x = dx;
  motion.y = dy;
  motion.cost = cost;

  return motion;
}

// 動作モデルを作成
void GlocalPathPlanner::create_motion_model(std::vector<Motion>& motion_set)
{
  motion_set.push_back(get_motion( 1,  0, 1));  // 前
  motion_set.push_back(get_motion( 0,  1, 1));  // 左
  motion_set.push_back(get_motion(-1,  0, 1));  // 後ろ
  motion_set.push_back(get_motion( 0, -1, 1));  // 右

  motion_set.push_back(get_motion(-1,-1, sqrt(2)));  // 右後ろ
  motion_set.push_back(get_motion(-1, 1, sqrt(2)));  // 左後ろ
  motion_set.push_back(get_motion( 1,-1, sqrt(2)));  // 右前
  motion_set.push_back(get_motion( 1, 1, sqrt(2)));  // 左前
}

// 密度マップのコストを計算
double GlocalPathPlanner::calc_density_map_value(const Node node)
{
  const int index = node.index_x + (node.index_y * density_map_.info.width);

  double cost = density_map_.data[index];

  // 正規化
  // コストが0以下の場合は0にする
  if(cost > 0)
    cost /= 100.0;
  else
    cost = 0.0;
  
  return cost;
}

// 隣接ノードを取得
Node GlocalPathPlanner::get_neighbor_node(const Node node, const Motion motion)
{
  Node neighbor_node;

  // 移動
  neighbor_node.index_x = node.index_x + motion.x;
  neighbor_node.index_y = node.index_y + motion.y;

  // f値を記録
  // 密度マップのコストを考慮
  neighbor_node.cost = (node.cost - calc_heuristic(node)) + calc_heuristic(neighbor_node) + motion.cost + density_map_weight_ * calc_density_map_value(neighbor_node);

  // 親ノードを記録
  neighbor_node.parent_index_x = node.index_x;
  neighbor_node.parent_index_y = node.index_y;

  return neighbor_node;
}

// 現在のノードをもとに隣接ノードを作成
void GlocalPathPlanner::create_neighbor_nodes(const Node current_node, std::vector<Node>& neighbor_nodes)
{
  // 動作モデルの作成
  std::vector<Motion> motion_model;
  create_motion_model(motion_model);
  const int motion_num = motion_model.size();

  // 隣接ノードを作成
  for(int i=0; i<motion_num; i++)
  {
    Node neighbor_node = get_neighbor_node(current_node, motion_model[i]); // 隣接ノードを取得
    neighbor_nodes.push_back(neighbor_node);
  }  
}

// ノードが障害物か判断
bool GlocalPathPlanner::is_obs(const Node node)
{
  const int grid_index = node.index_x + (node.index_y * density_map_.info.width);
  return density_map_.data[grid_index] == 100;
}

// OpenリストまたはCloseリストに含まれるか調べる
std::tuple<int, int> GlocalPathPlanner::search_node(const Node node)
{
  // Openリストに含まれるか検索
  const int open_node_index = search_node_from_set(node, open_set_);
  if(open_node_index != -1)
    return std::make_tuple(1, open_node_index);

  // Closeリストに含まれるか検索
  const int closed_node_index = search_node_from_set(node, closed_set_);
  if(closed_node_index != -1)
    return std::make_tuple(2, closed_node_index);

  // OpenリストにもCloseリストにもない場合
  return std::make_tuple(-1, -1);
}

// 隣接ノードをもとにOpenリスト・Closeリストを更新
void GlocalPathPlanner::update_set(Node current_node)
{
  // 隣接ノードを宣言
  std::vector<Node> neighbor_nodes;
  
  // 現在のノードをもとに隣接ノードを作成
  create_neighbor_nodes(current_node, neighbor_nodes);

  // Openリスト・Closeリストを更新
  for(const auto& neighbor_node : neighbor_nodes)
  {
    // 障害物の場合
    if(is_obs(neighbor_node))
      continue;

    // リストに同一ノードが含まれるか調べる
    int flag = 0;
    int node_index = 0;
    std::tie(flag, node_index) = search_node(neighbor_node);

    if(flag == -1)  // OpenリストにもCloseリストにもない場合
    {
      open_set_.push_back(neighbor_node);
    }
    else if(flag == 1)  // Openリストにある場合
    {
      if(neighbor_node.cost < open_set_[node_index].cost)
      {
        open_set_[node_index].cost = neighbor_node.cost;
        open_set_[node_index].parent_index_x = neighbor_node.parent_index_x;
        open_set_[node_index].parent_index_y = neighbor_node.parent_index_y;
      }
    }
    else if(flag == 2)  // Closeリストにある場合
    {
      if(neighbor_node.cost < closed_set_[node_index].cost)
      {
        closed_set_.erase(closed_set_.begin() + node_index);
        open_set_.push_back(neighbor_node);
      }
    }
  } 
}

// glocal_pathを生成
void GlocalPathPlanner::create_glocal_path()
{
  glocal_path_.header.stamp = ros::Time::now();

  // リストを空にする
  open_set_.clear();
  closed_set_.clear();

  // スタート地点の設定
  int start_x = density_map_.info.width / 2;
  int start_y = density_map_.info.height / 2;

  // スタートノードとゴールノードを保持
  start_node_ = {start_x, start_y, 0, 0, 0};
  goal_node_ = get_goal_node();

  // スタートノードをOpenリストに追加
  start_node_.cost = calc_heuristic(start_node_);  // f(s) = h(s)
  open_set_.push_back(start_node_);

  // スタートノードで初期化
  Node current_node = start_node_;

  // glocal_pathを生成
  while(!is_goal(current_node))
  {
    if(open_set_.empty())
      ROS_WARN_STREAM("No path found!!");

    // Openリスト内で最もコストの小さいノードを現在のノードに指定
    current_node = select_current_node();

    // デバック用に現在のノードを表示
    if(visualize_for_debug_)
      show_node_point(current_node);

    // 経路の探索
    if(is_goal(current_node))
    {
      create_path(current_node);  // ノードからパスを生成
      break;  // 探索終了
    }
    else
    {
      transfer_node(current_node, open_set_, closed_set_);  // 現在のノードをCloseリストに移動
      update_set(current_node);  // 隣接ノードをもとにOpenリスト・Closeリストを更新
    }
  }

  pub_glocal_path_.publish(glocal_path_);
  glocal_path_.poses.clear();
}

// ノードをRvizに表示（デバック用）
void GlocalPathPlanner::show_node_point(const Node node)
{
  current_node_.point.x = node.index_x * density_map_.info.resolution + density_map_.info.origin.position.x;
  current_node_.point.y = node.index_y * density_map_.info.resolution + density_map_.info.origin.position.y;

  pub_current_node_.publish(current_node_);
}

void GlocalPathPlanner::process()
{
  ros::Rate loop_rate(hz_);
  tf2_ros::TransformListener tf_listener(tf_buffer_);

  while(ros::ok())
  {
    ROS_INFO_STREAM("aaaaaaaaaaaaaaaaaaaaaaaaaa");
    if(flag_density_map_ && flag_local_goal_)
    {
      ROS_INFO_STREAM("bbbbbbbbbbbbbbbbbbbbbbbbbb");
      create_glocal_path();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}