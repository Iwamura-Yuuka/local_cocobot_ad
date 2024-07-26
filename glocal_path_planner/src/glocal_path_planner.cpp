# include "glocal_path_planner/glocal_path_planner.h"

GlocalPathPlanner::GlocalPathPlanner():private_nh_("~")
{
  // param
  private_nh_.param("visualize_target_goal", visualize_target_goal_, {false});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("node_frame", node_frame_, {"base_footprint"});
  private_nh_.param("path_frame", path_frame_, {"base_footprint"});
  private_nh_.param("goal_frame", goal_frame_, {"odom"});

  // subscriber
  sub_density_map_ = nh_.subscribe("/density_map", 1, &GlocalPathPlanner::density_map_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_local_goal_ = nh_.subscribe("/local_goal", 1, &GlocalPathPlanner::local_goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_glocal_path_ = nh_.advertise<nav_msgs::Path>("/glocal_path", 1);

  // debug
  pub_target_goal_ = nh_.advertise<geometry_msgs::PointStamped>("/target_goal", 1);
}

// density_mapのコールバック関数
void GlocalPathPlanner::density_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  density_map_ = *msg;
  flag_density_map_ = true;
}

// local_goalのコールバック関数
void GlocalPathPlanner::local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform(node_frame_, goal_frame_, ros::Time(0));
    flag_local_goal_ = true;
  }
  catch(tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    flag_local_goal_ = false;
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

// local_goalの方位を計算
double GlocalPathPlanner::calc_direction()
{
    const double theta = atan2(local_goal_.point.y, local_goal_.point.x);

    return normalize_angle(theta);
}

// 適切な角度(-M_PI ~ M_PI)を返す
double GlocalPathPlanner::normalize_angle(double theta)
{
    if(theta > M_PI)
        theta -= 2.0 * M_PI;
    if(theta < -M_PI)
        theta += 2.0 * M_PI;

    return theta;
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

    // local_goalの方位を計算
    const double theta = calc_direction();
    ROS_INFO_STREAM("theta: " << theta);

    if(abs(theta) <= M_PI/4.0)
    {
      target_goal_.point.x = (density_map_.info.width * density_map_.info.resolution) - (density_map_.info.resolution / 2.0);

      // double x_length = (density_map_.info.width * density_map_.info.resolution) - (density_map_.info.resolution / 2.0);
      // target_goal_.point.y = density_map_.info.origin.position.y + (x_length * tan(theta));
      target_goal_.point.y = abs(target_goal_.point.x) * tan(theta);
      ROS_INFO_STREAM("aaa");
    }
    else if((abs(theta) > M_PI/4.0) && (abs(theta) <= M_PI/2.0))
    {
      // double y_length = (density_map_.info.height * density_map_.info.resolution / 2.0) - (density_map_.info.resolution / 2.0);
      // double y_length = (density_map_.info.height * density_map_.info.resolution / 4.0) - (density_map_.info.resolution / 2.0);
      // target_goal_.point.x = density_map_.info.origin.position.x + (y_length / tan(theta));

      if(theta > 0)
      {
        target_goal_.point.y = (density_map_.info.height * density_map_.info.resolution / 2.0) - (density_map_.info.resolution / 2.0); 
      }
      else
      {
        target_goal_.point.y = (density_map_.info.height * density_map_.info.resolution / 2.0) + (density_map_.info.resolution / 2.0);
      }

      target_goal_.point.x = abs(target_goal_.point.y) /tan(theta);
      ROS_INFO_STREAM("bbb");
    }
    else
    {
      target_goal_.point.x = density_map_.info.resolution / 2.0;
      
      if(theta > 0)
      {
        target_goal_.point.y = (density_map_.info.height * density_map_.info.resolution / 2.0) - (density_map_.info.resolution / 2.0);
      }
      else
      {
        target_goal_.point.y = (density_map_.info.height * density_map_.info.resolution / 2.0) + (density_map_.info.resolution / 2.0);
      }
      ROS_INFO_STREAM("ccc");
    }
  }

  if(visualize_target_goal_)
  {
    pub_target_goal_.publish(target_goal_);
  }
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

  return goal_node;
}






















// glocal_pathを生成
void GlocalPathPlanner::create_glocal_path()
{
  ros::Time now = ros::Time::now();

  // リストを空にする
  open_set_.clear();
  closed_set_.clear();

  // スタート地点の設定
  int start_x = 0;
  int start_y = density_map_.info.height / 2;

  // スタートノードとゴールノードを保持
  start_node_ = {start_x, start_y, 0, 0, 0};
  goal_node_ = get_goal_node();

  // 





  // nav_msgs::Path path;
  // path.header.stamp = ros::Time::now();
  // path.header.frame_id = path_frame_;

  // // glocal_pathの作成
  // // ここにコードを追加

  // pub_glocal_path_.publish(glocal_path);
}


void GlocalPathPlanner::process()
{
  ros::Rate loop_rate(hz_);
  tf2_ros::TransformListener tf_listener(tf_buffer_);

  while(ros::ok())
  {
    if(flag_density_map_ && flag_local_goal_)
    {
      create_glocal_path();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}