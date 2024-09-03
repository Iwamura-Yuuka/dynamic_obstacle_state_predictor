#include "dynamic_obstacle_state_calculator/dynamic_obstacle_state_calculator.h"

DynamicObstacleStateCalculator::DynamicObstacleStateCalculator():private_nh_("~"), tf_listener_(tf_buffer_)
{
  // param
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("world_frame", world_frame_, {"map"});
  private_nh_.param("robot_frame", robot_frame_, {"base_link"});
    
  // subscriber
  sub_obs_traj_ = nh_.subscribe("/dynamic_obstacle/trajectories", 1, &DynamicObstacleStateCalculator::obs_traj_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_obs_pose_ = nh_.subscribe("/dynamic_obstacle/pose", 1, &DynamicObstacleStateCalculator::obs_pose_callback, this, ros::TransportHints().reliable().tcpNoDelay());

  // publisher
  pub_obs_state_ = nh_.advertise<pedestrian_msgs::PeopleStates>("/dynamic_obstacle_states/current", 1);

  // debug
  pub_obs_pose_ = nh_.advertise<geometry_msgs::PoseArray>("/dynamic_obstacle_pose/current", 1);
}

// 障害物情報のコールバック関数
void DynamicObstacleStateCalculator::obs_traj_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  obs_trajectories_ = *msg;
  is_obs_pose_received_ = true;
}

// 障害物情報をworld座標系からrobot座標系に変換
void DynamicObstacleStateCalculator::transform_obs_pose(geometry_msgs::Point& obs_pose)
{
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(robot_frame_, world_frame_, ros::Time(0));

    obs_pose.x -= tf.transform.translation.x;
    obs_pose.y -= tf.transform.translation.y;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
}

// 1つの障害物の状態を計算
void DynamicObstacleStateCalculator::calc_obs_state(const visualization_msgs::Marker& obs_trajectory, pedestrian_msgs::PersonState& obs)
{
  // 障害物の軌跡情報の後ろ2つを取得
  geometry_msgs::Point current_obs_pose = obs_trajectory.points.back();  // 障害物の現在位置を取得
  geometry_msgs::Point prev_obs_pose = obs_trajectory.points[obs_trajectory.points.size() - 2];  // 障害物の1つ前の位置を取得
  
  // 障害物の位置をworld座標系からrobot座標系に変換
  transform_obs_pose(current_obs_pose);
  transform_obs_pose(prev_obs_pose);

  // 障害物の移動速度を計算

}

// 障害物情報の更新
void DynamicObstacleStateCalculator::update_obs_data()
{
  if(is_obs_pose_received_)  // 障害物情報がsubscribeできている場合，その情報をもとに処理
  {
    if(obs_trajectories_.markers.size() != 0)  // 障害物情報が1つ以上ある場合は計算
    {
      for(int id = 0; id < obs_trajectories_.markers.size(); id++)
      {
        // 1つの障害物の状態を計算
        calc_obs_state(obs_trajectories_.markers[id]);
      }
    }
  }
  else
  {
    // 情報がsubできていなければ，予測で補完
  }
}

// メイン文で実行する関数
void DynamicObstacleStateCalculator::process()
{
  ros::Rate loop_rate(hz_);

  while(ros::ok())
  {
    update_obs_data();

    // msgの受け取り判定用flagをfalseに戻す
    is_obs_pose_received_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}