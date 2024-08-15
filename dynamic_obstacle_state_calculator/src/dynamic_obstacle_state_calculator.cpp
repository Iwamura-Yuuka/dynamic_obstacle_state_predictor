#include "dynamic_obstacle_state_calculator/dynamic_obstacle_state_calculator.h"

DynamicObstacleStateCalculator::DynamicObstacleStateCalculator():private_nh_("~"), tf_listener_(tf_buffer_)
{
  // param
  private_nh_.param("hz", hz_, {10});
    
  // Subscriber
  sub_obs_pose_ = nh_.subscribe("/dynamic_obstacle/trajectories", 1, &DynamicObstacleStateCalculator::obs_pose_callback, this);
}

// 障害物情報のコールバック関数
void DynamicObstacleStateCalculator::obs_pose_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  obs_trajectories_ = *msg;
  is_obs_pose_received_ = true;
}

// 1つの障害物の状態を計算
void DynamicObstacleStateCalculator::calc_obs_state(const visualization_msgs::Marker& obs_trajectory)
{
  geometry_msgs::Point current_obs_pose = obs_trajectory.points.back();  // 障害物の現在位置を取得
  geometry_msgs::Point prev_obs_pose = obs_trajectory.points[obs_trajectory.points.size() - 2];  // 障害物の1つ前の位置を取得
  // 障害物の軌跡情報の後ろ2つを取得
}

// 障害物情報の更新
void DynamicObstacleStateCalculator::update_obs_data()
{
  if(is_obs_pose_received_)  // 障害物情報がsubscribeできている場合，その情報をもとに処理
  {
    for(int id = 0; id < obs_trajectories_.markers.size(); id++)
    {
      calc_obs_state(obs_trajectories_.markers[id]);
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