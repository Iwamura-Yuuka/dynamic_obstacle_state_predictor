#ifndef DYNAMIC_OBSTACLE_STATE_CALCULATOR_H
#define DYNAMIC_OBSTACLE_STATE_CALCULATOR_H

#include <ros/ros.h>
#include <queue>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// original_msgs
#include <pedestrian_msgs/PersonState.h>
#include <pedestrian_msgs/PeopleStates.h>

// ===== 構造体 =====
struct Coordinate
{
    double x;  // [m]
    double y;  // [m]
};

// ===== クラス =====
class DynamicObstacleStateCalculator
{
public:
    DynamicObstacleStateCalculator();
    void process();

private:
    // コールバック関数
    void obs_pose_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    
    // 引数あり関数

    // 引数なし関数

    // yamlファイルで設定可能な変数

    // msgの受け取り判定用
    bool is_obs_pose_received_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_obs_pose_;

    // Publisher

    // 各種オブジェクト
    visualization_msgs::MarkerArray obs_trajectories_;
};

#endif // DYNAMIC_OBSTACLE_STATE_CALCULATOR_H