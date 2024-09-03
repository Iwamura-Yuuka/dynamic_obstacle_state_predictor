#ifndef DYNAMIC_OBSTACLE_STATE_CALCULATOR_H
#define DYNAMIC_OBSTACLE_STATE_CALCULATOR_H

#include <ros/ros.h>
#include <queue>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

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
    void obs_traj_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void obs_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    
    // 引数あり関数
    void transform_obs_pose(geometry_msgs::Point& obs_pose);  // 障害物情報をworld座標系からrobot座標系に変換
    void calc_obs_state(const visualization_msgs::Marker& obs_trajectory);  // 1つの障害物の状態を計算

    // 引数なし関数
    void update_obs_data();  // 障害物情報の更新

    // yamlファイルで設定可能な変数
    int hz_;
    std::string world_frame_;
    std::string robot_frame_;

    // msgの受け取り判定用
    bool is_obs_pose_received_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_obs_traj_;
    ros::Subscriber sub_obs_pose_;

    // Publisher
    ros::Publisher pub_obs_state_;
    ros::Publisher pub_obs_pose_;

    // tf
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;

    // 各種オブジェクト
    visualization_msgs::MarkerArray obs_trajectories_;
    geometry_msgs::PoseArray obs_poses_;
    geometry_msgs::PoseArray tmp_obs_poses_;
};

#endif // DYNAMIC_OBSTACLE_STATE_CALCULATOR_H