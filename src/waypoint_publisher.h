#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
//sgm
#include <nav_msgs/Odometry.h>
#include <boost/circular_buffer.hpp>
#include <time.h>
#define WP_INTERVAL_TIME 0.0//sec
#include <boost/timer/timer.hpp>

typedef struct {
    double x;
    double y;
    double yaw;
} Pos;

typedef struct{
    double x;
    double y;
    double yaw;
    double time;
} AMCL_POSE;

class WaypointPublisher {
  public:
    void signalCallback(const std_msgs::Bool::ConstPtr& signal_go_flag);
    void searchCallBack2(const geometry_msgs::PoseStamped& human_pose);
    void judgeCallBack(const std_msgs::Bool::ConstPtr& judge_result_flag);
    void odomReadCallback(const nav_msgs::Odometry::ConstPtr& a_wheel_odom);
    void Initialize();
    geometry_msgs::PoseArray ConvertToWayPointMsg(const std::vector<Pos> g_wp_array);
    int ReadWaypointFile(const std::string & file_name);
    void signalCallBack(const std_msgs::Bool::ConstPtr& signal_go_flag);
    void ForwardWaypointByKeyboardInterrupt(); 
    void ReadWaypointFile(std::string file_name);
    // setter
    void set_wp_skip_num(std::string value){ str_wp_skip_flag_ = value; }
    void set_wp_skip_flag(bool value){ wp_skip_flag_ = value; }
    // getter
    std::string get_wp_skip_num(){ return str_wp_skip_flag_; }
    bool get_wp_skip_flag(){ return wp_skip_flag_; }

  private:
    bool g_forward_wp_flag_;  //trueの時wpを更新要求するフラグ
    bool g_signal_go_flag_;
    bool g_found_human_flag_;   //Trueで探索対象者発見状態
    bool g_pause_flag_;     //
    bool g_judge_result_flag_;  //judge結果の格納用flag
    bool g_judge_receive_flag_; //judge結果を受け取ったflag
    bool g_entered_search_area_;//探索エリア内いるか
    bool g_search_publish_flag_;
    bool wp_skip_flag_;

    geometry_msgs::PoseStamped g_human_direction_; //caffe_serverから受け取るvelodyne座標上のreal_human_pose 
    geometry_msgs::PoseStamped g_waypoint_direction_;  //wpの位置算出用(探索対象者用)
    geometry_msgs::PoseStamped g_human_pose_global_; //上のグローバル座標
    geometry_msgs::PoseStamped g_waypoint_pose_global_; //上のグローバル座標
    std::vector<Pos> wp_array_;
    boost::circular_buffer<nav_msgs::Odometry> g_wheel_odom_msgs_;
    std::string str_wp_skip_flag_;
};

