#include <ros/ros.h>
// #include "calc.hpp"
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
    static tf::TransformListener* tf_listener1_;
    static tf::TransformListener* tf_listener2_;
    static tf::TransformListener* tf_listener3_;

    static void Initialize(){
        wp_skip_flag_ = false;
        g_forward_wp_flag_ = false;
        g_signal_go_flag_ = false;
        g_found_human_flag_ = false;
        g_pause_flag_ = false;
        g_judge_result_flag_ = false;
        g_judge_receive_flag_ = false;
        g_search_publish_flag_ = false;
        is_robot_reach_goal_ = false;
        wp_index_ = 0;
        current_wp_ = wp_array_[wp_index_];
        wp_clock_start_ = ros::Time::now();
        wp_timeout_ = 15;
        goal_tolerance_ = 0.5;
        wp_array_msg_ = ConvertToWayPointMsg(wp_array_);

        ros::NodeHandle node;
        sub_wheel_odom_ = node.subscribe("odom", 200, WaypointPublisher::odomReadCallback);
        sub_signal_decision_ = node.subscribe("signal_go_flag" ,10, WaypointPublisher::signalCallback);
        sub_real_human_pose_ = node.subscribe("real_human_pose", 1, WaypointPublisher::searchCallback2);
        sub_judge_human_ = node.subscribe("judge_result_flag", 1, WaypointPublisher::judgeCallback);
        pub_cmd_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
        pub_nav_goal_ = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        pub_signal_decision_ = node.advertise<std_msgs::Bool>("judge_start_flag", 1);
        pub_search_human_ = node.advertise<std_msgs::Bool>("search_human_flag", 1);
        pub_transform_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        pub_waypoint_ = node.advertise<geometry_msgs::PoseArray>("waypoint", 1);
        pub_human_pose_ = node.advertise<geometry_msgs::PoseStamped>("human_position", 1);
    
    }

    static void MainProc();

    static void signalCallback(std_msgs::Bool signal_go_flag);
    static void searchCallback2(geometry_msgs::PoseStamped human_pose);
    static void judgeCallback(std_msgs::Bool judge_result_flag);
    static void odomReadCallback(nav_msgs::Odometry a_wheel_odom);
    static geometry_msgs::PoseArray ConvertToWayPointMsg(std::vector<Pos> g_wp_array);
    static int ReadWaypointFile(std::string file_name);
    static void ForwardWaypointByKeyboardInterrupt(); 
    static bool StopCheck();
    static void SetSpecificPoint();
    static void SetSpecificPointIndex();
    static void IncrementWaypoint();
    static void PublishCurrentWaypoint();
    static void PublishWaypointArray();

    static double distance_point(Pos point1, Pos point2);
    static Pos SetPosXY(double x, double y); 

    // setter
    static void set_wp_skip_num(std::string value){ str_wp_skip_flag_ = value; }
    static void set_wp_skip_flag(bool value){ wp_skip_flag_ = value; }
    // getter
    static std::string get_wp_skip_num(){ return str_wp_skip_flag_; }
    static bool get_wp_skip_flag(){ return wp_skip_flag_; }
    static bool getGoalFlag() { return is_robot_reach_goal_; }

  private:
    static Pos now_pos_;
    static Pos current_wp_;

    static ros::Time wp_clock_start_;
    static int wp_timeout_;
    static double goal_tolerance_;

    static bool g_forward_wp_flag_;  //trueの時wpを更新要求するフラグ
    static bool g_signal_go_flag_;
    static bool g_found_human_flag_;   //Trueで探索対象者発見状態
    static bool g_pause_flag_;     //
    static bool g_judge_result_flag_;  //judge結果の格納用flag
    static bool g_judge_receive_flag_; //judge結果を受け取ったflag
    static bool g_entered_search_area_;//探索エリア内いるか
    static bool g_search_publish_flag_;
    static bool wp_skip_flag_;
    static bool is_robot_reach_goal_;

    // 地図中の制御をかけるべき座標
    static Pos start_pos_;
    static int start_wp_index_;
    static Pos end_pos_;
    static int end_wp_index_;
    static Pos stop_pos_;
    static int stop_pos_index_;

    static ros::Subscriber sub_wheel_odom_;
    static ros::Subscriber sub_signal_decision_;
    static ros::Subscriber sub_judge_human_;
    static ros::Subscriber sub_real_human_pose_;
    static ros::Publisher pub_cmd_;
    static ros::Publisher pub_nav_goal_;
    static ros::Publisher pub_signal_decision_;
    static ros::Publisher pub_search_human_;
    static ros::Publisher pub_transform_;
    static ros::Publisher pub_waypoint_;
    static ros::Publisher pub_human_pose_;

    static geometry_msgs::PoseStamped g_human_direction_; //caffe_serverから受け取るvelodyne座標上のreal_human_pose 
    static geometry_msgs::PoseStamped g_waypoint_direction_;  //wpの位置算出用(探索対象者用)
    static geometry_msgs::PoseStamped g_human_pose_global_; //上のグローバル座標
    static geometry_msgs::PoseStamped g_waypoint_pose_global_; //上のグローバル座標
    static geometry_msgs::PoseArray wp_array_msg_;
    static std::vector<Pos> wp_array_;
    static int wp_index_;
    static boost::circular_buffer<nav_msgs::Odometry> g_wheel_odom_msgs_;
    static std::string str_wp_skip_flag_;
};

