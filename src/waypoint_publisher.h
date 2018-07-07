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
    static tf::TransformListenr* tf_listener1_;
    static tf::TransformListenr* tf_listener2_;
    static tf::TransformListenr* tf_listener3_;

    static void Initialize(){
        wp_skip_flag_ = false;
        g_forward_wp_flag_ = false;
        g_signal_go_flag_ = false;
        g_found_human_flag_ = false;
        g_pause_flag_ = false;
        g_judge_result_flag_ = false;
        g_judge_receive_flag_ = false;
        g_search_publish_flag_ = false;

        ros::NodeHnadle node;
        sub_wheel_odom_ = node.subscribe("odom", 200, odomReadCallback);
        sub_signal_decision = node.subscribe("signal_go_flag" ,10, signalCallback);
        sub_real_human_pose_ = node.subscribe("real_human_pose", 1, searchCallback2);
        sub_judge_human_ = node.subscribe("judge_result_flag", 1, judgeCallback);
        pub_cmd = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
        pub_nav_goal = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        pub_signal_decision_ = node.advertise<std_msgs::Bool>("judge_start_flag", 1);
        pub_search_human_ = node.advertise<std_msgs::Bool>("search_human_flag", 1);
        pub_transform_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        pub_waypoint_ = node.advertise<geometry_msgs::PoseArray>("waypoint", 1);
        pub_huma_pose_ = node.advertise<geometry_msgs::PoseStamped>("human_position", 1);
    
    }

    void signalCallback(const std_msgs::Bool::ConstPtr& signal_go_flag);
    void searchCallBack2(const geometry_msgs::PoseStamped& human_pose);
    void judgeCallBack(const std_msgs::Bool::ConstPtr& judge_result_flag);
    void odomReadCallback(const nav_msgs::Odometry::ConstPtr& a_wheel_odom);
    geometry_msgs::PoseArray ConvertToWayPointMsg(const std::vector<Pos> g_wp_array);
    int ReadWaypointFile(const std::string & file_name);
    void signalCallBack(const std_msgs::Bool::ConstPtr& signal_go_flag);
    void ForwardWaypointByKeyboardInterrupt(); 
    void ReadWaypointFile(std::string file_name);
    void SetSpecificPointIndex();

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

    // 地図中の制御をかけるべき座標
    Vertex2D start_pos_;
    int start_wp_index_;
    Vertex2D end_pos_;
    int end_wp_index_;

    static ros::Subscriber sub_wheel_odom_;
    static ros::Subscriber sub_signal_decision_;
    static ros::Subscriber sub_judge_human_;
    static ros::Subscriber sub_real_human_pose_;
    static ros::Publisher pub_cmd_;
    static ros::Publisher pub_nav_goal_;
    static ros::Publisher pub_signal_decision_;
    static ros::Publisher pub_search_human_;
    static ros::Publisher pub_transfer_;
    static ros::Publisher pub_waypoint_;
    static ros::Publisher pub_human_pose_;

    geometry_msgs::PoseStamped g_human_direction_; //caffe_serverから受け取るvelodyne座標上のreal_human_pose 
    geometry_msgs::PoseStamped g_waypoint_direction_;  //wpの位置算出用(探索対象者用)
    geometry_msgs::PoseStamped g_human_pose_global_; //上のグローバル座標
    geometry_msgs::PoseStamped g_waypoint_pose_global_; //上のグローバル座標
    std::vector<Pos> wp_array_;
    boost::circular_buffer<nav_msgs::Odometry> g_wheel_odom_msgs_;
    std::string str_wp_skip_flag_;
};

