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
#include <actionlib_msgs/GoalID.h>
#include <visualization_msgs/MarkerArray.h>
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
    bool stop;
    bool in_line;
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

    static void Initialize(int start_num){
        wp_skip_flag_ = false;
        g_signal_go_flag_ = false;
        g_found_human_flag_ = false;
        g_pause_flag_ = false;
        g_judge_result_flag_ = false;
        g_judge_receive_flag_ = false;
        search_human_flag_ = false;
        is_robot_approaching_ = false;
        is_robot_reach_end_ = false;
        wp_index_ = start_num;
        current_wp_ = wp_array_[wp_index_];
        wp_clock_start_ = ros::Time::now();
        wp_timeout_ = 30;
        goal_tolerance_ = 1.5;
        is_vel_restricted_ = false;
        is_next_stop_ = false;

        ros::NodeHandle node;
        sub_wheel_odom_ = node.subscribe("odom", 200, WaypointPublisher::odomReadCallback);
        sub_signal_decision_ = node.subscribe("signal_go_flag" ,10, WaypointPublisher::signalCallback);
        sub_real_human_pose_ = node.subscribe("human_pose_imi", 1, WaypointPublisher::searchCallback2);
        sub_judge_human_ = node.subscribe("judge_result_flag", 1, WaypointPublisher::judgeCallback);
        pub_cmd_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
        pub_nav_goal_ = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        pub_signal_decision_ = node.advertise<std_msgs::Bool>("judge_start_flag", 1);
        pub_search_human_ = node.advertise<std_msgs::Bool>("search_start_flag", 1);
        pub_transform_ = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
        pub_goal_cancel_ = node.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
        pub_waypoint_ = node.advertise<geometry_msgs::PoseArray>("waypoint", 1);
        pub_human_pose_ = node.advertise<geometry_msgs::PoseStamped>("human_position", 1);
        pub_wp_text_ = node.advertise<visualization_msgs::MarkerArray>("waypoints_number", 100);
        pub_audio_req_ = node.advertise<std_msgs::String>("audio_request", 1);
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
    static bool LineCheck();
    static void CallLineSetting();
    static void CallDefaultSetting();
    static void SetSpecificPoint(std::string stop_file, std::string line_file);
    static void SetSpecificPointIndex();
    static void IncrementWaypoint();
    static void PublishCurrentWaypoint();
    static void PublishWaypointArray();
    static void PublishAudioRequest(std::string value);
    static void PublishSearchHumanRequest(bool value);
    static void PublishWaypointForApproach();
    static bool JudgeApproach();
    static bool ApproachStateCheck();
    static int HumanCheck(geometry_msgs::PoseStamped human_pose);
    
    static void SetMovebaseRecovBehavior(bool value); 
    static void SetMovebaseSimtime(double value);
    static void SetMovebaseMaxObsRange(double range);
    static void SetMovebaseMaxXvel(double x);
    static void SetMovebaseMinXvel(double x);
    static void SetMovebaseMaxRvel(double r);
    //Abe
    static void SetWayPointNumText();
    static void PublishWayPointNumText();static void SetMovebaseMinRvel(double r);

    static double distance_point(Pos point1, Pos point2);
    static Pos SetPosXY(double x, double y); 
    static Pos SetPosXYYaw(double x, double y, double yaw); 

    // setter
    static void set_wp_skip_num(std::string value){ str_wp_skip_flag_ = value; }
    // getter
    static std::string get_wp_skip_num(){ return str_wp_skip_flag_; }
    static bool get_wp_skip_flag(){ return wp_skip_flag_; }
    static bool getGoalFlag() { return is_robot_reach_end_; }
    static bool search_human_flag() { return search_human_flag_; }

  private:
    static Pos now_pos_;
    static Pos current_wp_;

    static ros::Time wp_clock_start_;
    static ros::Time wp_repub_clock_;
    static int wp_timeout_;
    static double goal_tolerance_;

    static bool is_next_stop_;
    static bool g_signal_go_flag_;
    static bool g_found_human_flag_;   //Trueで探索対象者発見状態
    static bool g_pause_flag_;     //
    static bool g_judge_result_flag_;  //judge結果の格納用flag
    static bool g_judge_receive_flag_; //judge結果を受け取ったflag
    static bool g_entered_search_area_;//探索エリア内いるか
    static bool search_human_flag_;
    static bool wp_skip_flag_;
    static bool is_robot_reach_end_;
    static bool is_vel_restricted_;
    static bool is_robot_approaching_;
    static visualization_msgs::MarkerArray g_wp_array_visualize_;

    // 地図中の制御をかけるべき座標
    static std::vector<Pos> stop_list_;
    static std::vector<Pos> line_list_;
    static std::vector<int> stop_index_;

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
    static ros::Publisher pub_goal_cancel_;
    static ros::Publisher pub_wp_text_;
    static ros::Publisher pub_audio_req_;

    static geometry_msgs::PoseStamped g_human_direction_; //caffe_serverから受け取るvelodyne座標上のreal_human_pose 
    static geometry_msgs::PoseStamped g_waypoint_direction_;  //wpの位置算出用(探索対象者用)
    static geometry_msgs::PoseStamped g_human_pose_global_; //上のグローバル座標
    static geometry_msgs::PoseStamped g_waypoint_pose_global_; //上のグローバル座標
    static geometry_msgs::PoseArray wp_array_msg_;
    static std::vector<Pos> wp_array_;
    static std::vector<Pos> human_pos_;
    static std::vector<Pos> wp_for_approach_;
    static int wp_index_;
    static boost::circular_buffer<nav_msgs::Odometry> g_wheel_odom_msgs_;
    static std::string str_wp_skip_flag_;
};

