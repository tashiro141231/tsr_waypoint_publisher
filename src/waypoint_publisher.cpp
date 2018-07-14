#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <string>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <signal.h>
#include <thread>
#include <queue>
// #include "wave.hpp"
//sgm
#include <nav_msgs/Odometry.h>
#include <boost/circular_buffer.hpp>
#include <time.h>
#include <boost/timer/timer.hpp>

#define WP_INTERVAL_TIME 0.0//sec

#include "waypoint_publisher.h"

//==========   static メンバ宣言   ==========//
tf::TransformListener* WaypointPublisher::tf_listener1_;
tf::TransformListener* WaypointPublisher::tf_listener2_;
tf::TransformListener* WaypointPublisher::tf_listener3_;

ros::Subscriber WaypointPublisher::sub_wheel_odom_;
ros::Subscriber WaypointPublisher::sub_signal_decision_;
ros::Subscriber WaypointPublisher::sub_judge_human_;
ros::Subscriber WaypointPublisher::sub_real_human_pose_;
ros::Publisher WaypointPublisher::pub_cmd_;
ros::Publisher WaypointPublisher::pub_nav_goal_;
ros::Publisher WaypointPublisher::pub_signal_decision_;
ros::Publisher WaypointPublisher::pub_search_human_;
ros::Publisher WaypointPublisher::pub_transform_;
ros::Publisher WaypointPublisher::pub_waypoint_;
ros::Publisher WaypointPublisher::pub_human_pose_;

bool WaypointPublisher::g_forward_wp_flag_;  //trueの時wpを更新要求するフラグ
bool WaypointPublisher::g_signal_go_flag_;
bool WaypointPublisher::g_found_human_flag_;   //Trueで探索対象者発見状態
bool WaypointPublisher::g_pause_flag_;     //
bool WaypointPublisher::g_judge_result_flag_;  //judge結果の格納用flag
bool WaypointPublisher::g_judge_receive_flag_; //judge結果を受け取ったflag
bool WaypointPublisher::g_entered_search_area_;//探索エリア内いるか
bool WaypointPublisher::g_search_publish_flag_;
bool WaypointPublisher::wp_skip_flag_;

bool WaypointPublisher::is_robot_reach_goal_;
ros::Time WaypointPublisher::wp_clock_start_;
int WaypointPublisher::wp_timeout_;
double WaypointPublisher::goal_tolerance_;

Pos WaypointPublisher::start_pos_;
int WaypointPublisher::start_wp_index_;
Pos WaypointPublisher::end_pos_;
int WaypointPublisher::end_wp_index_;
Pos WaypointPublisher::stop_pos_;
int WaypointPublisher::stop_pos_index_;

geometry_msgs::PoseStamped WaypointPublisher::g_human_direction_; //caffe_serverから受け取るvelodyne座標上のreal_human_pose 
geometry_msgs::PoseStamped WaypointPublisher::g_waypoint_direction_;  //wpの位置算出用(探索対象者用)
geometry_msgs::PoseStamped WaypointPublisher::g_human_pose_global_; //上のグローバル座標
geometry_msgs::PoseStamped WaypointPublisher::g_waypoint_pose_global_; //上のグローバル座標
std::vector<Pos> WaypointPublisher::wp_array_;
boost::circular_buffer<nav_msgs::Odometry> WaypointPublisher::g_wheel_odom_msgs_;
std::string WaypointPublisher::str_wp_skip_flag_;
int WaypointPublisher::wp_index_;
geometry_msgs::PoseArray WaypointPublisher::wp_array_msg_;

Pos WaypointPublisher::now_pos_;
Pos WaypointPublisher::current_wp_;
//========== static メンバ宣言終わり ==========//

//==========  Callback 関数  ==========//
/**
 * @fn 信号認識後のスタートフラグの受け取り
 * @brief g_signal_go_flag_ をたてる
 * @param 
 * @return None
 */
void WaypointPublisher::signalCallback(std_msgs::Bool signal_go_flag) {
    g_signal_go_flag_ = signal_go_flag.data;
    ROS_INFO("Receive Go flag");
}

/**
 * @fn 探索対象者の位置を保存する
 * @brief g_human_direction_ に格納され
 * @param human_pose: 対象者位置の保存用。real_human_poseのデータを読む。探索対象のロボット座標位置が格納されている 
 * @return None
 */
void WaypointPublisher::searchCallback2(geometry_msgs::PoseStamped human_pose){ 
    g_human_direction_ = human_pose;

    if((g_human_direction_.pose.position.x == 0 && g_human_direction_.pose.position.y == 0) ||
      sqrt(g_human_direction_.pose.position.x*g_human_direction_.pose.position.x + g_human_direction_.pose.position.y*g_human_direction_.pose.position.y) >=11.2 ){ //中身が0 or 距離が11.2m以上あったら誤認識と判断させる
        g_found_human_flag_ = false;        //探索対象者発見できず
    }else{
        g_found_human_flag_ = true;         //探索対象者発見した時
        std::cout<<"receive human_pose" << std::endl;
    }
    if(g_entered_search_area_ == true){
        g_search_publish_flag_ = true;
    }else{
        g_search_publish_flag_ = false;
    }
}

/**
 * @fn
 * @brief 探索対象者の判定結果受け取り 
 * @param 
 * @return None
 */
void WaypointPublisher::judgeCallback(std_msgs::Bool judge_result_flag){
    g_judge_result_flag_ = judge_result_flag.data;
    g_judge_receive_flag_ = true;
    std::cout << "receive human_judge_result" << std::endl;
}

/**
 * @fn
 * @brief odomを格納しておく
 * @param 
 * @return None
 */
void WaypointPublisher::odomReadCallback(nav_msgs::Odometry a_wheel_odom){
    g_wheel_odom_msgs_.push_back(a_wheel_odom);
}
//==========  end of Callback  ==========//


/**
 * @fn Keu入力でwaypointを次へ進める
 * @brief 別スレッドでキーボードを監視
 * @param 
 * @return  None
 */
void WaypointPublisher::ForwardWaypointByKeyboardInterrupt() { 
    while (1) {
        getchar();
        wp_skip_flag_ = true; //wp更新要求
        g_pause_flag_ = false;
        ROS_INFO("Keyboard interupt.");
        ROS_INFO("Current waypoint will be skipped.");
    }
}

/**
 * @fn
 * @brief Convert std::vector<Pos> to Posearray 
 * @param 
 * @return g_wp_arraY_msg:ウェイポイントを順番に格納されたarray
 */
geometry_msgs::PoseArray WaypointPublisher::ConvertToWayPointMsg(const std::vector<Pos> g_wp_array){
    geometry_msgs::PoseArray g_wp_array_msg;
    g_wp_array_msg.header.stamp = ros::Time::now();
    g_wp_array_msg.header.frame_id = "map";
    
    for(auto &wp : g_wp_array) {
        geometry_msgs::Pose wp_msg;
        wp_msg.position.x = wp.x;
        wp_msg.position.y = wp.y;
        wp_msg.position.z = 0.0;
        wp_msg.orientation = tf::createQuaternionMsgFromYaw(wp.yaw);        
        g_wp_array_msg.poses.push_back(wp_msg);
    }
    
    return g_wp_array_msg;
}


/**
 * @fn 指定されたファイルをwaypointとして読み込み
 * @brief メンバ変数wp_array_に格納する
 * @param file_name: 読み込むファイルへのfull path
 * @return None 
 */
int WaypointPublisher::ReadWaypointFile(std::string file_name) {
    std::vector<Pos> wp_array;
    std::ifstream wp_file(file_name);
    if (!wp_file) {
        std::cerr << "ERROR: Can't open waypoint file!!" << std::endl;
        return -1;
    }

    std::string reading_line;
    while (1) {
        std::getline(wp_file, reading_line);
        if (reading_line.empty()) {
            std::cout << "End of file." << std::endl;
            break;
        }
        std::stringstream line_ss(reading_line);

        Pos tmp_wp;
        line_ss >> tmp_wp.x >> tmp_wp.y >> tmp_wp.yaw;
        std::cout <<tmp_wp.x<<" "<<tmp_wp.y<<" "<<tmp_wp.yaw<<" "<<std::endl;
        wp_array.push_back(tmp_wp);
    }
    wp_array_ = wp_array;
    return 1;
}


/**
 * @fn スタート地点やスロープなどの制御に必要な場所の座標をメンバ変数にセット
 * @brief 
 * @param
 * @return
 */
void WaypointPublisher::SetSpecificPoint() {
    start_pos_ = SetPosXY(0, 0);
    end_pos_ = SetPosXY( 1, 1);
    stop_pos_ = SetPosXY( 2, 2);
}

/**
 * @fn スタート地点やスロープなどの制御に必要な場所のwp
 * @brief wp_arrayから取得
 * @param None
 * @return None 
 */
void WaypointPublisher::SetSpecificPointIndex() {
    double min_dist_pos = 10000;
    double buff_distance;

    for(auto wp_itr = wp_array_.begin(); wp_itr != wp_array_.end(); wp_itr++) {
        Pos wp_pos = SetPosXY( wp_itr->x, wp_itr->y);
        // Start地点
        buff_distance = distance_point(start_pos_, wp_pos);
        if(buff_distance <= min_dist_pos) {
            start_wp_index_ = std::distance(wp_array_.begin(), wp_itr);
        }

        // とりあえず
        buff_distance = distance_point(end_pos_, wp_pos);
        if(buff_distance <= min_dist_pos) {
            end_wp_index_ = std::distance(wp_array_.begin(), wp_itr);
        }
        
       buff_distance = distance_point(stop_pos_, wp_pos);
       if(buff_distance <= min_dist_pos) {
            stop_pos_index_ = std::distance(wp_array_.begin(), wp_itr);
       }
    }
}

/**
 * @fn 
 * @brief 2点間の距離を計算 
 * @param point1 点1
 * @param point2 点2
 * @return val 2点間の距離
 */
double WaypointPublisher::distance_point(Pos point1, Pos point2) {
    double val;
    val = sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
    return val;
}

/**
 * @fn 
 * @brief Posにx, yをセット 
 * @param x
 * @param y
 * @return p 値をセットしたPos型変数
 */
Pos WaypointPublisher::SetPosXY(double x, double y) {
    Pos p;
    p.x = x;
    p.y = y;

    return p;
}

/**
 * @fn ここに止めたいwpの番号のときtrueを返してロボットを止める．
 * @brief  
 * @param
 * @param 
 * @return 
 */
bool WaypointPublisher::StopCheck() {
    if(wp_index_ == stop_pos_index_ && !wp_skip_flag_){    //Do not skip when key board interupted
        return true;
    }
    else{
        return false;
    }
}
/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::IncrementWaypoint(){
    wp_clock_start_ = ros::Time::now();
    wp_index_++;

    current_wp_ = wp_array_[wp_index_];
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::PublishWaypointArray() {
   pub_waypoint_.publish(wp_array_msg_);
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::PublishCurrentWaypoint() {
    geometry_msgs::PoseStamped wp;

    wp.header.frame_id = "map";
    wp.header.stamp = ros::Time::now();
    wp.pose.position.x = current_wp_.x;
    wp.pose.position.y = current_wp_.y;
    wp.pose.position.z = 0;
    wp.pose.orientation = tf::createQuaternionMsgFromYaw(current_wp_.yaw);
    pub_nav_goal_.publish(wp);
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::MainProc() {
    g_forward_wp_flag_ = StopCheck();   //true: To stop  false: To allow increment way point.

    // Check if the waypoint can be updated.
    if(!g_forward_wp_flag_) {
        tf::StampedTransform transform;
        try {
            tf_listener1_->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
            tf_listener1_->lookupTransform("map", "base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        now_pos_.x = transform.getOrigin().x();
        now_pos_.y = transform.getOrigin().y();

        // When the robot in goal_tolerance [m], increment waypoint.
        if(distance_point(now_pos_, current_wp_) < goal_tolerance_) {
            ROS_INFO("Reached current waypoint. And set next waypoint.");
            IncrementWaypoint();
            g_forward_wp_flag_ = false;
            PublishCurrentWaypoint();
        }
        // Time out
        else if((ros::Time::now() - wp_clock_start_) > ros::Duration(wp_timeout_)) {
            ROS_INFO("Current waypoint is time out.");
            IncrementWaypoint();
            g_forward_wp_flag_ = false;
            PublishCurrentWaypoint();
        }
    }

    // For skip the waypoint from Key board
    if(wp_skip_flag_) {
        IncrementWaypoint();
        wp_skip_flag_ = false;
    }
}

