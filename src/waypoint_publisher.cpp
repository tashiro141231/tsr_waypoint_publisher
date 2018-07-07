#include <ros/ros.h>
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
// #include "calc.hpp"
// #include "wave.hpp"
//sgm
#include <nav_msgs/Odometry.h>
#include <boost/circular_buffer.hpp>
#include <time.h>
#include <boost/timer/timer.hpp>

#define WP_INTERVAL_TIME 0.0//sec

#include "waypoint_publisher.h"

//======= static member =========//
ros::Subscriber WaypointPublisher::sub_wheel_odom_;
ros::Subscriber WaypointPublisher::sub_signal_decision_;
ros::Subscriber WaypointPublisher::sub_judge_human_;
ros::Subscriber WaypointPublisher::sub_real_human_pose_;
ros::Publisher WaypointPublisher::pub_cmd_;
ros::Publisher WaypointPublisher::pub_nav_goal_;
ros::Publisher WaypointPublisher::pub_signal_decision_;
ros::Publisher WaypointPublisher::pub_search_human_;
ros::Publisher WaypointPublisher::pub_transfer_;
ros::Publisher WaypointPublisher::pub_waypoint_;
ros::Publisher WaypointPublisher::pub_human_pose_;


//-------------- Callback --------------//
/**
 * @fn 信号認識後のスタートフラグの受け取り
 * @brief g_signal_go_flag_ をたてる
 * @param 
 * @return None
 */
void WaypointPublisher::signalCallBack(const std_msgs::Bool::ConstPtr& signal_go_flag) {
    g_signal_go_flag_ = signal_go_flag->data;
    ROS_INFO("Receive Go flag");
}

/**
 * @fn 探索対象者の位置を保存する
 * @brief g_human_direction_ に格納され
 * @param human_pose: 対象者位置の保存用。real_human_poseのデータを読む。探索対象のロボット座標位置が格納されている 
 * @return None
 */
void WaypointPublisher::searchCallBack2(const geometry_msgs::PoseStamped& human_pose){ 
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
void WaypointPublisher::judgeCallBack(const std_msgs::Bool::ConstPtr& judge_result_flag){
    g_judge_result_flag_ = judge_result_flag->data;
    g_judge_receive_flag_ = true;
    std::cout << "receive human_judge_result" << std::endl;
}

/**
 * @fn
 * @brief odomを格納しておく
 * @param 
 * @return None
 */
void WaypointPublisher::odomReadCallback(const nav_msgs::Odometry::ConstPtr& a_wheel_odom){
    g_wheel_odom_msgs_.push_back(*a_wheel_odom);
}
//----------- end of callback  -------------//


/**
 * @fn Keu入力でwaypointを次へ進める
 * @brief 別スレッドでキーボードを監視
 * @param 
 * @return  None
 */
void WaypointPublisher::ForwardWaypointByKeyboardInterrupt() { 
    while (1) {
        getchar();
        g_forward_wp_flag_ = true; //wp更新要求
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
geometry_msgs::PoseArray ConvertToWayPointMsg(const std::vector<Pos> g_wp_array){
    geometry_msgs::PoseArray g_wp_array_msg;
    g_wp_array_msg.header.stamp = ros::Time::now();;
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
int WaypointPublisher::ReadWaypointFile(const std::string & file_name) {
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
 * @fn スタート地点やスロープなどの制御に必要な場所のwp
 * @brief wp_arrayから取得
 * @param None
 * @return None 
 */
void WaypointPublisher::SetSpecificPointIndex() {
    double min_dist_pos = 10000;
    double buff_distance;

    for(auto wp_itr = wp_array_.begin(); wp_itr != wp_array_.end(); wp_itr++) {
        Vertex2D wp_pos = {wp_itr->x, wp_itr->y};
        // Start地点
        buff_distance = distance_vertex(start_pos_, wp_pos);
        if(buff_distance <= min_dist_pos) {
            start_wp_index_ = std::distance(wp_array_.begin(), wp_itr);
        }

        // とりあえず
        buff_distance = distance_vertex(end_pos_, wp_pos);
        if(buff_distance <= min_dist_pos) {
            end_wp_index_ = std::distance(wp_array_.begin(), wp_itr);
        }
    }
}
