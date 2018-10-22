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
ros::Publisher WaypointPublisher::pub_goal_cancel_;
ros::Publisher WaypointPublisher::pub_wp_text_;
ros::Publisher WaypointPublisher::pub_audio_req_;
 
bool WaypointPublisher::is_next_stop_;
bool WaypointPublisher::g_signal_go_flag_;   //信号認識の結果
bool WaypointPublisher::g_found_human_flag_;   //Trueで探索対象者発見状態
bool WaypointPublisher::g_pause_flag_;      //
bool WaypointPublisher::g_judge_result_flag_;  //judge結果の格納用flag
bool WaypointPublisher::g_judge_receive_flag_; //judge結果を受け取ったflag
bool WaypointPublisher::g_entered_search_area_;//探索エリア内いるか
bool WaypointPublisher::search_human_flag_;
bool WaypointPublisher::wp_skip_flag_;      // waypointのスキップフラグ
bool WaypointPublisher::is_vel_restricted_;

bool WaypointPublisher::is_robot_approaching_;
bool WaypointPublisher::is_robot_reach_end_;
ros::Time WaypointPublisher::wp_clock_start_;
ros::Time WaypointPublisher::wp_repub_clock_;
int WaypointPublisher::wp_timeout_;
double WaypointPublisher::goal_tolerance_;

std::vector<Pos> WaypointPublisher::stop_list_;
std::vector<Pos> WaypointPublisher::line_list_;
std::vector<int> WaypointPublisher::stop_index_;

geometry_msgs::PoseStamped WaypointPublisher::g_human_direction_; //caffe_serverから受け取るvelodyne座標上のreal_human_pose 
geometry_msgs::PoseStamped WaypointPublisher::g_waypoint_direction_;  //wpの位置算出用(探索対象者用)
geometry_msgs::PoseStamped WaypointPublisher::g_human_pose_global_; //上のグローバル座標
geometry_msgs::PoseStamped WaypointPublisher::g_waypoint_pose_global_; //上のグローバル座標
visualization_msgs::MarkerArray WaypointPublisher::g_wp_array_visualize_;
std::vector<Pos> WaypointPublisher::wp_array_;
std::vector<Pos> WaypointPublisher::human_pos_;
std::vector<Pos> WaypointPublisher::wp_for_approach_;
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
    if(!is_robot_approaching_) {
        int result = HumanCheck(human_pose);
        if(result == -1) {
            g_found_human_flag_ = false;        //探索対象者発見できず
        }
        else if(result == 1) {
            g_found_human_flag_ = true;        //探索対象者発見. 次のメインループからアプローチ開始
        }
        else if(result == -2) {
            g_found_human_flag_ = false;
        }
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
    // std::cout << "receive human_judge_result" << std::endl;
}

/**
 * @fn
 * @brief odomを格納しておく
 * @param 
 * @return None
 */
void WaypointPublisher::odomReadCallback(nav_msgs::Odometry a_wheel_odom){
    g_wheel_odom_msgs_.push_back(a_wheel_odom);
    // std::cout << "receive human_judge_result" << std::endl;
}
//==========  end of Callback  ==========//

/*
 * @fn
 * @brief Move_baseのリカバリーモードのon/off
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseRecovBehavior(bool value) {
    std::string s_val;
    if(value) s_val = "true";
    else s_val = "false";

    std::string sentence = "rosrun dynamic_reconfigure dynparam set /move_base recovery_behavior_enabled " + s_val;

    std::system(sentence.c_str());
}

/*
 * @fn
 * @brief Move_baseのリカバリーモードのon/off
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseSimtime(double value) {
    std::string s_val = std::to_string(value);

    std::string sentence = "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS simtime " + s_val;

    std::system(sentence.c_str());
}

/*
 * @fn
 * @brief 障害物を認識する最大距離を変更
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseMaxObsRange(double range) {
    std::string s_range = std::to_string(range);
    std::string sentence = "rosrun dynamic_reconfigure dynparam set move_base/local_costmap/obstacle_layer max_obstacle_range " + s_range;

    std::system(sentence.c_str());
}

/*
 * @fn
 * @brief Move_baseの最大並進速度の指定
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseMaxXvel(double x) {
    std::string s_x = std::to_string(x);
    std::string sentence = "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS max_vel_x " + s_x;

    std::system(sentence.c_str());
}

/**
 * @fn
 * @brief Move_baseの最小並進速度の指定
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseMinXvel(double x) {
    std::string s_x = std::to_string(x);
    std::string sentence = "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS min_vel_x " + s_x;

    std::system(sentence.c_str());
}

/**
 * @fn
 * @brief Move_baseの最大回転角速度の指定
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseMaxRvel(double r) {
    std::string s_r = std::to_string(r);
    std::string sentence = "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS max_vel_theta " + s_r;

    std::system(sentence.c_str());
}

/**
 * @fn
 * @brief Move_baseの最小回転角速度の指定
 * @param 
 * @return None
 */
void WaypointPublisher::SetMovebaseMinRvel(double r) {
    std::string s_r = std::to_string(r);
    std::string sentence = "rosrun dynamic_reconfigure dynparam set /move_base/TrajectoryPlannerROS min_vel_theta " + s_r;

    std::system(sentence.c_str());
}

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
 * @fn 音声の再生リクエストをパブリッシュ
 * @brief
 * @param 
 * @return  None
 */
void WaypointPublisher::PublishAudioRequest(std::string value) {
    std_msgs::String data;
    data.data = value.c_str();
    pub_audio_req_.publish(data);
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
 * @fn WayPoint番号をRviz上に表示する関数
 * @brief  
 * @param
 * @param 
 * @return
 */
void WaypointPublisher::SetWayPointNumText() {

    // visualization_msgs::MarkerArray g_wp_array_visualize_;
    visualization_msgs::Marker g_wp_visualize;
    int wp_data_num = 0;

    g_wp_visualize.header.stamp = wp_array_msg_.header.stamp;
    g_wp_visualize.header.frame_id = wp_array_msg_.header.frame_id; 

    for(auto &wp : wp_array_msg_.poses) {
        std::string wp_text = "No." + std::to_string(wp_data_num);
        g_wp_visualize.ns = wp_text;
        g_wp_visualize.id = wp_data_num;
        g_wp_visualize.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        g_wp_visualize.action = visualization_msgs::Marker::ADD;
        g_wp_visualize.pose = wp;
        g_wp_visualize.scale.z = 0.5;
        g_wp_visualize.color.a = 1.0; // Don't forget to set the alpha!
        g_wp_visualize.color.r = 1.0;
        g_wp_visualize.color.g = 1.0;
        g_wp_visualize.color.b = 1.0;
        g_wp_visualize.text = wp_text;     
        g_wp_array_visualize_.markers.push_back(g_wp_visualize);
        wp_data_num++;
    }
}

/**
 * @fn 指定されたファイルをwaypointとして読み込み
 * @brief メンバ変数wp_array_に格納する
 * @param file_name: 読み込むファイルへのfull path
 * @return None 
 */
int WaypointPublisher::ReadWaypointFile(std::string file_name) {
    std::vector<Pos> wp_array;
    std::cout << file_name << std::endl;
    std::ifstream wp_file(file_name);
    std::string reading_line;

    if (!wp_file) {
        std::cerr << "ERROR: Can't open waypoint file!!" << std::endl;
        return -1;
    }

    while (1) {
        std::getline(wp_file, reading_line);
        if (reading_line.empty()) {
            std::cout << "End of file." << std::endl;
            break;
        }
        std::stringstream line_ss(reading_line);
        // std::cout << line_ss << std::endl;

        Pos tmp_wp;
        line_ss >> tmp_wp.x >> tmp_wp.y >> tmp_wp.yaw;
        std::cout <<tmp_wp.x<<" "<<tmp_wp.y<<" "<<tmp_wp.yaw<<" "<<std::endl;
        tmp_wp.stop = false;
        wp_array.push_back(tmp_wp);
    }
    wp_array_ = wp_array;
    wp_array_msg_ = ConvertToWayPointMsg(wp_array_);
    return 1;
}

/**
 * @fn WayPoint番号をpublishする関数
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::PublishWayPointNumText() {

    // visualization_msgs::MarkerArray g_wp_array_visualize = wp_data;
    int wp_data_num = 0;

    for(auto &wp : g_wp_array_visualize_.markers) {
        if(wp_data_num == wp_index_) {
            wp.color.a = 1.0; 
            wp.color.r = 1.0;
            wp.color.g = 0.0;
            wp.color.b = 1.0;
        } 
        else {
            wp.color.a = 1.0; 
            wp.color.r = 1.0;
            wp.color.g = 1.0;
            wp.color.b = 1.0;
        }  
        wp_data_num++;
    }
    pub_wp_text_.publish(g_wp_array_visualize_);   
}

/**
 * @fn スタート地点やスロープなどの制御に必要な場所の座標をメンバ変数にセット
 * @brief 
 * @param
 * @return
 */
void WaypointPublisher::SetSpecificPoint(std::string stop_file, std::string line_file) {
    std::ifstream wp_info;
    std::string reading_line;
    
    for(int i = 0; i < 2; i++) {
        if(i == 0) {
            wp_info.open(stop_file, std::ios::in);
        }
        else if(i == 1) {
            wp_info.open(line_file, std::ios::in);
        }
            
        while(1 && wp_info) {
            std::getline(wp_info, reading_line);
            if(reading_line.empty()) {
                break;
            }
            Pos tmp;
            std::stringstream line_ss(reading_line);
            line_ss >> tmp.x >> tmp.y >> tmp.yaw;
            std::cout << tmp.x << " " << tmp.y << " " << tmp.yaw << std::endl;
            if(i == 0)  stop_list_.push_back(tmp);
            if(i == 1)  line_list_.push_back(tmp);
        }
   }

    // stop_pos1_ = SetPosXY( -23.223, -29.2705);
    // // stop_pos1_ = SetPosXY( 59.51379, -6.51099);
    // stop_pos2_ = SetPosXY( -229.875, 48.9852);
    // stop_list
    SetSpecificPointIndex();
}

/**
 * @fn スタート地点やスロープなどの制御に必要な場所のwp
 * @brief wp_arrayから取得
 * @param None
 * @return None 
 */
void WaypointPublisher::SetSpecificPointIndex() {
    double min_dist_pos = 1.0;
    double buff_distance;
    int i =0;

    for(auto wp_itr = wp_array_.begin(); wp_itr != wp_array_.end(); wp_itr++) {
        Pos wp_pos = SetPosXY( wp_itr->x, wp_itr->y);
        wp_itr->stop = false;
        wp_itr->in_line = false;

        for(auto stop_itr = stop_list_.begin(); stop_itr != stop_list_.end(); stop_itr++) {
            Pos stop_pos = SetPosXY(stop_itr->x, stop_itr->y);
            buff_distance = distance_point(stop_pos, wp_pos);
            if(buff_distance <= min_dist_pos) {
                ROS_INFO("Found Stop position.");
                wp_itr->stop = true;
            }
        }

        for(auto line_itr = line_list_.begin(); line_itr != line_list_.end(); line_itr++) {
            Pos line_pos = SetPosXY(line_itr->x, line_itr->y);
            buff_distance = distance_point(line_pos, wp_pos);
            if(buff_distance <= min_dist_pos) {
                ROS_INFO("Found line position.");
                wp_itr->in_line = true;
            }
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
 * @fn 
 * @brief Posにx, yをセット 
 * @param x
 * @param y
 * @return p 値をセットしたPos型変数
 */
Pos WaypointPublisher::SetPosXYYaw(double x, double y, double yaw) {
    Pos p;
    p.x = x;
    p.y = y;
    p.yaw = yaw;

    return p;
}

/**
 * @fn アプローチすべき探索対象であるかチェック
 * @brief  
 * @param
 * @param 
 * @return 
 */
int WaypointPublisher::HumanCheck(geometry_msgs::PoseStamped human_pose) {
    double distance = sqrt(human_pose.pose.position.x*human_pose.pose.position.x + human_pose.pose.position.y*human_pose.pose.position.y);
    // Invalid data
    if(distance >=11.2 ){ //中身が0 or 距離が(velodyneから)11.2m以上あったら誤認識と判断させる
        ROS_INFO("Receive but data is too far.");
        std::cout << "distance = " << distance << std::endl;
        return -1;
    }
    else if(human_pose.pose.position.x == 0 && human_pose.pose.position.y == 0) {
        // ROS_INFO("Empty data.");
        return -1;
    }

    try {
        geometry_msgs::PoseStamped wp;
        wp.header.stamp = human_pose.header.stamp;
        wp.header.frame_id = "velodyne";
        wp.pose.orientation = human_pose.pose.orientation;
        wp.pose.position.x = human_pose.pose.position.x + 0.5 * cos(tf::getYaw(human_pose.pose.orientation));
        wp.pose.position.y = human_pose.pose.position.y + 0.5 * cos(tf::getYaw(human_pose.pose.orientation));
        
        //human pose and waypoint pose in map.
        tf_listener1_->waitForTransform("map", "velodyne", human_pose.header.stamp, ros::Duration(1.0));
        tf_listener1_->transformPose("map", human_pose, human_pose);
        tf_listener1_->transformPose("map", wp, wp);
        // tf_listener1_->lookupTransform("map", "velodyne", human_pose.header.stamp, human_pose);
        // tf_listener1_->lookupTransform("map", "velodyne", human_pose.header.stamp, wp_to_human);
        
        if(human_pos_.empty()) {
            Pos p = SetPosXY(human_pose.pose.position.x, human_pose.pose.position.y);
            human_pos_.push_back(p);
            p = SetPosXYYaw(wp.pose.position.x, wp.pose.position.y, tf::getYaw(wp.pose.orientation));
            wp_for_approach_.push_back(p);
            ROS_INFO("---------------------------");
            ROS_INFO("           FOUND           ");
            ROS_INFO("        first person       ");
            ROS_INFO("---------------------------");
            std::string req("found_candidate");
            PublishAudioRequest(req);
            return 1;
        }

        bool end_check = true;
        for(auto itr_pose = human_pos_.begin(); itr_pose != human_pos_.end(); itr_pose++) {
            double diff_x = itr_pose->x - human_pose.pose.position.x;
            double diff_y = itr_pose->y - human_pose.pose.position.y;
            distance = sqrt(diff_x * diff_x + diff_y * diff_y);
            std::cout << "DISTANCE = " << distance << std::endl;

            // 以前発見した探索対象者から10[m]以上離れていた場合
            if(distance < 10.0 ) {
                end_check = false;
            } 
        }
        if(end_check) {
            Pos p = SetPosXY(human_pose.pose.position.x, human_pose.pose.position.y);
            human_pos_.push_back(p);
            p = SetPosXYYaw(wp.pose.position.x, wp.pose.position.y, tf::getYaw(wp.pose.orientation));
            wp_for_approach_.push_back(p);
            ROS_INFO("---------------------------");
            ROS_INFO("           FOUND           ");
            ROS_INFO("---------------------------");
            std::string req("found_candidate");
            PublishAudioRequest(req);
            return 1;
        }
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return -1;
    }

    // ROS_INFO("Same person or TF error.");
    return -2;
}

/**
 * @fn ここに止めたいwpの番号のときtrueを返してロボットを止める．
 * @brief  
 * @param
 * @param 
 * @return 
 */
bool WaypointPublisher::StopCheck() {
    if(wp_array_[wp_index_].stop) {
        is_vel_restricted_ = true;
        return true;
    }
    //if(g_found_human_flag_) return false;
    return false;
}

/**
 * @fn 行列用のwpのとき、trueを返す.
 * @brief  
 * @param
 * @param 
 * @return 
 */
bool WaypointPublisher::LineCheck() {
    if(wp_array_[wp_index_].in_line && !wp_array_[wp_index_-1].in_line) {
        CallLineSetting();
        return true;
    }
    else if(!wp_array_[wp_index_].in_line && wp_array_[wp_index_-1].in_line) {
        CallDefaultSetting();
    }
    //if(g_found_human_flag_) return false;
    return false;
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::CallLineSetting() {
    ROS_INFO("======================= Revoverry enable ================================");
    SetMovebaseRecovBehavior(false);
    SetMovebaseSimtime(1.5);
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::CallDefaultSetting() {
    SetMovebaseRecovBehavior(true);
    SetMovebaseSimtime(2.5);
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
    wp_repub_clock_ = ros::Time::now();
    if(wp_array_.size() != wp_index_){
        wp_index_++;
        is_next_stop_ = !wp_array_[wp_index_].stop;
        if(wp_array_[wp_index_ + 1].stop){
            is_next_stop_ = true;
            goal_tolerance_ = 0.5;
        }
        else {
            is_next_stop_ = false;
            goal_tolerance_ = 1.5;
        }
        if(wp_array_[wp_index_].in_line) {
            goal_tolerance_ = 0.3;
        }
    }
    else {
        is_robot_reach_end_ = true;
        is_next_stop_ = false;
        // std::string req("dd");
        // PublishAudioRequest(req);
    }

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
void WaypointPublisher::PublishSearchHumanRequest(bool value) {
   std_msgs::Bool flag;
   flag.data = value;
   pub_search_human_.publish(flag);
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
    std::cout << current_wp_.x << " " << current_wp_.y << std::endl;  

    wp.header.frame_id = "map";
    wp.header.stamp = ros::Time::now();
    wp.pose.position.x = current_wp_.x;
    wp.pose.position.y = current_wp_.y;
    wp.pose.position.z = 0;
    wp.pose.orientation = tf::createQuaternionMsgFromYaw(current_wp_.yaw);
    pub_nav_goal_.publish(wp);
    ROS_INFO("Published waypoint.");
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::PublishWaypointForApproach() {
    geometry_msgs::PoseStamped wp;
    std::cout <<"Approach to :" << wp_for_approach_.back().x << ", " << wp_for_approach_.back().y << std::endl;  
    wp_repub_clock_ = ros::Time::now();

    // human_pos_.back()が最新のデータのはず
    wp.header.frame_id = "map";
    wp.header.stamp = ros::Time::now();
    wp.pose.position.x = wp_for_approach_.back().x;
    wp.pose.position.y = wp_for_approach_.back().y;
    wp.pose.position.z = 0;
    wp.pose.orientation = tf::createQuaternionMsgFromYaw(wp_for_approach_.back().yaw);
    pub_nav_goal_.publish(wp);

    is_robot_approaching_ = true;
    ROS_INFO("Published waypoint for Approach.");
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
bool WaypointPublisher::ApproachStateCheck() {
    tf::StampedTransform transform;

    try {
        ros::Time now = ros::Time::now();
        tf_listener1_->waitForTransform("map", "base_link", now, ros::Duration(1.0));
        tf_listener1_->lookupTransform("map", "base_link", now, transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    now_pos_.x = transform.getOrigin().x();
    now_pos_.y = transform.getOrigin().y();
    double distance = distance_point(now_pos_, wp_for_approach_.back());
    std::cout << "Goal to " << distance << "[m]" << std::endl;
    if(distance < 0.5) {
        ROS_INFO("End Approach.");
        std::string req("found_target");
        PublishAudioRequest(req);
        return true;
    }

    return false;
}

// #<{(|*
//  * @fn 
//  * @brief  
//  * @param
//  * @param 
//  * @return 
//  |)}>#
// bool WaypointPublisher::JudgeApproach() {
//     geometry_msgs::PoseStamped human_pose_in_map;
//
//     if(g_found_human_flag_) {
//         tf::StampedTransform past_transform;
//         try{
//             ros::Time found_time = g_human_direction_.header.stamp;
//             g_human_direction_.header.frame_id = "velodyne";
//             // set human pos before 1.3[m].
//             g_human_direction_.pose.position.x = g_human_direction_.pose.position.x - 1.3 * cos(tf::getYaw(g_human_direction_.pose.orientation));
//             g_human_direction_.pose.position.y = g_human_direction_.pose.position.y - 1.3 * sin(tf::getYaw(g_human_direction_.pose.orientation));
//             // Get the robot positon when the system found a human.
//             tf_listener1_->waitForTransform("map", "velodyne", found_time, ros::Duration(1.0));
//             tf_listener1_->lookupTransform("map", "velodyne", found_time, past_transform); 
//             tf_listener1_->transformPose("map", g_human_direction_, human_pose_in_map);
//         }
//         catch (tf::TransformException ex) {
//             ROS_ERROR("%s", ex.what());
//         }
//         double x = (double)human_pose_in_map.pose.position.x;
//         double y = (double)human_pose_in_map.pose.position.y;
//
//         Pos robot = SetPosXY((double)past_transform.getOrigin().x(), (double)past_transform.getOrigin().y()); 
//         Pos human = SetPosXYYaw( x, y, tf::getYaw(g_human_direction_.pose.orientation)); 
//         //When a human is 10 [m] far away. 
//         if(distance_point(robot, human) >= 10.0) {
//             return false;
//         }
//         g_judge_receive_flag_ = false;
//         human_pos_.push_back(human);
//         return true;
//     }
//     else return false;
// }

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
bool WaypointPublisher::JudgeApproach() {
    geometry_msgs::PoseStamped human_pose_in_map;

    if(g_found_human_flag_) {
        tf::StampedTransform past_transform;
        try{
            ros::Time found_time = g_human_direction_.header.stamp;
            g_human_direction_.header.frame_id = "velodyne";
            // set human pos before 1.3[m].
            g_human_direction_.pose.position.x = g_human_direction_.pose.position.x - 1.3 * cos(tf::getYaw(g_human_direction_.pose.orientation));
            g_human_direction_.pose.position.y = g_human_direction_.pose.position.y - 1.3 * sin(tf::getYaw(g_human_direction_.pose.orientation));
            // Get the robot positon when the system found a human.
            tf_listener1_->waitForTransform("map", "velodyne", found_time, ros::Duration(1.0));
            tf_listener1_->lookupTransform("map", "velodyne", found_time, past_transform); 
            tf_listener1_->transformPose("map", g_human_direction_, human_pose_in_map);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        double x = (double)human_pose_in_map.pose.position.x;
        double y = (double)human_pose_in_map.pose.position.y;

        Pos robot = SetPosXY((double)past_transform.getOrigin().x(), (double)past_transform.getOrigin().y()); 
        Pos human = SetPosXYYaw( x, y, tf::getYaw(g_human_direction_.pose.orientation)); 
        //When a human is 10 [m] far away. 
        if(distance_point(robot, human) >= 10.0) {
            return false;
        }
        g_judge_receive_flag_ = false;
        human_pos_.push_back(human);
        return true;
    }
    else return false;
}

/**
 * @fn 
 * @brief  
 * @param
 * @param 
 * @return 
 */
void WaypointPublisher::MainProc() {
    // g_forward_wp_flag_ = StopCheck();   //true: To stop  false: To allow increment way point.
    bool stop_flag = StopCheck();   //true: To stop  false: To allow increment way point.
    bool line_mode = LineCheck();

    // Check if the waypoint can be updated.
    // Will not enter
    // ・A waypoint's stop flag is true.
    // ・Receive not empty human data from caffe_server.
    // ・While approaching.
    if(!stop_flag && !g_found_human_flag_ && !is_robot_approaching_) {
        tf::StampedTransform transform;
        try {
            ros::Time now = ros::Time::now();
            tf_listener1_->waitForTransform("map", "base_link", now, ros::Duration(1.0));
            tf_listener1_->lookupTransform("map", "base_link", now, transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        now_pos_.x = transform.getOrigin().x();
        now_pos_.y = transform.getOrigin().y();
        // ROS_INFO("Goal to %lf[m]", distance_point(now_pos_, current_wp_));
        // std::cout << "Goal to " << distance_point(now_pos_, current_wp_) << "[m]" << std::endl;

        // std::cout << "------------" << distance_point(now_pos_, current_wp_) << "-------------" << std::endl;
        // When the robot in goal_tolerance [m], increment a waypoint.
        if(distance_point(now_pos_, current_wp_) < goal_tolerance_) {
            ROS_INFO("Reached current waypoint. And set next waypoint.");
            IncrementWaypoint();
            stop_flag = false;
            PublishCurrentWaypoint();
        }
        //Don't skip waypoint.
        else if(is_next_stop_) {
            ;
        }
        // Time out
        else if((ros::Time::now() - wp_clock_start_) > ros::Duration(wp_timeout_)) {
            ROS_INFO("Current waypoint is timeout.");
            IncrementWaypoint();
            stop_flag = false;
            PublishCurrentWaypoint();
            std::string req("timeout");
            PublishAudioRequest(req);
        }
        // Republish current waypoint every 5[s].
        else if((ros::Time::now() - wp_repub_clock_) > ros::Duration(5)) {
            wp_repub_clock_ = ros::Time::now();
            PublishCurrentWaypoint();
        }
    }
    else if(stop_flag) {
        // Stop robot(cancel the goal)
        std::string req("pause_robot");
        PublishAudioRequest(req);
        actionlib_msgs::GoalID cancel;
        pub_goal_cancel_.publish(cancel);
    }

    // Judge approach or not.
    if(g_found_human_flag_ && !is_robot_approaching_) {
        PublishWaypointForApproach();
        ROS_INFO("Start approach.");
        std::string req("start_approach");
        PublishAudioRequest(req);
        wp_clock_start_ = ros::Time::now();
        is_robot_approaching_ = true;
        g_found_human_flag_ = false;
    }
    
    // While approaching
    if(is_robot_approaching_) {
        // ROS_INFO("Approaching");

        // Rebpulish just in case.
        if((ros::Time::now() - wp_repub_clock_) > ros::Duration(5)) {
            PublishWaypointForApproach();
        }
        bool end_approach = ApproachStateCheck();
        if(end_approach) {
            is_robot_approaching_ = false;
            actionlib_msgs::GoalID cancel;
            pub_goal_cancel_.publish(cancel);
            ros::Duration(3).sleep();
        }
        // Time Out.
        if((ros::Time::now() - wp_clock_start_) > ros::Duration(40)) {
            is_robot_approaching_ = false;
            PublishCurrentWaypoint();
        }
    }

    // For skip the waypoint from Key board
    if(wp_skip_flag_) {
        ROS_INFO("Skip has been accepted.");
        //For simple skip.
        if(!wp_array_[wp_index_].stop) {
            IncrementWaypoint();
        }
        // for start.
        else {
            wp_array_[wp_index_].stop = false;
        }
        PublishCurrentWaypoint();
        wp_skip_flag_ = false;
    }
}

