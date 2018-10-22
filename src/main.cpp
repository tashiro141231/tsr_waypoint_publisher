#include <ros/ros.h>
#include <thread>
#include <cstdlib>
#include "waypoint_publisher.h"

int main(int argc, char **argv){

    std::vector<std::string> args(argv, argv + argc);
    int start_num;
    
    // 引数が正しくない場合は使い方を表示
    if (argc < 4 || argc > 5) {
        std::cout << "USAGE:" << std::endl;
        std::cout << "  $ " << argv[0] << " <wp_file_path>" << " <stop_position> "<< " <line_position> " << " <start_wp_number>" << std::endl;
        std::cout << "  e.g.)  $ " << argv[0] << " wp.txt stop_pos.txt" << " 0" << std::endl;
        return -1;
    }
    if(argc == 4){
        WaypointPublisher::set_wp_skip_num(args[3]); //4つ目の要素を入れたら、ここまでで終了してくれる
        try {
            start_num = std::stoi(args[3]);
        }
        catch(const std::invalid_argument& e) {
            start_num = 0;
            std::cout << "Invakid start number. start from beginning of wp." << std::endl;
        }
        if(start_num  < 0) {
            start_num = 0;
            std::cout << "Invakid start number. start from beginning of wp." << std::endl;
        }
    }

    ros::init(argc, argv, "tsr_waypoint_publisher");
    ros::NodeHandle n;
    ros::Rate rate(20);
    
    // Enterでスタート
    getchar();

    ROS_INFO("Init start.");
    std::string waypoint_file = "/home/kenaf/catkin_ws/src/tsukuchalle2018/waypoint/" + args[1];
    std::string stop_pos = "/home/kenaf/catkin_ws/src/tsukuchalle2018/waypoint/stop_position/" + args[2];
    std::string line_wp = "/home/kenaf/catkin_ws/src/tsukuchalle2018/waypoint/" + args[3];
    std::cout << waypoint_file << std::endl;
    tf::TransformListener li1(ros::Duration(1));
    tf::TransformListener li2(ros::Duration(1));
    tf::TransformListener li3(ros::Duration(1));
    WaypointPublisher::tf_listener1_ = &li1;
    WaypointPublisher::tf_listener2_ = &li2;
    WaypointPublisher::tf_listener3_ = &li3;

    //Waypointの読み込み
    // if(WaypointPublisher::ReadWaypointFile("/home/kenaf/catkin_ws/src/tsukuchalle2018/waypoint/merged_wp_0721.txt") == -1) {
    if(WaypointPublisher::ReadWaypointFile(waypoint_file) == -1) {
        ROS_INFO("Fatal error: Could not read waypoint file.");
        std::exit(0);
    }
    WaypointPublisher::SetWayPointNumText();
    WaypointPublisher::Initialize(start_num);
    ROS_INFO("Initialization has been completed.");
    std::cout << "Waypoint starts from " << start_num << "." << std::endl; 

    //一時停止位置や信号位置などの事前情報のセット
    WaypointPublisher::SetSpecificPoint(stop_pos, line_wp);
   
    // getchar();
    // Keyboardによるwpの更新関数を別スレッドで回しておく
    std::thread key_interupt(WaypointPublisher::ForwardWaypointByKeyboardInterrupt);
    WaypointPublisher::PublishCurrentWaypoint();

    ROS_INFO("Loop start.");
    while(ros::ok() && !WaypointPublisher::getGoalFlag()){
        WaypointPublisher::PublishWaypointArray();
        if(WaypointPublisher::search_human_flag()) {
            WaypointPublisher::PublishSearchHumanRequest(true);
        }
        WaypointPublisher::PublishWayPointNumText();
        WaypointPublisher::PublishSearchHumanRequest(true);   // For debug
        WaypointPublisher::MainProc();
        ros::spinOnce();
        rate.sleep();
    }
  
    return 0;
}

