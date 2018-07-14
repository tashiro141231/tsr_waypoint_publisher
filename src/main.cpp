#include <ros/ros.h>
#include <thread>
#include <cstdlib>
#include "waypoint_publisher.h"

int main(int argc, char **argv){
    
    // 引数が正しくない場合は使い方を表示
    if (argc < 3) {
        std::cout << "USAGE:" << std::endl;
        std::cout << "  $ " << argv[0] << " <wp_file_path>" << " <start_wp_number>" << std::endl;
        std::cout << "  e.g.)  $ " << argv[0] << " wp.txt" << " 0" << std::endl;
        return -1;
    }
    if(argc > 3){
        WaypointPublisher::set_wp_skip_num(std::string(argv[3])); //4つ目の要素を入れたら、ここまでで終了してくれる
        if(WaypointPublisher::get_wp_skip_num() == "skip"){
            WaypointPublisher::set_wp_skip_flag(true);
            std::cout<<"SKIP FLAG TRUE"<<std::endl;
        }
    }

    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle n;
    
    std::string waypoint_file = argv[2];
    WaypointPublisher::Initialize();
    tf::TransformListener li1(ros::Duration(1));
    tf::TransformListener li2(ros::Duration(1));
    tf::TransformListener li3(ros::Duration(1));
    WaypointPublisher::tf_listener1_ = &li1;
    WaypointPublisher::tf_listener2_ = &li2;
    WaypointPublisher::tf_listener3_ = &li3;

    // Keyboardによるwpの更新関数を別スレッドで回しておく
    std::thread key_interupt(WaypointPublisher::ForwardWaypointByKeyboardInterrupt);
    //Waypointの読み込み
    if(WaypointPublisher::ReadWaypointFile( waypoint_file ) == -1) {
        ROS_INFO("Fatal error: Could not read waypoint file.");
        std::exit(0);
    }
    // wp.SetSpecificPointIndex();
   
   getchar();
   WaypointPublisher::PublishCurrentWaypoint();

    while(ros::ok() && WaypointPublisher::getGoalFlag()){
        WaypointPublisher::PublishWaypointArray();
        WaypointPublisher::MainProc();
    }
  
    return 0;
}
