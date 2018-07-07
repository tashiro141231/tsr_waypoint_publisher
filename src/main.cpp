#include <ros/ros.h>
#include "waypoint_publisher.h"

WaypointPublisher wp;

int main(int argc, char **argv){
    
    // 引数が正しくない場合は使い方を表示
    if (argc < 3) {
        std::cout << "USAGE:" << std::endl;
        std::cout << "  $ " << argv[0] << " <wp_file_path>" << " <start_wp_number>" << std::endl;
        std::cout << "  e.g.)  $ " << argv[0] << " wp.txt" << " 0" << std::endl;
        return -1;
    }
    if(argc > 3){
        wp.set_wp_skip_num(std::string(argv[3])); //4つ目の要素を入れたら、ここまでで終了してくれる
        if(wp.get_wp_skip_num() == "skip"){
            wp.set_wp_skip_flag(true);
            std::cout<<"SKIP FLAG TRUE"<<std::endl;
        }
    }

    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle n;
    
    wp.Initialize();
    tf::TransformListener li1(ros::Duration(1));
    tf::TransformListener li2(ros::Duration(1));
    tf::TransformListener li3(ros::Duration(1));
    wp.tf_listener1_ = &li1;
    wp.tf_listener2_ = &li2;
    wp.tf_listener3_ = &li3;
    // wp.SetSpecificPointIndex();
   

    while(ros::ok()){
        ROS_INFO("hello hello");
    }
  
    return 0;
}
