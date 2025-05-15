#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"
#include "toad_auto_drive/msg/trash_info.hpp"

using std::placeholders::_1;

class ToadControllerNode : public rclcpp::Node{
    public :
    ToadControllerNode() : Node("toad_controller_node"){
        cap_subscription_ = tihs->create_subscription<trash_info::msg:::TrashInfo>(
            "/trash_detect", 10,
            std::bind(&ToadControllerNode::trashDetectCallback, tihs, _1);
        );
    }
    private :

    void trashDetectCallback(){
        
    }


};

int main(int argc, char * argv[]){

}