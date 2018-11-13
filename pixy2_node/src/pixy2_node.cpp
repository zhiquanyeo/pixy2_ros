#include <ros/ros.h>
#include <pixy2_msgs/PixyData.h>
#include <pixy2_msgs/PixyBlock.h>

#include "libpixyusb2.h"

class Pixy2Node
{
public:
    Pixy2Node();
    void spin();
private:
    void update();

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Rate rate_;
    //tf::TransformBroadcaster td_broadcaster_;

    ros::Publisher publisher_;
    ros::Subscriber servo_subscriber_;
    std::string frame_id;

    bool use_servos_;

    Pixy2 pixy;
};

Pixy2Node::Pixy2Node() :
                node_handle_(),
                private_node_handle_("~"),
                use_servos_(false),
                rate_(50.0)
{
    private_node_handle_.param<std::string>(std::string("frame_id"), frame_id, std::string("pixy_frame"));

    double rate;
    private_node_handle_.param("rate", rate, 50.0);
    rate_ = ros::Rate(rate);

    private_node_handle_.param("use_servos", use_servos_, false);

    if (use_servos_) {
        //servo_subscriber_ = node_handle_.subscribe("servo_cmd", 20, &Pixy2Node::setServo, this);
    }

    int ret = pixy.init();
    if (ret != 0) {
        ROS_FATAL("Pixy2Node - %s - Failed to open with USB error %d!", __FUNCTION__, ret);
        ROS_BREAK();
    }

    publisher_ = node_handle_.advertise<pixy2_msgs::PixyData>("block_data", 50.0);
}

void Pixy2Node::update()
{
    // Query pixy for blocks
    pixy.ccc.getBlocks();

    pixy2_msgs::PixyData data;

    if (pixy.ccc.numBlocks > 0) {
        data.header.stamp = ros::Time::now();

        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            pixy2_msgs::PixyBlock pixy_block;

            Block raw_block = pixy.ccc.blocks[i];

            if (raw_block.m_signature > CCC_MAX_SIGNATURE) {
                pixy_block.type = pixy2_msgs::PixyBlock::COLOR_CODE;
            }
            else {
                pixy_block.type = pixy2_msgs::PixyBlock::NORMAL_SIGNATURE;
            }

            pixy_block.signature = raw_block.m_signature;
            pixy_block.roi.x_offset = raw_block.m_x;
            pixy_block.roi.y_offset = raw_block.m_y;
            pixy_block.roi.width = raw_block.m_width;
            pixy_block.roi.height = raw_block.m_height;
            pixy_block.roi.do_rectify = false;

            pixy_block.angle = (pixy_block.type == pixy2_msgs::PixyBlock::COLOR_CODE) ?
                                raw_block.m_angle : 0.0;

            data.blocks.push_back(pixy_block);
        }
    }
    else if (pixy.ccc.numBlocks < 0) {
        ROS_INFO("Pixy2 read error");
        return;
    }

    publisher_.publish(data);
}

void Pixy2Node::spin()
{
    while(node_handle_.ok()) {
        update();

        ros::spinOnce();
        rate_.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pixy2_node");

    ROS_INFO("Pixy2Node for ROS");

    Pixy2Node myPixy2;
    myPixy2.spin();

    return 0;
}