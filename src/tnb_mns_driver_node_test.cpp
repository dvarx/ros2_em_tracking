#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/Float32MultiArray.h>

ros::Publisher currents_pub;
const double delta_T=10e-3;     //publishing time interval
const double i_amp=2.0;         //amplitude of desirec currents [A]
const double omega=2*M_PI;      //frequency of reference waveform

std::vector<double> currents={1.0,0,0,0,0,0};

std_msgs::Float32MultiArray msg;

void tnb_mns_driver_timer_cb(const ros::TimerEvent&){
    ros::Time time = ros::Time::now();
    double current_time=time.sec+1e-9*time.nsec;

    double current=i_amp*sin(omega*current_time);

    for(int i=0; i<6; i++)
        msg.data[i]=0;

    msg.data[0]=current;

    currents_pub.publish(msg);
}

//main
int main(int argc,char** argv){
    ros::init(argc, argv, "tnb_mns_driver_node_test",ros::init_options::NoSigintHandler);

    ROS_INFO("staring up tnb_mns_driver test node");

    for(int i=0; i<6; i++)
        msg.data.push_back(0);

    ros::NodeHandle nh("~");

    currents_pub = nh.advertise<std_msgs::Float32MultiArray>("/tnb_mns_driver/des_currents_reg", 256);

    //main timer shows callback function send the driver messages
    ros::Timer timer = nh.createTimer(ros::Duration(delta_T), tnb_mns_driver_timer_cb);

    ROS_INFO("Start spinning");

    ros::spin();
}