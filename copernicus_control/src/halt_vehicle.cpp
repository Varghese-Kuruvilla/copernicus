#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <iostream>

using namespace std; 
class Stopvehicle {
    private:
    ros::Subscriber human_stop_status; //Flag which indicates that a human is detected
    ros::Publisher cmdvel_pub; //Publisher that publishes command velocity to stop the vehicle
    int flag = 0; 

    public:
        Stopvehicle(ros::NodeHandle *nh){
            cmdvel_pub = nh->advertise<geometry_msgs::Twist>("stop/cmd_vel",100);
            human_stop_status = nh->subscribe("person", 1000 ,&
            Stopvehicle::stop_callback,this);
            //Subscribe to /human_stop_status and publish zeros on the command velocity topics
        }

        void stop_callback(const std_msgs::Int32& msg)
        {
            std::cout<<"Inside stop_callback"<<std::endl;
            this->flag = msg.data;
            cout<<"this->flag"<<this->flag<<endl;
            publish_cmdvel();
        }

        void publish_cmdvel()
        {
            //Publish cmd velocity to halt the vehicle
            cout<<"this->flag"<<this->flag<<endl;
            geometry_msgs::Twist msg;
            if(this->flag == 1) //Human is seen close to the camera
            {
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                cmdvel_pub.publish(msg);
            }

        }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"halt_vehicle");
    ros::NodeHandle nh;
    Stopvehicle Stopvehicleobj = Stopvehicle(&nh);
    ros::spin();
}
