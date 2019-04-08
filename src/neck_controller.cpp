#include <ros/ros.h> 
#include <iostream> 
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <my_neck/neck_state.h>
#include <my_neck/neck_command.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

std::string neck_name;

float Current_Roll=0;
float Current_Yaw=0;

float Current_X=0;
float Current_Y=0;
float Current_Z=0;

int Target_Roll=0;
int Target_Yaw=0;

float Target_Roll_f=0;
float Target_Yaw_f=0;

my_neck::neck_state neck_state_msg;
my_neck::neck_command neck_command_msg;

ros::Publisher neck_state_pub;
ros::ServiceClient client;

dynamixel_workbench_msgs::DynamixelCommand neck_command;

float RadInit(float rad)
{
    float temp;
    temp=rad;
while (temp>3.1415926)
{
temp=temp-2*3.1415926;
}
while (temp<-3.1415926)
{
temp=temp+2*3.1415926;
}
return temp;
}

int IntInit(int i)
{
    int temp=i;
while (temp>4096)
{
temp=temp-4096;
}
while (temp<0)
{
temp=temp+4096;
}
return temp;
}

void stateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    Current_Roll=RadInit(msg->position[1]-0.409573);
    Current_Yaw=RadInit(msg->position[0]+1.748738);
    neck_state_msg.roll_angle=Current_Roll;
    neck_state_msg.yaw_angle=Current_Yaw;
    neck_state_pub.publish(neck_state_msg);
   // ROS_INFO("Roll:%f Yaw:%f\n",Current_Roll,Current_Yaw); 
}

void neck_commandCallback(const my_neck::neck_command::ConstPtr& msg)
{
    Target_Roll=(int)(((float)msg->target_yaw_angle*4096)/(2*3.1415926))+2332;
    Target_Yaw=(int)(((float)msg->target_roll_angle*4096)/(2*3.1415926))+930;
    Target_Roll=IntInit(Target_Roll);
    Target_Yaw=IntInit(Target_Yaw);

    if(Target_Roll>4000) 
    {
        Target_Roll=4000;
    }
    else if(Target_Roll<1000)
    {
        Target_Roll=1000;
    }

    if(Target_Yaw>1240) 
    {
        Target_Yaw=1240;
    }
    else if(Target_Yaw<400)
    {
        Target_Yaw=400;
    }

    neck_command.request.id=6;
    neck_command.request.addr_name="Goal_Position";
    neck_command.request.value=Target_Roll;

 //   ROS_INFO("Roll:%d   Yaw:%d\n",Target_Roll,Target_Yaw);

    client.call(neck_command);
    
    neck_command.request.id=4;
    neck_command.request.addr_name="Goal_Position";
    neck_command.request.value=Target_Yaw;

    ROS_INFO("Roll:%d   Yaw:%d\n",Target_Roll,Target_Yaw);

    client.call(neck_command);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "neck_controller"); //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错） 
    ros::NodeHandle n; 

    neck_name = "fuck";

    ros::Subscriber motor_states_sub = n.subscribe("/dynamixel_workbench/joint_states", 1000, stateCallback);
    neck_state_pub = n.advertise<my_neck::neck_state>("neck_state", 1000);
    ros::Subscriber neck_command_sub = n.subscribe("neck_command", 1000, neck_commandCallback);
    
    ros::Publisher test_pub = n.advertise<my_neck::neck_command>("neck_command", 1000);

    client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

    ros::Rate loop_rate(100); 

    int flag=1;
    while(ros::ok()) 
    { 
       //ROS_INFO("1\n");

         static tf::TransformBroadcaster br;
         tf::Transform transform;

        Current_X=0.01*(-10*sin(Current_Yaw)*cos(Current_Roll));
        Current_Y=0.01*(-10*sin(Current_Yaw)*sin(Current_Roll));
        Current_Z=0.01*(10*cos(Current_Yaw)+10);

        transform.setOrigin( tf::Vector3(Current_X, Current_Y,Current_Z) );

        tf::Quaternion q;
        q.setRPY(Current_Yaw, 0, Current_Roll);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "neck", neck_name));

/*
if(flag==1)
{
        neck_command_msg.target_yaw_angle=1;
        neck_command_msg.target_roll_angle=1;
        test_pub.publish(neck_command_msg);
        flag=2;
}
else if(flag==2)
{
          neck_command_msg.target_yaw_angle=0;
        neck_command_msg.target_roll_angle=-1;
        test_pub.publish(neck_command_msg);        
        flag=3; 
}
else if(flag==3)
{
          neck_command_msg.target_yaw_angle=-1;
        neck_command_msg.target_roll_angle=1;
        test_pub.publish(neck_command_msg);        
        flag=1; 
}
*/
       ros::spinOnce();
       loop_rate.sleep();  
    } 
    //关闭串口 
    return 0; 
} 

