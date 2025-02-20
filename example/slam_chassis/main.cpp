// ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

// cpp
#include <iostream>
#include <assert.h>
#include <thread>
#include <vector>

#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

float limit(float a, float min, float max){
    a = a > min ? a : min; 
    a = a < max ? a : max;
    return a;
}

// slam chassis 
class SLAMChassis{
private:
    class Param{
        public:
            /*
            "cmd_vel_topic"
            "cmd_pos_topic"
            "max_x_vel"    
            "max_yaw_vel"  
            "max_height"   
            "tf_thresh"    
            "ctrl_rate"         
            "stop_time"    
            */
            std::string cmd_vel_topic, cmd_pos_topic, power_topic;
            float max_x_vel, max_yaw_vel, max_height, tf_thresh, ctrl_rate, stat_rate, stop_time;
            Param(){}
            void get(){
                ros::NodeHandle n("~");
                // get param
                if(!n.getParam("cmd_vel_topic", cmd_vel_topic)) ROS_ERROR("SLAMChassis: No param cmd_vel_topic.");
                if(!n.getParam("cmd_pos_topic", cmd_pos_topic)) ROS_ERROR("SLAMChassis: No param cmd_pos_topic.");
                if(!n.getParam("power_topic", power_topic))     ROS_ERROR("SLAMChassis: No param power_topic.");

                if(!n.getParam("max_x_vel", max_x_vel))         ROS_ERROR("SLAMChassis: No param max_x_vel.");
                if(!n.getParam("max_yaw_vel", max_yaw_vel))     ROS_ERROR("SLAMChassis: No param max_yaw_vel.");
                if(!n.getParam("max_height", max_height))       ROS_ERROR("SLAMChassis: No param max_height.");
                if(!n.getParam("tf_thresh", tf_thresh))         ROS_ERROR("SLAMChassis: No param tf_thresh.");

                if(!n.getParam("ctrl_rate", ctrl_rate))         ROS_ERROR("SLAMChassis: No param ctrl_rate.");
                if(!n.getParam("stat_rate", stat_rate))         ROS_ERROR("SLAMChassis: No param stat_rate.");
                if(!n.getParam("stop_time", stop_time))         ROS_ERROR("SLAMChassis: No param stop_time.");
            }
    };
    
    ros::NodeHandle _nh;
    
    // hyperparameters
    Param _param;
    
    ros::Subscriber _sub_cmd_vel;
    ros::Subscriber _ser_cmd_pos;
    ros::Publisher  _pub_power;
    
    geometry_msgs::Twist _cmd_vel;
    geometry_msgs::Twist _cmd_pos;
    int _cmd_vel_tick = 0; bool _cmd_pos_set = false;
    bool _in_ctrl = false;
public:
    SLAMChassis(){
        _param.get();
        _sub_cmd_vel = _nh.subscribe<geometry_msgs::Twist>(_param.cmd_vel_topic, 1, &SLAMChassis::cmdVelHandler, this);
        _ser_cmd_pos = _nh.subscribe<geometry_msgs::Twist>(_param.cmd_pos_topic, 1, &SLAMChassis::cmdPoshandler, this);
        _pub_power   = _nh.advertise<std_msgs::Float32MultiArray>(_param.power_topic, 1);

        _cmd_vel.linear.x = 0;_cmd_vel.linear.y = 0;_cmd_vel.linear.z = 0;
        _cmd_vel.angular.x = 0;_cmd_vel.angular.y = 0;_cmd_vel.angular.z = 0;
        _cmd_pos.linear.x = 0;_cmd_pos.linear.y = 0;_cmd_pos.linear.z = 0;
        _cmd_pos.angular.x = 0;_cmd_pos.angular.y = 0;_cmd_pos.angular.z = 0;
    }

    ~SLAMChassis(){
        
    }

    void cmdVelHandler(const geometry_msgs::TwistConstPtr &msg){
        // std::cout << "cmd_vel" << std::endl;
        if(!_in_ctrl) return;
        _cmd_vel = *msg;
        // limit
        _cmd_vel.linear.x  = limit(_cmd_vel.linear.x, -_param.max_x_vel, _param.max_x_vel);
        _cmd_vel.angular.z = limit(_cmd_vel.angular.z, -_param.max_yaw_vel, _param.max_yaw_vel);

        _cmd_vel_tick = 0;
    }

    void cmdPoshandler(const geometry_msgs::TwistConstPtr &msg){
        if(!_in_ctrl) return;
        _cmd_pos = *msg;
        _cmd_pos.linear.z = limit(_cmd_pos.linear.z, 0.0, _param.max_height);
        _cmd_pos_set = true;
    }

    // chassis running
    void run(){
        // initialize chassis
        DIABLO::OSDK::HAL_Pi Hal;
        if(Hal.init()) return;
        DIABLO::OSDK::Vehicle vehicle(&Hal);                                  //Initialize Onboard SDK
        if(vehicle.init()) return;
        vehicle.telemetry->activate();

        ros::Rate waiting(0.5);

        waiting.sleep();
        //stat_thread：调用 stat() 函数，定时读取并发布车辆状态（例如电源信息）。
        std::thread stat_thread(&SLAMChassis::stat, this, vehicle.telemetry);

        waiting.sleep();
        //ctrl_thread：调用 ctrl() 函数，周期性发送运动控制命令。
        std::thread ctrl_thread(&SLAMChassis::ctrl, this, vehicle.movement_ctrl);

        ros::spin();
        ctrl_thread.join();
        stat_thread.join();
        return;
    }

    void stat(DIABLO::OSDK::Telemetry *telemetry){
        OSDK_Push_Data_Freq_Select_t freq; int rate = (int)_param.stat_rate;


        if     (rate == 0)   freq = OSDK_PUSH_DATA_OFF;
        else if(rate == 1)   freq = OSDK_PUSH_DATA_1Hz;
        else if(rate == 10)  freq = OSDK_PUSH_DATA_10Hz;
        else if(rate == 50)  freq = OSDK_PUSH_DATA_50Hz;
        else if(rate == 100) freq = OSDK_PUSH_DATA_100Hz;
        else if(rate == 500) freq = OSDK_PUSH_DATA_500Hz;
        else {
            ROS_WARN("SLAMChassis: Invalid rate = %d, set freq = OSDK_PUSH_DATA_OFF. (0, 1, 10, 50, 100, 500)", rate);
            freq = OSDK_PUSH_DATA_OFF;
        }

        // subscribe from chassis
        telemetry->configTopic(DIABLO::OSDK::TOPIC_POWER, freq);
        // update
        telemetry->configUpdate(); 

        // init power_msg
        //初始化一个 std_msgs::Float32MultiArray 消息，用于发布电源数据（包含电压、电流、电容能量和电量百分比）。
        std_msgs::Float32MultiArray power_msg;
        power_msg.layout.data_offset = 0;
        std_msgs::MultiArrayDimension power_msg_dim; 
        power_msg_dim.label = "[voltage, current, capacitor_energy, power_percent]";
        power_msg_dim.size = 4; power_msg_dim.stride = 1;
        power_msg.layout.dim.push_back(power_msg_dim);

        while(ros::ok()){
            if(telemetry->newcome & 0x02){
                power_msg.data = std::vector<float>{
                    telemetry->power.voltage, 
                    telemetry->power.current, 
                    telemetry->power.capacitor_energy, 
                    (float)telemetry->power.power_percent
                };
                _pub_power.publish(power_msg);
                telemetry->eraseNewcomeFlag(0xFD);
            }
        }
    }

    void ctrl(DIABLO::OSDK::Movement_Ctrl* movement_ctrl){
        ros::Rate rate(_param.ctrl_rate);

        // 25hz, 0.04s*5 = 0.2s, stop_time / (1/ctrl_rate) = stop_time * ctrl_rate
        int stop_tick = (int)(_param.ctrl_rate * _param.stop_time);

        // start control
        while(ros::ok()){      
            rate.sleep();     

            // if no data is received within 0.2 seconds, reset _cmd_vel
            //计算一个 stop_tick，即若连续没有收到新的 cmd_vel 指令达到一定次数，则归零运动命令（防止车辆持续运动）。
            if(_cmd_vel_tick++ > stop_tick){
                // set _cmd_vel to 0
                _cmd_vel.linear.x = 0;_cmd_vel.linear.y = 0;_cmd_vel.linear.z = 0;
                _cmd_vel.angular.x = 0;_cmd_vel.angular.y = 0;_cmd_vel.angular.z = 0;
            } 

            // get control
            if(!movement_ctrl->in_control())
            {
                printf("SLAMClassis: Getting movement ctrl ...\n");
                if(movement_ctrl->obtain_control() == 0) _in_ctrl = true;
                else{_in_ctrl = false; continue;}
            }

            // _cmd_pos is a low-frequency control
            if(_cmd_pos_set){
                // reset flag
                _cmd_pos_set = false;
                // set transform
                if(_cmd_pos.linear.z > _param.tf_thresh) movement_ctrl->SendTransformUpCmd();
                else                                     movement_ctrl->SendTransformDownCmd();
                // set cmd_pos
                movement_ctrl->ctrl_data.up = _cmd_pos.linear.z;
                if(movement_ctrl->SendMovementCtrlCmd() != 0) std::cout << "SLAMChassis: SendMovementCtrlCmd failed!" << std::endl;
                continue;
            }

            // set cmd_vel
            // std::cout << _cmd_vel.linear.x << " " << _cmd_vel.angular.z << std::endl;
            movement_ctrl->ctrl_data.forward = _cmd_vel.linear.x;
            movement_ctrl->ctrl_data.left =  _cmd_vel.angular.z;
            if(movement_ctrl->SendMovementCtrlCmd() != 0) std::cout << "SLAMChassis: SendMovementCtrlCmd failed!" << std::endl;

        }
        return;
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_chassis");

    SLAMChassis chassis;

    // ros::Rate rate(0.5); rate.sleep();
    // std::thread run_thread(&SLAMChassis::run, &chassis);
    chassis.run();

    return 0;
}
