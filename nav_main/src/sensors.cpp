#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <librealsense2/rs.hpp>
#include <serial/serial.h>
#include <cstdint>
#include <sensor_msgs/Imu.h>

/*IMU数据包结构*/
typedef struct SendPacket {
    uint8_t header;    
    float ac_x;           
    float ac_y;         
    float ac_z;           
    float yaw_v;          
    float pitch_v;         
    float roll_v;          
    uint8_t ender;        
} __attribute__((packed)) SendPacket;
const size_t PACKET_SIZE = sizeof(SendPacket);

serial::Serial ser;     //串口对象

// 从缓冲区读取一个完整的数据包
bool readPacket(SendPacket& packet) {
    std::vector<uint8_t> buffer(PACKET_SIZE);
    while (ser.available() > 0) {
        // 逐字节读取
        uint8_t byte;
        ser.read(&byte, 1);

        // 判断帧头
        if (byte == 0x5A) {  // 检测到帧头
            ser.read(buffer.data() + 1, PACKET_SIZE - 1);  // 读取剩余字节
            buffer[0] = 0x5A;                              // 填充帧头
            packet = *reinterpret_cast<SendPacket*>(buffer.data());

            // 验证帧尾
            if (packet.ender == 0x5C) {
                return true; // 有效包
            } else {
                ROS_WARN_STREAM("Invalid frame end: " << std::hex << packet.ender);
                continue; // 跳过无效包
            }
        }
    }
    return false; // 未找到有效数据包
}



int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "sensor_node");
    ros::NodeHandle nh;
    //IMU话题
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

    /*启动串口*/
    try {
        // 打开串口
        ser.setPort("/dev/ttyACM0"); 
        ser.setBaudrate(115200);    
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // 设置发布频率
    ros::Rate loop_rate(2000); 
    int drop_i = 0;
    while (ros::ok()) {
        SendPacket data;
        if (readPacket(data)) {
            // 输出解析数据
            // ROS_INFO_STREAM("IMU:" << data.ac_x << "\t" << data.ac_y << "\t" << data.ac_z << "\t"
            //                              << data.yaw_v << "\t" << data.pitch_v << "\t" << data.roll_v << "\t");
            //单位换算常量
            const double mg_to_mps2 = 9.8 / 1000.0; // mg 转换为 m/s^2
            const double mdps_to_rps = 3.14159265359 / (1000.0 * 180.0); // mdps 转换为 rad/s
            // 创建 IMU 消息
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_frame";

            //xyz - roll pitch yaw √
            //xzy - roll yaw pitch √
            //yxz - pitch roll yaw √
            //yzx - pitch yaw roll √
            //zxy - yaw roll pitch √
            //zyx - yaw pitch roll √
            //why???
            

            // // 设置线性加速度
            // imu_msg.linear_acceleration.x = data.ac_z * mg_to_mps2;
            // imu_msg.linear_acceleration.y = data.ac_x * mg_to_mps2;
            // imu_msg.linear_acceleration.z = data.ac_y * mg_to_mps2;
            // imu_msg.linear_acceleration_covariance[0] = -1; // 无实际协方差数据

            // // 设置角速度
            // imu_msg.angular_velocity.x = data.yaw_v * mdps_to_rps;
            // imu_msg.angular_velocity.y = -data.roll_v * mdps_to_rps;
            // imu_msg.angular_velocity.z = -data.pitch_v * mdps_to_rps;
            // imu_msg.angular_velocity_covariance[0] = -1; // 无实际协方差数据

            //设置角度协方差
            imu_msg.orientation_covariance[0] = 99999.9; // Roll 方差
            imu_msg.orientation_covariance[4] = 99999.9; // Pitch 方差
            imu_msg.orientation_covariance[8] = 99999.9; // Yaw 方差

            // // 设置线性加速度
            imu_msg.linear_acceleration.x = data.ac_x * mg_to_mps2;
            imu_msg.linear_acceleration.y = data.ac_y * mg_to_mps2;
            imu_msg.linear_acceleration.z = data.ac_z * mg_to_mps2;
            // imu_msg.linear_acceleration_covariance[0] = -1; // 无实际协方差数据

            // 设置角速度
            imu_msg.angular_velocity.x = data.roll_v * mdps_to_rps;
            imu_msg.angular_velocity.y = data.pitch_v * mdps_to_rps;
            imu_msg.angular_velocity.z = data.yaw_v * mdps_to_rps;
            // imu_msg.angular_velocity_covariance[0] = -1; // 无实际协方差数据

            // 设置线性加速度
            // imu_msg.linear_acceleration.x = data.ac_z * mg_to_mps2;         //向下
            // imu_msg.linear_acceleration.y = -data.ac_y * mg_to_mps2;         //向右
            // imu_msg.linear_acceleration.z = data.ac_x * mg_to_mps2;         //向后
            // imu_msg.linear_acceleration_covariance[0] = -1; // 无实际协方差数据

            // // 设置角速度
            // imu_msg.angular_velocity.x = data.yaw_v * mdps_to_rps;         //向下
            // imu_msg.angular_velocity.y = -data.pitch_v * mdps_to_rps;        //向右
            // imu_msg.angular_velocity.z = data.roll_v * mdps_to_rps;          //向后
            // imu_msg.angular_velocity_covariance[0] = -1; // 无实际协方差数据


            // imu_msg.linear_acceleration.x/*向左*/ = -data.ac_y * mg_to_mps2; /*向右*/
            // imu_msg.linear_acceleration.y/*向上*/ = -data.ac_z * mg_to_mps2; /*向下*/
            // imu_msg.linear_acceleration.z/*向后*/ = data.ac_x * mg_to_mps2; /*向后*/
            // imu_msg.linear_acceleration_covariance[0] = -1; // 无实际协方差数据
            // // 设置角速度
            // imu_msg.angular_velocity.x = -data.pitch_v * mdps_to_rps;         //原y，pitch，-
            // imu_msg.angular_velocity.y = -data.yaw_v * mdps_to_rps;        //原z，yaw，-
            // imu_msg.angular_velocity.z = data.roll_v * mdps_to_rps;          //原x，roll，+
            // imu_msg.angular_velocity_covariance[0] = -1; // 无实际协方差数据


            // 发布 IMU 消息
            if(drop_i >= 5){
                imu_pub.publish(imu_msg);
                drop_i = 0;
            }
            drop_i ++;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}