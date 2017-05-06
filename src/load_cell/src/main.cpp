/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <cctype>
#include <string>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 10, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::Float64>("SYD/tension", 10);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(100);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String result;
            result.data = ser.read(ser.available());
			double tension = 0.0;
			int i=0;
			int length = result.data.length();
			bool isPositive = false;
			for(i=0; i<length-4; i++)
			{
				if(result.data[i]==45)
				{
					isPositive = false;
					break;
				}
				else if(isdigit(result.data[i]))
				{
					isPositive = true;
					break;
				}
			}
			if(isPositive)
			{
				result.data = result.data.substr(i,7);
			}
			else
			{
				result.data = result.data.substr(i,8);
			}
			char *end;
			std_msgs::Float64 topic_result;
			topic_result.data = (std::strtod(result.data.c_str(),&end))/2.0*4.53592*9.81 + 0.05;

            read_pub.publish(topic_result);
        }
        loop_rate.sleep();

    }
}

