// #include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <roboteq_motor_controller_driver/querylist.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>

#include <stdlib.h>
#include <stdio.h>

class RoboteqDriver
{
public:
	RoboteqDriver()
	{
		initialize(); //constructor - Initialize
	}

	~RoboteqDriver()
	{
		if (ser.isOpen())
		{
			ser.close();
		}
	}

private:
	serial::Serial ser;
	std::string port;
	std::string name_;
	int32_t baud;
	ros::Publisher read_publisher;
	ros::Publisher diagnostic_pub;
	ros::Subscriber cmd_vel_sub;

	int channel_number_1;
	int channel_number_2;
	int frequencyH;
	int frequencyL;
	int frequencyG;
	ros::NodeHandle nh;

	void initialize()
	{

		nh.getParam("port", port);
		nh.getParam("baud", baud);
		cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &RoboteqDriver::cmd_vel_callback, this);

		connect();
	}

	int bitwise_parse(int input){
		if (input & 1) {
			return 1;
		} else if (input & 2) {
			return 2;
		} else if (input & 4) {
			return 3;
		} else if (input & 8) {
			return 4;
		} else if (input & 16) {
			return 5;
		} else if (input & 32) {
			return 6;
		} else if (input & 64) {
			return 7;
		} else if (input & 128) {
			return 8;
		} else {
			return 0;
		}
	}

	void connect()
	{

		try
		{

			ser.setPort(port);
			ser.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{

			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
			;
		}
		if (ser.isOpen())
		{

			ROS_INFO_STREAM("Serial Port initialized\"");
		}
		else
		{
			// ROS_INFO_STREAM("HI4");
			ROS_INFO_STREAM("Serial Port is not open");
		}
		run();
	}

	void cmd_vel_callback(const geometry_msgs::Twist &msg)
	{
		std::stringstream cmd_sub;
		cmd_sub << "!G 1"
				<< " " << msg.linear.x << "_"
				<< "!G 2"
				<< " " << msg.angular.z << "_";

		ser.write(cmd_sub.str());
		ser.flush();
		ROS_INFO_STREAM(cmd_sub.str());
	}

	ros::NodeHandle n;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;

	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
	{
		std::stringstream str;
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
			<< "%\clsav321654987";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
	{
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
	{
		std::stringstream str;
		str << "%" << request.userInput << " "
			<< "_";
		ser.write(str.str());
		ser.flush();
		response.result = ser.read(ser.available());

		ROS_INFO_STREAM(response.result);
		return true;
	}

	void initialize_services()
	{
		n = ros::NodeHandle();
		//ros::NodeHandle nh_priv("~");
		configsrv = n.advertiseService("/config_service", &RoboteqDriver::configservice, this);
		commandsrv = n.advertiseService("/command_service", &RoboteqDriver::commandservice, this);
		maintenancesrv = n.advertiseService("/maintenance_service", &RoboteqDriver::maintenanceservice, this);
	}

	void run()
	{
		initialize_services();
		std_msgs::String str1;
		ros::NodeHandle nh;
		nh.getParam("frequencyH", frequencyH);
		nh.getParam("frequencyL", frequencyL);
		nh.getParam("frequencyG", frequencyG);

		nh.getParam("name",name_);

		typedef std::string Key;
		typedef std::string Val;
		std::map<Key, Val> map_sH;
		nh.getParam("queryH", map_sH);
		std::map<Key, Val> fault_map;
		nh.getParam("fault_list", fault_map);

		std::stringstream ss0;
		std::stringstream ss1;
		std::stringstream ss2;
		std::stringstream ss3;
		std::vector<std::string> KH_vector;

		int fault_iter = 0;

		// Create command for roboteq to send readings at set frequency
		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";
		int counter = 0; // Not sure how iterator works just yet
		for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
		{
			Key KH = iter->first;
			KH_vector.push_back(KH);

			Val VH = iter->second;
			if (VH.compare("?FF")==0){
				fault_iter = counter;
				ROS_INFO("Fault flag found: %d", fault_iter);
				ROS_INFO(VH.c_str());
			}
			ss1 << VH << "_";
			counter++;
		}
		ss1 << "# " << frequencyH << "_";

		std::vector<ros::Publisher> publisherVecH;
		for (int i = 0; i < KH_vector.size(); i++)
		{
			publisherVecH.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KH_vector[i], 100));
		}

		// Load fault list into system
		std::vector<std::string> Fault_vector;
		std::vector<std::string> Fault_Desc;
		for (std::map<Key, Val>::iterator iter = fault_map.begin(); iter != fault_map.end(); ++iter)
		{
			Key KH = iter->first;

			Fault_vector.push_back(KH);

			Val VH = iter->second;
			Fault_Desc.push_back(VH); // Index corresponds to fault code
			//ss1 << VH << "_";
		}

		// Test here
		read_publisher = nh.advertise<std_msgs::String>("read", 1000);
		diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("machine/diagnostics", 1000);
		/*sleep(2);
		ros::spinOnce();
		sleep(2);

		int fault_code = bitwise_parse(22);
		diagnostic_msgs::DiagnosticStatus fault_msg;
		fault_msg.level = 2;
		fault_msg.name = "Roboteq: " + name_;
		fault_msg.message = Fault_Desc[fault_code - 1];
		diagnostic_msgs::KeyValue temp;
    temp.key = "Fault Code";
    temp.value = std::to_string(fault_code);
    fault_msg.values.push_back(temp);
		diagnostic_pub.publish(fault_msg);
		sleep(2);
		ros::spinOnce();
		sleep(2);*/


		ser.write(ss0.str());
		ser.write(ss1.str());
		ser.write(ss2.str());
		ser.write(ss3.str());

		ser.flush();
		int count = 0;

		sleep(2);
		ros::Rate loop_rate(5);
		while (ros::ok())
		{

			ros::spinOnce();
			if (ser.available())
			{

				std_msgs::String result;
				result.data = ser.read(ser.available());

				read_publisher.publish(result);
				boost::replace_all(result.data, "\r", "");
				boost::replace_all(result.data, "+", "");

				std::vector<std::string> fields;

				std::vector<std::string> Field9;
				boost::split(fields, result.data, boost::algorithm::is_any_of("D"));

				// if fields is empty (or only "+"), seg fault can occur do to memory violation.
				if (fields.size() < 2) {
					ROS_INFO_STREAM("Error reading data; configuration incorrect");
					ROS_INFO_STREAM(result.data.c_str());
					continue; // return to loop and try again
				}
				std::vector<std::string> fields_H;
				boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));

				if (fields_H[0] == "H")
				{

					for (int i = 0; i < publisherVecH.size(); ++i)
					{

						std::vector<std::string> sub_fields_H;

						boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
						roboteq_motor_controller_driver::channel_values Q1;

						for (int j = 0; j < sub_fields_H.size(); j++)
						{

							try
							{
								Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
							}
							catch (const std::exception &e)
							{
								count++;
								if (count > 10)
								{
									ROS_INFO_STREAM("Garbage data on Serial");
									//std::cerr << e.what() << '\n';
								}
							}
						}

						publisherVecH[i].publish(Q1);

						// Check if fault flag; and send to diagnostics
						if (i == fault_iter) {
							int fault_code = bitwise_parse(Q1.value[0]); // first value is fault code; stored as bit wise
							if ((fault_code > 0) && (fault_code < 9)) { // data sanity check
								ROS_INFO_STREAM("Fault detected;");
								diagnostic_msgs::DiagnosticStatus fault_msg;
								fault_msg.level = 2;
								fault_msg.name = "Roboteq: " + name_;
								fault_msg.message = Fault_Desc[fault_code - 1]; // Index starts at 0; check above prevents accidental seg fault with poor data
								diagnostic_msgs::KeyValue temp;
						    temp.key = "Fault Code";
						    temp.value = std::to_string(fault_code);
						    fault_msg.values.push_back(temp);
								diagnostic_pub.publish(fault_msg);
							}
						}
					}
				}
			}
			loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");

	RoboteqDriver driver;

	ros::waitForShutdown();

	return 0;
}
