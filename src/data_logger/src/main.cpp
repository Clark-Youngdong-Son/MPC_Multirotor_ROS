#include <message_handler.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_logger");

	hss::MessageHandler mh;

	ros::spin();

	return 0;
}
