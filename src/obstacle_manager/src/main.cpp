#include <ObstacleManager.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "obstacle_manager");
	SYD::ObstacleManager _ObstacleManager;
	_ObstacleManager.start();

	return 0;
}
