#include "path_planning.h"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PathPlanning>());
	rclcpp::shutdown();
	return 0;
}