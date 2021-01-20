
#include <ros_tegra_utilities/jpeg_compression_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jpeg_compression_node");

  JpegCompressionNode node;

  while (node.isOkay())
  {
    ros::spinOnce();
    node.spin();
  }
  ros::shutdown();

  return 0;
}
