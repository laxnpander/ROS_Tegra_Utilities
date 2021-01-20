
#ifndef SRC_JPEG_COMPRESSION_NODE_H
#define SRC_JPEG_COMPRESSION_NODE_H

#include <memory>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <ros_tegra_utilities/tegra_utilities.h>

class JpegCompressionNode
{
public:
  JpegCompressionNode();
  ~JpegCompressionNode() = default;
  JpegCompressionNode(const JpegCompressionNode&) = default;
  JpegCompressionNode& operator=(const JpegCompressionNode&) = default;

  void spin() const;

  bool isOkay() const;

private:

  int quality_;

  double fps_;
  double spin_rate_;

  // topics
  std::string topic_image_in_;
  std::string topic_image_out_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_image_;
  ros::Publisher pub_image_compressed_;

  std::unique_ptr<hw_acceleration::JpegCompressor> jpeg_compressor_;

  void readParameters();

  void subImageRaw(const sensor_msgs::ImageConstPtr &msg) const;
};

#endif //SRC_JPEG_COMPRESSION_NODE_H
