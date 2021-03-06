
#include <ros_tegra_utilities/jpeg_compression_node.h>

JpegCompressionNode::JpegCompressionNode()
 : fps_(15),
   spin_rate_(45.0)
{
  readParameters();

  jpeg_compressor_.reset(new hw_acceleration::JpegCompressor(quality_));

  sub_image_ = nh_.subscribe(topic_image_in_, 10, &JpegCompressionNode::subImageRaw, this, ros::TransportHints());

  pub_image_compressed_ = nh_.advertise<sensor_msgs::CompressedImage>(topic_image_out_, 10);
}

void JpegCompressionNode::readParameters()
{
  ros::NodeHandle param_nh("~");

  param_nh.param("fps", fps_, 15.0);
  param_nh.param("quality", quality_, 100);
  param_nh.param("topic_in", topic_image_in_, std::string("/camera/image_raw"));
  param_nh.param("topic_out", topic_image_out_, std::string("/tegra_utils/image_compressed"));

  spin_rate_ = fps_ * 3;
}

void JpegCompressionNode::spin() const
{
  ros::Rate rate(spin_rate_);
  rate.sleep();
}

bool JpegCompressionNode::isOkay() const
{
  return nh_.ok();
}

void JpegCompressionNode::subImageRaw(const sensor_msgs::ImageConstPtr &msg_in) const
{
  long t = getCurrentTimeMilliseconds();

  cv_bridge::CvImagePtr img_ptr;
  try
  {
    img_ptr = cv_bridge::toCvCopy(msg_in);
  }
  catch(...)
  {
    throw(cv_bridge::Exception("Error converting compressed image!"));
  }

  cv::Mat mat = img_ptr->image;
  
  std::vector<unsigned char> data = jpeg_compressor_->compress(mat);

  sensor_msgs::CompressedImage msg_out;
  msg_out.header = msg_in->header;
  msg_out.data = data;
  msg_out.format = "jpeg";

  pub_image_compressed_.publish(msg_out);
}
