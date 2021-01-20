
#include <ros_tegra_utilities/tegra_utilities.h>

using namespace hw_acceleration;

JpegCompressor::JpegCompressor(int quality)
 : quality_(quality)
{
  nv_jpeg_encoder_.reset(NvJPEGEncoder::createJPEGEncoder("jpenenc"));
  if(!nv_jpeg_encoder_)
    throw(std::runtime_error("jpegenc error"));
}

std::vector<unsigned char> JpegCompressor::compress(const cv::Mat &mat)
{
  if (nv_buffer_ == nullptr)
  {
    nv_buffer_.reset(new NvBuffer(V4L2_PIX_FMT_YUV420M, mat.cols, mat.rows, 0));
    nv_buffer_->allocateMemory();
  }

  cv::Mat mat_yuv;
  cv::cvtColor(mat, mat_yuv, cv::ColorConversionCodes::COLOR_RGB2YUV_I420);

  auto ret = createNvidiaBuffer((const char*)mat_yuv.data, mat_yuv.total()*mat_yuv.elemSize(), *nv_buffer_);
  if(ret < 0)
    throw(std::runtime_error("Creating nvidia buffer failed.")); 

  unsigned long length = mat.cols * mat.rows * 3 / 2;
  unsigned char* buffer = new unsigned char[length];

  ret = nv_jpeg_encoder_->encodeFromBuffer(*nv_buffer_, JCS_YCbCr, &buffer, length, quality_);
  if(ret < 0)
    throw(std::runtime_error("Encoding from buffer failed!"));

  std::vector<unsigned char> data(length);
  for (unsigned long i = 0; i < length; ++i)
    data[i] = buffer[i];

  delete buffer;

  return data;
}

int JpegCompressor::createNvidiaBuffer(const char* buffer_raw, unsigned buffer_raw_length, NvBuffer& buffer_nv)
{
  uint32_t i, j;
  char *data;

  for (i = 0; i < buffer_nv.n_planes; i++)
  {
    NvBuffer::NvBufferPlane &plane = buffer_nv.planes[i];
    std::streamsize bytes_to_read = plane.fmt.bytesperpixel * plane.fmt.width;
    data = (char *) plane.data;
    plane.bytesused = 0;
    for (j = 0; j < plane.fmt.height; j++)
    {
      unsigned num_read = std::min((unsigned) bytes_to_read, (unsigned) buffer_raw_length);

      memcpy(data, buffer_raw, num_read);

      if (num_read < bytes_to_read)
      {
        return -1;
      }

      buffer_raw += num_read;
      buffer_raw_length -= num_read;

      data += plane.fmt.stride;
    }
    plane.bytesused = plane.fmt.stride * plane.fmt.height;
  }

  return 0;
}

long getCurrentTimeMilliseconds()
{
  using namespace std::chrono;
  milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  return ms.count();
}
