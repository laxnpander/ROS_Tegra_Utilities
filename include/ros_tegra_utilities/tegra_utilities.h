
#ifndef SRC_TEGRA_UTILITIES_H
#define SRC_TEGRA_UTILITIES_H

#include <memory>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "NvUtils.h"
#include "NvJpegEncoder.h"
#include "NvVideoConverter.h"

namespace hw_acceleration
{

class JpegCompressor
{
public:
  JpegCompressor(int quality);

  std::vector<unsigned char> compress(const cv::Mat &mat);

private:

  int quality_;

  std::unique_ptr<NvBuffer> nv_buffer_;
  std::unique_ptr<NvJPEGEncoder> nv_jpeg_encoder_;

  int createNvidiaBuffer(const char* buffer_raw, unsigned buffer_raw_length, NvBuffer& buffer_nv);

};

}

long getCurrentTimeMilliseconds();

#endif //SRC_TEGRA_UTILITIES_H
