// Decode RTAB-Map's image/depth compression through its official codec.
#include <rtabmap/core/Compression.h>
#include <opencv2/imgproc.hpp>
#include <cstring>
#include <vector>

extern "C" int decode_image(const unsigned char *bytes, size_t size, int depth,
                            void *output, size_t capacity, int *width, int *height) {
  try {
    cv::Mat image = rtabmap::uncompressImage(std::vector<unsigned char>(bytes, bytes + size));
    if (image.empty()) return -1;
    cv::Mat decoded;
    if (depth) {
      if (image.type() != CV_16UC1 && image.type() != CV_32FC1) return -2;
      image.convertTo(decoded, CV_32FC1, image.type() == CV_16UC1 ? 0.001 : 1.0);
    } else if (image.type() == CV_8UC3) {
      cv::cvtColor(image, decoded, cv::COLOR_BGR2RGB);
    } else if (image.type() == CV_8UC1) {
      cv::cvtColor(image, decoded, cv::COLOR_GRAY2RGB);
    } else return -2;
    *width = decoded.cols;
    *height = decoded.rows;
    size_t required = decoded.total() * decoded.elemSize();
    if (required > capacity) return -3;
    std::memcpy(output, decoded.data, required);
    return 0;
  } catch (...) { return -4; }
}
