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

// All RTAB-Map 3-D scan formats (5..12) store XYZ as their first three floats.
extern "C" int decode_scan(const unsigned char *bytes, size_t size, int format,
                           float *output, size_t capacity, size_t *count) {
  try {
    if (format < 5 || format > 12) return -2;
    if (size < 3 * sizeof(int)) return -1;
    int header[3];
    std::memcpy(header, bytes + size - sizeof(header), sizeof(header));
    if (header[0] <= 0 || header[1] <= 0 || (header[2] & 7) != CV_32F ||
        static_cast<size_t>(header[0]) * header[1] > capacity / 3) return -3;
    cv::Mat scan = rtabmap::uncompressData(std::vector<unsigned char>(bytes, bytes + size));
    if (scan.empty() || scan.depth() != CV_32F || scan.channels() < 3) return -1;
    *count = scan.total();
    if (*count * 3 > capacity) return -3;
    const int channels = scan.channels();
    size_t index = 0;
    for (int row = 0; row < scan.rows; ++row) {
      const float *data = scan.ptr<float>(row);
      for (int col = 0; col < scan.cols; ++col, ++index)
        std::memcpy(output + index * 3, data + col * channels, 3 * sizeof(float));
    }
    return 0;
  } catch (...) { return -4; }
}
