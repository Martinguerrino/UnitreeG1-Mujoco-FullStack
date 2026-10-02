"""Apply small, reproducible simulator/Jazzy fixes to the pinned ROS2 port."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
source = root / 'src/laserMapping.cpp'
text = source.read_text()
if 'preprocess.instantaneous_cloud' not in text:
    text = text.replace('if (meas.lidar->points.size() <= 1)',
                        'if (instantaneous_cloud) { lidar_end_time = meas.lidar_beg_time; }\n'
                        '        else if (meas.lidar->points.size() <= 1)')
    marker = 'this->declare_parameter<double>("preprocess.blind", 0.01);'
    assert marker in text
    text = text.replace(marker,
                        'instantaneous_cloud = this->declare_parameter<bool>("preprocess.instantaneous_cloud", false);\n        ' + marker)
    # Publish current, rather than previous, covariance.
    text = text.replace('    pubOdomAftMapped->publish(odomAftMapped);\n', '')
    text = text.replace('    geometry_msgs::msg::TransformStamped trans;',
                        '    pubOdomAftMapped->publish(odomAftMapped);\n    geometry_msgs::msg::TransformStamped trans;')
    source.write_text(text)
text = source.read_text()
if 'bool instantaneous_cloud = false;' not in text:
    text = text.replace('#define INIT_TIME', 'bool instantaneous_cloud = false;\n#define INIT_TIME', 1)
    source.write_text(text)
cmake = root / 'CMakeLists.txt'
text = cmake.read_text()
# Jazzy/PCL require C++17; upstream overrides its own standard several times.
text = text.replace('-std=c++14', '-std=c++17').replace('-std=c++0x', '-std=c++17')
cmake.write_text(text)
imu = root / 'src/IMU_Processing.hpp'
text = imu.read_text()
if 'bool instantaneous_cloud = false;' not in text:
    text = text.replace('  double first_lidar_time;',
                        '  bool instantaneous_cloud = false;\n  double first_lidar_time;')
    text = text.replace('  /*** undistort each lidar point (backward propagation) ***/',
                        '  if (instantaneous_cloud) return;\n  /*** undistort each lidar point (backward propagation) ***/')
    imu.write_text(text)
text = source.read_text()
marker = 'instantaneous_cloud = this->declare_parameter<bool>("preprocess.instantaneous_cloud", false);'
if 'p_imu->instantaneous_cloud = instantaneous_cloud;' not in text:
    assert marker in text
    source.write_text(text.replace(marker, marker + '\n        p_imu->instantaneous_cloud = instantaneous_cloud;'))
