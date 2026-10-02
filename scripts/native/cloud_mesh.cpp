// Local surface reconstruction of the optimized RGB-D map using PCL GP3.
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/io.h>
#include <fstream>
#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
  if (argc != 3) { std::cerr << "Usage: cloud_mesh input.pcd output.ply\n"; return 2; }
  using Point = pcl::PointXYZRGB;
  auto input = pcl::make_shared<pcl::PointCloud<Point>>();
  if (pcl::io::loadPCDFile<Point>(argv[1], *input) < 0) return 1;
  input->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *input, indices);
  auto cloud = pcl::make_shared<pcl::PointCloud<Point>>();
  float leaf = 0.08f;
  do {
    pcl::VoxelGrid<Point> filter;
    filter.setInputCloud(input);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*cloud);
    leaf *= 1.4f;
  } while (cloud->size() > 50000);
  pcl::PolygonMesh mesh;
  if (cloud->size() >= 3) {
    auto normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimation<Point, pcl::Normal> estimator;
    estimator.setInputCloud(cloud);
    estimator.setKSearch(std::min<int>(20, cloud->size()));
    estimator.compute(*normals);
    auto oriented = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    pcl::concatenateFields(*cloud, *normals, *oriented);
    oriented->is_dense = false;
    pcl::removeNaNNormalsFromPointCloud(*oriented, *oriented, indices);
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> reconstruction;
    reconstruction.setInputCloud(oriented);
    reconstruction.setSearchRadius(0.25);
    reconstruction.setMu(2.5);
    reconstruction.setMaximumNearestNeighbors(50);
    reconstruction.setMaximumSurfaceAngle(std::acos(-1.0) / 4);
    reconstruction.setMinimumAngle(std::acos(-1.0) / 18);
    reconstruction.setMaximumAngle(2 * std::acos(-1.0) / 3);
    reconstruction.setNormalConsistency(false);
    reconstruction.reconstruct(mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  }
  std::vector<pcl::Vertices> faces;
  for (const auto& face : mesh.polygons) {
    if (face.vertices.size() != 3) continue;
    const auto a = (*cloud)[face.vertices[0]].getVector3fMap();
    const auto b = (*cloud)[face.vertices[1]].getVector3fMap();
    const auto c = (*cloud)[face.vertices[2]].getVector3fMap();
    if ((a-b).norm() > 0.25001 || (a-c).norm() > 0.25001 || (b-c).norm() > 0.25001
        || (b-a).cross(c-a).norm() < 1e-8) continue;
    faces.push_back(face);
    if (faces.size() >= 100000) break;
  }
  std::ofstream output(argv[2]);
  output << "ply\nformat ascii 1.0\nelement vertex " << cloud->size()
         << "\nproperty float x\nproperty float y\nproperty float z"
         << "\nproperty uchar red\nproperty uchar green\nproperty uchar blue"
         << "\nelement face " << faces.size()
         << "\nproperty list uchar int vertex_indices\nend_header\n";
  output.precision(9);
  for (const auto& p : *cloud)
    output << p.x << ' ' << p.y << ' ' << p.z << ' '
           << unsigned(p.r) << ' ' << unsigned(p.g) << ' ' << unsigned(p.b) << '\n';
  for (const auto& f : faces)
    output << "3 " << f.vertices[0] << ' ' << f.vertices[1] << ' ' << f.vertices[2] << '\n';
  output.close();
  if (!output) return 1;
  std::cout << cloud->size() << " vertices, " << faces.size() << " triangles\n";
  return 0;
}
