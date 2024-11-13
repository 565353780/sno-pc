#include "octTree.h"
#include "pointCloud.h"
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

void writeResult(std::string filePath, std::vector<Point> points,
                 std::vector<double> normal) {
  std::ofstream fout;
  fout.open(filePath, std::ios::out);
  if (!fout.is_open()) {
    if (!fout.is_open()) {
      std::cout << filePath << "File write error" << std::endl;
      return;
    }
  }
  /* ply
  fout << "ply" << std::endl;
  fout << "format ascii 1.0" << std::endl;
  fout << "element vertex " << points.size() << std::endl;
  fout << "property double x" << std::endl;
  fout << "property double y" << std::endl;
  fout << "property double z" << std::endl;
  fout << "property double nx" << std::endl;
  fout << "property double ny" << std::endl;
  fout << "property double nz" << std::endl;
  fout << "end_header" << std::endl;
  */
  for (int i = 0; i < points.size(); i++) {
    fout << points[i].x << " ";
    fout << points[i].y << " ";
    fout << points[i].z << " ";
    fout << normal[3 * i] << " ";
    fout << normal[3 * i + 1] << " ";
    fout << normal[3 * i + 2] << std::endl;
  }
  fout.close();
}

void writeResult(std::string filePath, std::vector<Point> points) {
  std::ofstream fout;
  fout.open(filePath, std::ios::out);
  if (!fout.is_open()) {
    if (!fout.is_open()) {
      std::cout << filePath << "File write error" << std::endl;
      return;
    }
  }

  for (int i = 0; i < points.size(); i++) {
    fout << points[i].x << " ";
    fout << points[i].y << " ";
    fout << points[i].z << std::endl;
  }
  fout.close();
}
void writeOctreeNode(std::string filePath, std::vector<Point> points,
                     OctTree &octree, std::vector<double> &normal) {
  std::ofstream fout;
  fout.open(filePath, std::ios::out);
  if (!fout.is_open()) {
    if (!fout.is_open()) {
      std::cout << filePath << "File write error" << std::endl;
      return;
    }
  }
  std::vector<double> p;
  octree.energy_evaluation_w_print_p(normal, p);
  int N = octree.points.size();
  fout << "ply" << std::endl;
  fout << "format ascii 1.0" << std::endl;
  fout << "element vertex " << points.size() << std::endl;
  fout << "property double x" << std::endl;
  fout << "property double y" << std::endl;
  fout << "property double z" << std::endl;
  fout << "property uint8 red" << std::endl;
  fout << "property uint8 green" << std::endl;
  fout << "property uint8 blue" << std::endl;
  fout << "end_header" << std::endl;
  for (int i = 0; i < points.size(); i++) {
    fout << points[i].x << " ";
    fout << points[i].y << " ";
    fout << points[i].z << " ";
    if (i < N) {
      fout << 255 << " ";
      fout << 0 << " ";
      fout << 0 << std::endl;
    } else {
      fout << (int)((p[i - N]) * 255) << " ";
      fout << (int)((1 - p[i - N]) * 255) << " ";
      fout << (int)((p[i - N]) * 255) << std::endl;
    }
  }

  fout.close();
}

int main(int argc, char *argv[]) {
  const std::string input_xyz_file_path = argv[1];
  const std::string output_folder_path = argv[2];
  const std::string type = ".xyz";

  const std::string input_file_type =
      std::filesystem::path(input_xyz_file_path).extension();
  if (input_file_type != type) {
    std::cerr << "[ERROR][main::main]" << std::endl;
    std::cerr << "\t input file type not valid!" << std::endl;
    std::cerr << "\t input_xyz_file_path: " << input_xyz_file_path << std::endl;
    return -1;
  }

  const std::string input_xyz_filename =
      std::filesystem::path(input_xyz_file_path).filename();

  PointCloud pointcloud(input_xyz_file_path);
  int N = pointcloud.points.size();

  Eigen::VectorXd Nx = Eigen::VectorXd::Constant(N, 0);
  Eigen::VectorXd Ny = Eigen::VectorXd::Constant(N, 0);
  Eigen::VectorXd Nz = Eigen::VectorXd::Constant(N, 0);

  std::vector<double> uv(2 * N);
  std::vector<double> nor(3 * N);
  srand(time(0));
  // initialize
  for (int i = 0; i < N; i++) {
    uv[2 * i] = rand();
    uv[2 * i + 1] = rand();
  }
  for (int i = 0; i < N; i++) {
    nor[3 * i] = sin(uv[2 * i]) * cos(uv[2 * i + 1]);
    nor[3 * i + 1] = sin(uv[2 * i]) * sin(uv[2 * i + 1]);
    nor[3 * i + 2] = cos(uv[2 * i]);
  }

  OctTree octree(&pointcloud);

  std::vector<Point> points;

  if (DEBUG) {
    for (int i = 0; i < pointcloud.points.size(); i++) {
      points.push_back(octree.points[i]);
    }

    for (int i = 0; i < octree.nopNode.size(); i++) {
      points.push_back(octree.nodeArr[octree.nopNode[i]]->center);
    }
    // writeOctreeNode(output_folder_path + fileName + "_grid_p_start" + ".ply",
    // points, octree, uv);
  }

  octree.optimize_LBFS_w_uv(uv);

  for (int i = 0; i < N; i++) {
    nor[3 * i] = sin(uv[2 * i]) * cos(uv[2 * i + 1]);
    nor[3 * i + 1] = sin(uv[2 * i]) * sin(uv[2 * i + 1]);
    nor[3 * i + 2] = cos(uv[2 * i]);
  }
  if (DEBUG) {
    writeOctreeNode(output_folder_path + input_xyz_filename + "_grid_p_end" +
                        ".ply",
                    points, octree, uv);
  }

  writeResult(output_folder_path + input_xyz_filename + "_res" + type,
              pointcloud.points, nor);
  return 0;
}
