#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
static constexpr int SUCCESS = 0;
static constexpr int ERROR_IN_COMMAND_LINE = 1;
static constexpr int ERROR_UNHANDLED_EXCEPTION = 2;

typedef struct POINT_3D_XYZ_RGB {
  double x; // mm world coordinate x
  double y; // mm world coordinate y
  double z; // mm world coordinate z
  int r;
  int g;
  int b;

} POINT_XYZRGB;

using POINT_XYZ = struct POINT_3D_XYZ {
  double x;
  double y;
  double z;
};

// struct lin
//{
//    double a[6];
//};

void load_xyzrgb_point_cloud(const std::string &src_txt,
                             const std::string &output_name) {
  /////加载txt数据
  int number_Txt;
  FILE *fp_txt;
  POINT_XYZRGB TxtPoint;
  std::vector<POINT_XYZRGB> m_vTxtPoints;

  fp_txt = fopen(src_txt.c_str(), "r");

  if (fp_txt) {
    while (fscanf(fp_txt, "%lf %lf %lf %d %d %d", &TxtPoint.x, &TxtPoint.y,
                  &TxtPoint.z, &TxtPoint.r, &TxtPoint.g, &TxtPoint.b) != EOF) {
      m_vTxtPoints.push_back(TxtPoint);
    }
  } else
    std::cerr << "txt数据加载失败！" << std::endl;
  number_Txt = m_vTxtPoints.size();
  pcl::PointCloud<pcl::PointXYZRGB> cloud;

  // Fill in the cloud data
  cloud.width = number_Txt;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = m_vTxtPoints[i].x;
    cloud.points[i].y = m_vTxtPoints[i].y;
    cloud.points[i].z = m_vTxtPoints[i].z;
    cloud.points[i].r = m_vTxtPoints[i].r;
    cloud.points[i].g = m_vTxtPoints[i].g;
    cloud.points[i].b = m_vTxtPoints[i].b;
  }
  pcl::io::savePCDFileASCII(output_name.c_str(), cloud);
  std::cerr << "Saved " << cloud.points.size() << " data points to"
            << output_name << std::endl;
}

void load_xyz_point_cloud(const std::string &src_txt,
                          const std::string &output_name) {
  /////加载txt数据
  int number_Txt;
  FILE *fp_txt;
  POINT_XYZ TxtPoint;
  std::vector<POINT_XYZ> m_vTxtPoints;

  fp_txt = fopen(src_txt.c_str(), "r");

  if (fp_txt) {
    while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y,
                  &TxtPoint.z) != EOF) {
      m_vTxtPoints.push_back(TxtPoint);
    }
  } else
    std::cerr << "txt数据加载失败！" << std::endl;
  number_Txt = m_vTxtPoints.size();
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = number_Txt;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = m_vTxtPoints[i].x;
    cloud.points[i].y = m_vTxtPoints[i].y;
    cloud.points[i].z = m_vTxtPoints[i].z;
  }
  pcl::io::savePCDFileASCII(output_name.c_str(), cloud);
  std::cerr << "Saved " << cloud.points.size() << " data points to "
            << output_name << std::endl;
}

int main(int argc, char **argv) {

  std::string src_file;
  std::string target_file = "output.pcd";
  bool with_rgb = false;
  try {
    /** Define and parse the program options
     */
    namespace po = boost::program_options;
    po::options_description desc("Options");
    // clang-format off
    desc.add_options()
    ("help,h", "Print help messages")
    ("source,s", po::value(&src_file)->required(), "src_txt, include XYZ or XYZ_RGB, splitted by space or tab")
    ("target,t", po::value(&target_file),"target pcd file,default is output.pcd")
    ("rgb,r", po::bool_switch(&with_rgb),"whether point with rgb,default is false");

    // clang-format on
    po::variables_map vm;
    try {
      po::store(po::parse_command_line(argc, argv, desc),
                vm); // can throw

      /** --help option
       */
      if (vm.count("help") || argc == 1) {
        std::cout << "Basic Command Line Parameter App" << std::endl
                  << desc << std::endl;
        return SUCCESS;
      }

      po::notify(vm); // throws on error, so do after help in case
                      // there are any problems
    } catch (po::error &e) {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return ERROR_IN_COMMAND_LINE;
    }
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception reached the top of main: " << e.what()
              << ", application will now exit" << std::endl;
    return ERROR_UNHANDLED_EXCEPTION;
  }

  if (with_rgb) {
    load_xyzrgb_point_cloud(src_file, target_file);
  } else {
    load_xyz_point_cloud(src_file, target_file);
  }

  return 0;
}
