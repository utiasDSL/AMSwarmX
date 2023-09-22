#include "txt_reader.hpp"
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>

#include <iostream>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>


int main(int argc, char **argv) {
  if (argc != 2) {
    printf(ANSI_COLOR_RED "Input txt file required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read obstacles
  vec_Vec3f obs;
  if(!read_obs<3>(argv[1], obs)) {
    printf(ANSI_COLOR_RED "Cannot find input file [%s]!\n" ANSI_COLOR_RESET,
           argv[1]);
    return -1;
  }

  // Set map size
  const Vec3f origin(-2, -2, -2);
  const Vec3f range(5, 5, 5);

  // Path to dilate
  vec_Vec3f path;
  path.push_back(Vec3f(0.0, 0.0, 0.0));
  path.push_back(Vec3f(0.5, 0.0+0.0000000001, 0.0+0.0000000001));
  

  // Initialize SeedDecomp2D
  EllipsoidDecomp3D decomp(origin, range);
  decomp.set_obs(obs);
  decomp.set_local_bbox(Vec3f(4, 4, 4));
  decomp.dilate(path, 0);

    std::ofstream outdata("poly_py.txt");
    for(const auto& poly: decomp.get_polyhedrons()) {
        const auto vertices = cal_vertices(poly);
        std::string ss("POLYGON((");
        for (size_t i = 0; i < vertices.size(); i++) {
            for(size_t j = 0; j < vertices[i].size(); j++){
                // std::cout<<vertices[i][j](0) << " " << vertices[i][j](1) << " " << vertices[i][j](2) << "\n";
                outdata <<vertices[i][j](0) << " " << vertices[i][j](1) << " " << vertices[i][j](2) << "\n";
            }
        }

        for (const auto &p : poly.hyperplanes()) 
          std :: cout << p.p_(0) << " " << p.p_(1) << " " << p.p_(2) << " \n";
    }
    outdata.close();
    std::ofstream outdata2("obs_py.txt");
    for(int i = 0; i < obs.size(); i++){
        outdata2 << obs[i][0] << " " << obs[i][1] << " " << obs[i][2] << "\n";
    }
    outdata2.close();

  // Draw polygon
//   {
//     for(const auto& poly: decomp.get_polyhedrons()) {
//       const auto vertices = cal_vertices(poly);
//       std::string ss("POLYGON((");
//       for (size_t i = 0; i < vertices.size(); i++) {
//         ss += std::to_string(vertices[i](0)) + " " +
//           std::to_string(vertices[i](1));
//         if(i == vertices.size() - 1)
//           ss += "))";
//         else
//           ss += ",";
//         std::cout<<vertices[i](0) << " " << vertices[i](1) << "\n";
//       }

//       boost::geometry::model::polygon<point_2d> p;
//       boost::geometry::read_wkt(ss, p);
//       mapper.add(p);
//       mapper.map(p, "fill-opacity:0.2;fill:rgb(51,51,153);stroke:rgb(51,51,153);stroke-width:2");
//     }
//   }

  return 0;
}
