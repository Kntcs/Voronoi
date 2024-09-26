#include "core/modules/world_model/landmark_navi/voronoi_wrapper.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_traits_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <cstddef>
#include <utility>  // for std::pair
#include <vector>
#include "vec2d.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>


namespace npp {
namespace voronoi {

typedef CGAL::Exact_predicates_inexact_constructions_kernel           CGAL_K;
typedef CGAL_K::Point_2                                               Point_2;
typedef CGAL_K::Segment_2                                             Segment_2;
typedef CGAL::Segment_Delaunay_graph_traits_2<CGAL_K>                 Gt;
typedef CGAL::Segment_Delaunay_graph_2<Gt>                            SDG2;
typedef CGAL::Delaunay_triangulation_2<CGAL_K>                        DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>          AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT,AT,AP>                             VD;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              boost::no_property, boost::property<boost::edge_weight_t, double>> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor                 VertexDescriptor;
typedef boost::graph_traits<Graph>::edge_descriptor                   EdgeDescriptor;



bool is_intersecting_with_polyline(
    const Segment_2& voronoi_edge,
    const std::vector<Segment_2>& polyline_segments) {
  for (const auto& seg : polyline_segments) {
    if (CGAL::do_intersect(voronoi_edge, seg)) {
      return true;
    }
  }
  return false;
}

bool is_intersecting_with_polyline_offset(//offset=0.5//去小杂边
    const Segment_2& voronoi_edge,
    const std::vector<Segment_2>& polyline_segments) {
  for (const auto& seg : polyline_segments) {
    auto seg1 = Segment_2(Point_2(seg.source().x()+.01,seg.source().y()+.01), 
    Point_2(seg.target().x()+.01,seg.target().y()+.01));
    // auto seg2 = Segment_2(Point_2(seg.source().x()-.01,seg.source().y()-.01), 
    // Point_2(seg.target().x()-.01,seg.target().y()-.01));
    // auto seg3 = Segment_2(Point_2(seg.source().x()+.01,seg.source().y()-.01), 
    // Point_2(seg.target().x()-.01,seg.target().y()+.01));
    // auto seg4 = Segment_2(Point_2(seg.source().x()-.01,seg.source().y()+.01), 
    // Point_2(seg.target().x()+.01,seg.target().y()-.01));

    // auto seg5 = Segment_2(Point_2(seg.source().x()+.1,seg.source().y()+.1), 
    // Point_2(seg.target().x()+.1,seg.target().y()+.1));
    // auto seg6 = Segment_2(Point_2(seg.source().x()-.1,seg.source().y()-.1), 
    // Point_2(seg.target().x()-.1,seg.target().y()-.1));
    auto seg7 = Segment_2(Point_2(seg.source().x()+.1,seg.source().y()-.1), 
    Point_2(seg.target().x()-.1,seg.target().y()+.1));
    // auto seg8 = Segment_2(Point_2(seg.source().x()-.1,seg.source().y()+.1), 
    // Point_2(seg.target().x()+.1,seg.target().y()-.1));

    // auto seg9 = Segment_2(Point_2(seg.source().x()+.8,seg.source().y()+.8), 
    // Point_2(seg.target().x()+.8,seg.target().y()+.8));
    auto seg10 = Segment_2(Point_2(seg.source().x()-.8,seg.source().y()-.8), 
    Point_2(seg.target().x()-.8,seg.target().y()-.8));
    // auto seg11 = Segment_2(Point_2(seg.source().x()+.8,seg.source().y()-.8), 
    // Point_2(seg.target().x()-.8,seg.target().y()+.8));
    // auto seg12 = Segment_2(Point_2(seg.source().x()-.8,seg.source().y()+.8), 
    // Point_2(seg.target().x()+.8,seg.target().y()-.8));
    
    if (CGAL::do_intersect(voronoi_edge, seg1)||
    // CGAL::do_intersect(voronoi_edge, seg2)||
    // CGAL::do_intersect(voronoi_edge, seg3)||
    // CGAL::do_intersect(voronoi_edge, seg4)||
    // CGAL::do_intersect(voronoi_edge, seg5)||CGAL::do_intersect(voronoi_edge, seg6)||
    CGAL::do_intersect(voronoi_edge, seg7)||
    // CGAL::do_intersect(voronoi_edge, seg8)||
    // CGAL::do_intersect(voronoi_edge, seg9)||
    CGAL::do_intersect(voronoi_edge, seg10)
    // ||CGAL::do_intersect(voronoi_edge, seg11)||CGAL::do_intersect(voronoi_edge, seg12)
    ) {
      return true;
    }
  }
  return false;
}

bool is_ray(const Segment_2& e){//voronoi_edge
  return (std::fabs(e.source().x()-e.target().x())+std::fabs(e.source().y()-e.target().y())>8);
}

// bool is_near(std::vector<Point_2> polyline, const npp::common::EgoPose &egopose_navi){
//   float ego_x = egopose_navi.position().x();
//   float ego_y = egopose_navi.position().y();
//   for (Point_2 p : polyline){
//     if (std::fabs(p.x()-ego_x)+std::fabs(p.y()-ego_y)<10) return true;
//   }
//   return false;
  
// }
// bool is_vertical(
//     const Segment_2& voronoi_edge,
//     std::vector<double> avg_dir_vec){
//   const double x11=voronoi_edge.source().x();
//   const double y11=voronoi_edge.source().y();
//   const double x12=voronoi_edge.target().x();
//   const double y12=voronoi_edge.target().y();
//   return std::fabs((x12-x11)+avg_dir_vec[1]*(y12-y11))<.05;
// }

// std::vector<double> dir_vector(std::vector<Point_2> polyline){
//   //默认两个点
//   const auto& p1=polyline[0];
//   const auto& p2=polyline.back();
//   const double k = std::fabs(p1.y()-p2.y())/(std::fabs(p1.x()-p2.x())+1e6);//防止除0
//   std::vector<double> vec(1,1);
//   vec.emplace_back(k);
//   return vec;
// }


// 检查联通性的
// bool find_site_point(const VD& vd, const planning_math::Vec2d& p, Point_2& pp){
//   const float epsilon=0.05;
//   for (VD::Vertex_iterator vi = vd.vertices_begin(); vi != vd.vertices_end(); ++vi) {
//         Point_2 vp = vi->point();
//         if (std::fabs(vp.x()-p.x())+std::fabs(vp.y()-p.y())<epsilon){
//           pp=vp;
//           return true;
//         }
// }//no res 保护
//   return false;
// }

// void add_edge_to_graph(const Point_2& source, const Point_2& target, Graph& graph, std::map<Point_2, VertexDescriptor>& points_to_vertices) {
//     auto source_it = points_to_vertices.find(source);
//     if (source_it == points_to_vertices.end()) {
//         source_it = points_to_vertices.insert({source, add_vertex(graph)}).first;
//     }
    
//     auto target_it = points_to_vertices.find(target);
//     if (target_it == points_to_vertices.end()) {
//         target_it = points_to_vertices.insert({target, add_vertex(graph)}).first;
//     }

//     double weight = std::sqrt(CGAL::squared_distance(source, target));
//     add_edge(source_it->second, target_it->second, weight, graph);
// }

// bool is_connected(const VD& vd, const planning_math::Vec2d& p1, const planning_math::Vec2d& p2){//dijsktra
//   //先找对应点，搜索最短路，若最短路长度在曼哈顿距离和欧氏距离之间返回true
//   Point_2 pp1;
//   Point_2 pp2;
//   if (!find_site_point(vd,p1,pp1)||!find_site_point(vd,p2,pp2)){
//     return false;
//   }
//   else{
//     Graph graph;
//     std::map<Point_2, VertexDescriptor> points_to_vertices;
//     for (VD::Edge_iterator ei = vd.edges_begin(); ei != vd.edges_end(); ++ei) {
//             if (ei->is_segment()) {
//                 add_edge_to_graph(ei->source()->point(), ei->target()->point(), graph, points_to_vertices);
//             }
//     }
    
//     std::vector<VertexDescriptor> p(num_vertices(graph)); 
//     std::vector<double> d(num_vertices(graph));
//     VertexDescriptor start = points_to_vertices[Point_2(p1.x(), p1.y())];
//     VertexDescriptor goal = points_to_vertices[Point_2(p2.x(), p2.y())];
//     boost::dijkstra_shortest_paths(graph, start,
//                                    boost::predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, graph)))
//                                    .distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, graph))));
//     if (d[goal] < std::numeric_limits<double>::max()) {
//         return true;
//     } else {
//         return false;
//     }

//   }
// return false;
// }


bool VoronoiWrapper::Process(
  const VoronoiWrapperInput& input, VoronoiWrapperOutput* output,
  const maf_landmark::LandmarkNavi &landmark_navi){
  const auto &lcs = landmark_navi.landmarks.lane_centers;
  const auto& boundaries = input.landmark_context->boundaries();
  std::vector<std::vector<Point_2>> polylines;
  std::vector<Point_2> points;
  std::vector<Segment_2> segments;
  // std::vector<std::vector<double>> dir_vecs;//方向向量
  constexpr double kLengthThreshold = 20.0;  
  constexpr double sampleDelta = 0.5;//

  for(const auto& lc : lcs){
    
  }


  for (const auto& boundary : boundaries) {
    std::vector<Point_2> polyline;
    polyline.reserve(boundary.points_boot().size());
    //Segments
    for (size_t i = 0; i < boundary.points_boot().size(); ++i) {
      const auto& p = boundary.points_boot()[i];
      if (!polyline.empty() && i + 1 < boundary.points_boot().size()) {
        const auto& last = polyline.back();
        const double dist_ =
            std::fabs(p.x - last.x()) + std::fabs(p.y - last.y());
        if (dist_ < kLengthThreshold) {
          continue;
        } 
      }
      polyline.push_back(Point_2(p.x, p.y));
      // //sdg
      // points.push_back(Point_2(p.x+.1, p.y+.1));//防止被归类到seg上
    }
    polylines.push_back(polyline);
    // if (is_near(polyline,egopose_navi)){
    //   const std::vector<double> vec=dir_vector(polyline);
    //   dir_vecs.push_back(vec);
    // }

    //SamplingPoints
    for (size_t i = 1; i < boundary.points_boot().size(); ++i) {
      const auto& p = boundary.points_boot()[i];
      const auto& q = boundary.points_boot()[i-1];
      const double dist_ = std::fabs(p.x - q.x) + std::fabs(p.y - q.y);
      if (dist_ < sampleDelta) {
        continue;
      }
      points.push_back(Point_2(p.x, p.y));
    }
  }

  for (const auto& polyline : polylines) {
    for (size_t i = 1; i < polyline.size(); ++i) {
      auto curr_seg = Segment_2(polyline[i - 1], polyline[i]);
      output->input_segments.emplace_back(
          planning_math::Vec2d{curr_seg.source().x(), curr_seg.source().y()},
          planning_math::Vec2d{curr_seg.target().x(), curr_seg.target().y()});
      segments.push_back(curr_seg);
    }
  }
  // //用sdg加上端点
  // SDG2 sdg;
  // sdg.insert_segments(segments.begin(), segments.end());
  // sdg.insert(points.begin(), points.end());
  // // Extract Voronoi edges
  // for (auto it = sdg.finite_edges_begin(); it != sdg.finite_edges_end(); ++it) {
  //   CGAL::Object o = sdg.primal(it);
  //   CGAL_K::Segment_2 s;
  //   if (CGAL::assign(s, o)
  //   && !is_intersecting_with_polyline(s, segments)
  //   && !is_intersecting_with_polyline_offset(s, segments)
  //   ) {
  //     output->voronoi_g.emplace_back(
  //         planning_math::Vec2d(s.source().x(), s.source().y()),
  //         planning_math::Vec2d(s.target().x(), s.target().y()));
  //   }
  // }
  // const int vec_num=dir_vecs.size();
  // double dir_x_acc=.0;
  // double dir_y_acc=.0;
  // for (const auto& vec : dir_vecs){
  //   dir_x_acc+=vec[0];
  //   dir_y_acc+=vec[1];
  // }
  // std::vector<double> avg_dir_vec;
  // avg_dir_vec.emplace_back(dir_x_acc/(vec_num+1e6));
  // avg_dir_vec.emplace_back(dir_y_acc/(vec_num+1e6));

  //ooaa dt
  DT dt;
  dt.insert(points.begin(), points.end());
  VD vd(dt);    
  for (VD::Edge_iterator ei = vd.edges_begin(); ei != vd.edges_end(); ++ei) {
      if (ei->is_segment()) {
        VD::Face_handle fh1=ei->face();
        VD::Face_handle fh2=ei->twin()->face();
        Point_2 p1 = fh1->dual()->point();
        Point_2 p2 = fh2->dual()->point();
        const double site_dist=
            std::fabs(p1.x() - p2.x()) + std::fabs(p1.y() - p2.y());
        if (site_dist<3.0) continue;

        const Segment_2 e(ei->source()->point(), ei->target()->point());
        if(!is_ray(e) && !is_intersecting_with_polyline(e, segments)
        && !is_intersecting_with_polyline_offset(e, segments)
        // && !is_vertical(e, avg_dir_vec)
        ) {
          output->voronoi_g.emplace_back(
            planning_math::Vec2d(e.source().x(), e.source().y()),
            planning_math::Vec2d(e.target().x(), e.target().y()));
      }
    }
  }

//is connected?






  return true;
}//end of sample process

}
}
