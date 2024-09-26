#include "core/modules/common/math/vec2d.h"
#include "core/modules/common/math/line_segment2d.h"
#include "core/modules/world_model/landmark_navi/landmark_context.h"

namespace npp {
namespace voronoi {

struct VoronoiWrapperInput {
  std::shared_ptr<LandmarkContext> landmark_context;
};

struct VoronoiWrapperOutput {
  std::vector<planning_math::LineSegment2d> input_segments;
  std::vector<planning_math::LineSegment2d> voronoi_g;
  // bool is_connected_boolean;
};

class VoronoiWrapper {
 public:
  VoronoiWrapper() {}
  bool Process(const VoronoiWrapperInput& input, VoronoiWrapperOutput* output, const maf_landmark::LandmarkNavi &landmark_navi);
 private:
};
}
}
