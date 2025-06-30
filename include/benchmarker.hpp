
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

#include "neck_model.hpp"
#include "utils.hpp"

#include <set>
#include <queue>
#include <algorithm>
#include <memory>
#include <chrono>

using namespace geometrycentral;
using namespace geometrycentral::surface;

void finalStretch(std::unique_ptr<NeckModel> & nm);
void findLeaders(std::unique_ptr<NeckModel> & nm);
void salientLine(std::unique_ptr<NeckModel> & nm);