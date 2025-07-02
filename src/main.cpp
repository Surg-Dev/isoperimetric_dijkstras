#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

#include "args/args.hxx"
#include "imgui.h"

#include "neck_model.hpp"
#include "utils.hpp"
#include "benchmarker.hpp"

#include <set>
#include <queue>
#include <algorithm>
#include <memory>


using namespace geometrycentral;
using namespace geometrycentral::surface;

// == Geometry-central data
// std::unique_ptr<ManifoldSurfaceMesh> mesh;
// std::unique_ptr<VertexPositionGeometry> geometry;
std::unique_ptr<NeckModel> nm;
std::vector<Halfedge> best_cycle;
std::set<Face> best_cut;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh = NULL;

// Some algorithm parameters
int param1 = 10;
int cutidxs = 0;

int cut_index = 0;

int cycles_made = 0;
int cycle_sel = 0;

float vec[3] = {0., 0., 0.};

void set_look(float a, float b, float c){
  polyscope::view::lookAt(glm::vec3{a, b, c}, glm::vec3{0., 0., 0.});
}

void myCallback() {
    if (ImGui::Button("Final Stretch")){
      finalStretch(nm);
    }

    if (ImGui::Button("Find Leaders")){
      findLeaders(nm);
    }

    if (ImGui::Button("Salient Line")){
      salientLine(nm);
    }

    ImGui::SliderFloat3("Look Vector", vec, -3, 3);

    if (ImGui::Button("Apply LookVector")){
      set_look(vec[0], vec[1], vec[2]);
    }

    ImGui::SliderInt("Cycle Select", &cycle_sel, 0, (nm->salient_cyles_output).size());

    if (ImGui::Button("See Cycle")) {
      // std::cout << cycle_sel << std::endl;

      
      auto test_cycle = nm->salient_cyles_output[cycle_sel];

      // for (auto he : test_cycle) {
      //   std::cout << "(" << he.tipVertex() << " -> " << he.tailVertex() << ") -> ";
      // }
      // std::cout << std::endl;
      std::vector<std::array<double, 3>> cyclefaces(nm->mesh->nFaces(), {0.0,0.0,0.0});

      for (auto he : test_cycle) {
        Face f = he.face();
        cyclefaces[f.getIndex()] = {0.0, 1.0, 0.0};
      }

      // for (auto face : Y.second.adjacentFaces()) {
      //   cyclefaces[face.getIndex()] = {1.0, 0.0, 0.0};
      // }

      // for (auto face : Z.second.adjacentFaces()) {
      //   cyclefaces[face.getIndex()] = {0.0, 0.0, 1.0};
      // }
      auto surf = polyscope::getSurfaceMesh("human_tri");
      surf->addFaceColorQuantity("cyclefaces", cyclefaces);
      std::vector<glm::vec3> output_ve;
      std::vector<std::array<size_t, 2>>   output_ed;
      int base_count = 0;
      for (size_t j = 0 ; j < test_cycle.size(); j++) {
        Halfedge he = test_cycle[j];
        Vector3 vertdat = nm->geometry->vertexPositions[he.tailVertex()];
        output_ve.push_back({vertdat.x, vertdat.y, vertdat.z});
        // output_ve.push_back(nm->geometry .tailVertex().getIndex);
        // std::cout << j << ", " << (j+1) % 37 << std::endl;
        output_ed.push_back({base_count + j, base_count +((j+1) % (test_cycle.size()))});
        // ecolors[he.edge().getIndex()] = {1.0, 0.0, 0.0};
      }
      base_count += test_cycle.size();
      auto curve2 = polyscope::registerCurveNetwork("cyclecurve", output_ve, output_ed);
      curve2->setColor({1.0,0.0,0.0});
    }

}

int main(int argc, char **argv) {

  // Configure the argument parser
  args::ArgumentParser parser("geometry-central & Polyscope example project");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help &h) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError &e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename) {
    std::cerr << "Please specify a mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  // Initialize polyscope
  polyscope::init();

  // Set the callback function
  polyscope::state::userCallback = myCallback;

  //
  NeckModel nmtemp = NeckModel(args::get(inputFilename));
  nm = std::unique_ptr<NeckModel>(std::move(&nmtemp));
  //TEMP
  nm->_source = nm->mesh->vertex(17815);
  nm->_anti_source = nm->mesh->vertex(5687);
  // toe: 5687
  // finger: 21497
  // head: 17815
  // nm->_source = nm->mesh->vertex(4788);

  //teapot tip: 2696
  psMesh = polyscope::registerSurfaceMesh(
      polyscope::guessNiceNameFromPath(args::get(inputFilename)),
      nm->geometry->inputVertexPositions, nm->mesh->getFaceVertexList());
  auto perms = polyscopePermutations(*(nm->mesh));
  psMesh->setAllPermutations(perms);

  std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 1.0, 0.0});
  vcolors[nm->_source.getIndex()] = {1.0, 0.0, 0.0};
  psMesh->addVertexColorQuantity("vcolor", vcolors)->setEnabled(false);

  // Register Overlay curve network
  auto nodes = nm->geometry->inputVertexPositions;
  // Create edge list of vector of array of size 2 of size_t values from nodes indices
  std::vector<std::array<size_t, 2>> edges;
  for (Edge e : nm->mesh->edges()){
    edges.push_back({e.halfedge().vertex().getIndex(), e.halfedge().twin().vertex().getIndex()});
  }
  auto curve = polyscope::registerCurveNetwork("curve", nodes, edges);
  curve->setRadius(0.0002);
  curve->setEnabled(false);

  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  finalStretch(nm);

  // polyscope::CameraIntrinsics camInt = polyscope::CameraIntrinsics::fromFoVDegHorizontalAndVertical(80., 60.);

  // for (double angle; angle < 2.0*PI; angle += ((2.0*PI)/8)) {
  //   double _X = cos(angle)*3;
  //   double _Z = sin(angle)*3;
  //   polyscope::view::lookAt(glm::vec3{_X, 1.5, _Z}, glm::vec3{0., 0., 0.});
  //   polyscope::screenshot();
  // }

  //   for (double angle; angle < 2.0*PI; angle += ((2.0*PI)/8)) {
  //   double _X = cos(angle)*3;
  //   double _Z = sin(angle)*3;
  //   polyscope::view::lookAt(glm::vec3{_X, -1.5, _Z}, glm::vec3{0., 0., 0.});
  //   polyscope::screenshot();
  // }

  polyscope::show();
  
  nm.release();
  nm = NULL;
  return EXIT_SUCCESS;
}
