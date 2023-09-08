#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/pick.h"

#include "args/args.hxx"
#include "imgui.h"

#include "neck_model.hpp"

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

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh = NULL;

// Some algorithm parameters
float param1 = 42.0;

size_t cut_index = 0;

void myCallback() {

  if (ImGui::Button("do work")) {
    nm->sweepline_dist_process();
  }

  if (ImGui::Button("Visualize")){
    std::vector<std::array<double, 3>> fcolors(nm->mesh->nFaces(), {.5,.5,.5});
    if (cut_index >= nm->neck_ratio_faces_list.size()){
      cut_index = 0;
    }
    // auto faceset = nm->neck_ratio_faces_list[cut_index];
    auto faceset = nm->compute_candidate_cut();
    for (Face f : faceset){
      fcolors[f.getIndex()] = {1.0, 0.0, 0.0};
    }
    psMesh->addFaceColorQuantity("fcolor", fcolors);

    cut_index++;
  }

  if (ImGui::Button("Set Source")){
    // Get selection from polyscope
    auto x = polyscope::pick::getSelection();
    polyscope::Structure * strc = x.first;
    auto ind = x.second;
    if (ind < psMesh->nVertices()){
      nm->_source = nm->mesh->vertex(ind);
      std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 1.0, 0.0});
      vcolors[nm->_source.getIndex()] = {1.0, 0.0, 0.0};
      psMesh->addVertexColorQuantity("vcolor", vcolors);
    }
  }

  // ImGui::SliderFloat("param", &param1, 0., 100.);
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

  NeckModel nmtemp = NeckModel(args::get(inputFilename));
  nm = std::unique_ptr<NeckModel>(std::move(&nmtemp));

  psMesh = polyscope::registerSurfaceMesh(
      polyscope::guessNiceNameFromPath(args::get(inputFilename)),
      nm->geometry->inputVertexPositions, nm->mesh->getFaceVertexList());
  auto perms = polyscopePermutations(*(nm->mesh));
  psMesh->setAllPermutations(perms);

  std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 1.0, 0.0});
  vcolors[nm->_source.getIndex()] = {1.0, 0.0, 0.0};
  psMesh->addVertexColorQuantity("vcolor", vcolors)->setEnabled(true);

  polyscope::show();

  return EXIT_SUCCESS;
}
