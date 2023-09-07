#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

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

// Example computation function -- this one computes and registers a scalar
// quantity
void doWork() {
  polyscope::warning("Computing Gaussian curvature.\nalso, parameter value = " +
                     std::to_string(param1));

  nm->geometry->requireVertexGaussianCurvatures();
  psMesh->addVertexScalarQuantity("curvature",
                                  nm->geometry->vertexGaussianCurvatures,
                                  polyscope::DataType::SYMMETRIC);
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {

  if (ImGui::Button("do work")) {
    doWork();
  }

  ImGui::SliderFloat("param", &param1, 0., 100.);
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

  // Load mesh
  // std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));
  NeckModel nmtemp = NeckModel(args::get(inputFilename));
  nm = std::unique_ptr<NeckModel>(std::move(&nmtemp));

  // Register the mesh with polyscope
  // psMesh = polyscope::registerSurfaceMesh(
  //     polyscope::guessNiceNameFromPath(args::get(inputFilename)),
  //     geometry->inputVertexPositions, mesh->getFaceVertexList(),
  //     polyscopePermutations(*mesh));
  psMesh = polyscope::registerSurfaceMesh(
      polyscope::guessNiceNameFromPath(args::get(inputFilename)),
      nm->geometry->inputVertexPositions, nm->mesh->getFaceVertexList());
  auto perms = polyscopePermutations(*(nm->mesh));
  psMesh->setAllPermutations(perms);
  // // Set vertex tangent spaces
  // nm->geometry->requireVertexTangentBasis();
  // VertexData<Vector3> vBasisX(*mesh);
  // for (Vertex v : mesh->vertices()) {
  //   vBasisX[v] = geometry->vertexTangentBasis[v][0];
  // }
  // VertexData<Vector3> vBasisY(*mesh);
  // for (Vertex v : mesh->vertices()) {
  //   vBasisY[v] = geometry->vertexTangentBasis[v][1];
  // }
  // // psMesh->setVertexTangentBasisX(vBasisX);

  // auto vField =
  //     geometrycentral::surface::computeSmoothestVertexDirectionField(*geometry);
  // psMesh->addVertexTangentVectorQuantity("VF", vField, vBasisX, vBasisY);


  // Run Dijkstra's



  // Compute Neck Cut Ratio

  // Get independent cycles

  // Optimize cycle




  // Give control to the polyscope gui
  polyscope::show();

  return EXIT_SUCCESS;
}
