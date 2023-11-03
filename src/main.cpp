#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"

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
std::vector<Halfedge> best_cycle;
std::set<Face> best_cut;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh = NULL;

// Some algorithm parameters
int param1 = 10;
int cutidxs = 0;

int cut_index = 0;

void myCallback() {

  if (ImGui::Button("do work")) {
    nm->sweepline_dist_process();
    cutidxs = nm->neck_ratio_faces_list.size();
  }

  if (ImGui::Button("Visualize")){
    std::vector<std::array<double, 3>> fcolors(nm->mesh->nFaces(), {.5,.5,.5});
    // auto faceset = nm->neck_ratio_faces_list[cut_index];
    best_cut = nm->compute_candidate_cut();
    for (Face f : best_cut){
      fcolors[f.getIndex()] = {1.0, 0.0, 0.0};
    }
    psMesh->addFaceColorQuantity("fcolor", fcolors);
  }

  if (ImGui::Button("Visualize Selected")){
    std::vector<std::array<double, 3>> fcolors(nm->mesh->nFaces(), {.5,.5,.5});
    best_cut = nm->get_candidate_cut(cut_index);
    for (Face f : best_cut){
      fcolors[f.getIndex()] = {1.0, 0.0, 0.0};
    }
    psMesh->addFaceColorQuantity("fcolor", fcolors);

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

  if (ImGui::Button("Get Dijkstra Cyc from Edge")){
    auto x = polyscope::pick::getSelection();
    polyscope::Structure * strc = x.first;
    
    // auto ind = x.second;
    // std::cout << strc->typeName() << " " << ind << std::endl;
    // std::cout << nm->mesh->nEdges() << std::endl;
    // std::cout << nm->mesh->nVertices() << std::endl;
    // std::cout << psMesh->nEdges() << std::endl;
    // std::cout << psMesh->nVertices() << std::endl;
    // std::cout << psMesh->vertexDataSize << " " << psMesh->edgeDataSize << " " << psMesh->faceDataSize << " " << psMesh->halfedgeDataSize << std::endl;
    auto ind = x.second- (psMesh->vertexDataSize + psMesh->edgeDataSize + psMesh->faceDataSize);
    // if (ind < psMesh->nEdges()){
      auto e = nm->mesh->halfedge(ind).edge();
      // auto e = nm->mesh->edge(ind);
      auto v1 = e.halfedge().vertex();
      auto v2 = e.halfedge().twin().vertex();

      Vertex curr = v1;

      std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
      ecolors[ e.getIndex() ] = {1.0, 0.0, 0.0};
      while(curr != nm->_source){
        auto prev = nm->_prev[curr];

        e = nm->get_edge(prev, curr);

        ecolors[e.getIndex()] = {1.0, 0.0, 0.0};
        curr = nm->_prev[curr];
      }
      curr = v2;
      while(curr != nm->_source){
        auto prev = nm->_prev[curr];

        e = nm->get_edge(prev, curr);

        ecolors[e.getIndex()] = {1.0, 0.0, 0.0};
        curr = nm->_prev[curr];
      }
      auto curve = polyscope::getCurveNetwork("curve");
      curve->addEdgeColorQuantity("cycle", ecolors);
    // }
  }

  if (ImGui::Checkbox("Restricted Search", &nm->_restricted_search)){}

  if (ImGui::Button("Show Edge Cycle")){
    std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
    // auto faceSet = nm->compute_candidate_cut();
    auto edgeSet = nm->determine_cycle_edgeset(best_cut);
    auto cycle = nm->determine_single_cycle(edgeSet);
    best_cycle = nm->orient_cycle(cycle);
    std::cout << std::endl;
    for (Halfedge e : best_cycle){
      ecolors[e.edge().getIndex()] = {1.0, 0.0, 0.0};
    }
    auto curve = polyscope::getCurveNetwork("curve");
    curve->addEdgeColorQuantity("ecolor", ecolors);
    // psMesh->addEdgeColorQuantity("ecolor", ecolors);
  }

  if (ImGui::Button("Improve")){
    std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
    for (size_t i = 0; i < param1; i++){
        best_cycle = nm->optimize_oriented_cycle_single(best_cycle);
    }
    // std::cout << std::endl;
    for (Halfedge e : best_cycle){
      ecolors[e.edge().getIndex()] = {1.0, 0.0, 0.0};
    }
    auto curve = polyscope::getCurveNetwork("curve");
    curve->addEdgeColorQuantity("ecolor", ecolors);
    // psMesh->addEdgeColorQuantity("ecolor", ecolors);
  }
  ImGui::SliderInt("Improve Iters", &param1, 1, 40);
  ImGui::SliderInt("Cut Index", &cut_index, 0, cutidxs);

  if (ImGui::Button("Generate Middle Edges")){
    std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
    for (Edge e : nm->mesh->edges()){
      if (nm->_middle[e]){
        ecolors[e.getIndex()] = {1.0, 0.0, 1.0};
      }
    }
    auto curve = polyscope::getCurveNetwork("curve");
    curve->addEdgeColorQuantity("middle", ecolors);
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
  // nm->_source = nm->mesh->vertex(17815);
  nm->_source = nm->mesh->vertex(4788);

  psMesh = polyscope::registerSurfaceMesh(
      polyscope::guessNiceNameFromPath(args::get(inputFilename)),
      nm->geometry->inputVertexPositions, nm->mesh->getFaceVertexList());
  auto perms = polyscopePermutations(*(nm->mesh));
  psMesh->setAllPermutations(perms);

  std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 1.0, 0.0});
  vcolors[nm->_source.getIndex()] = {1.0, 0.0, 0.0};
  psMesh->addVertexColorQuantity("vcolor", vcolors)->setEnabled(true);

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
  polyscope::show();

  return EXIT_SUCCESS;
}
