#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "geometrycentral/surface/direction_fields.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/pick.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"

#include "glm/gtx/string_cast.hpp"

#include "args/args.hxx"
#include "imgui.h"

#include "neck_model.hpp"
#include "utils.hpp"
#include "benchmarker.hpp"

#include <set>
#include <queue>
#include <thread>
#include <algorithm>
#include <memory>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// void orbitCam(float angle, glm::vec3 axis) {
//   using namespace glm;
 

// }

// == Geometry-central data
// std::unique_ptr<ManifoldSurfaceMesh> mesh;
// std::unique_ptr<VertexPositionGeometry> geometry;
std::unique_ptr<NeckModel> nm;
std::vector<Halfedge> best_cycle;
std::set<Face> best_cut;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh = NULL;

std::string mesh_name;

// Some algorithm parameters
int param1 = 10;
int cutidxs = 0;

int cut_index = 0;

int cycles_made = 0;
int cycle_sel = 0;
int bone_sel = 0;

float gpos[3] = {0., 0., 0.};
float gla[3] = {0., 0., 0.};

void set_look()
{
  polyscope::view::lookAt(glm::vec3{gpos[0], gpos[1], gpos[2]}, glm::vec3{gla[0], gla[1], gla[2]});
}

void myCallback()
{
  if (ImGui::Button("Final Stretch"))
  {
    finalStretch(nm);
  }

  if (ImGui::Button("Find Leaders"))
  {
    findLeaders(nm);
  }

  if (ImGui::Button("Salient Line"))
  {
    salientLine(nm);
  }

  ImGui::InputFloat3("Look Pos", gpos);
  ImGui::InputFloat3("Look Target", gla);

  if (ImGui::Button("Apply LookVector"))
  {
    set_look();
  }

  if (ImGui::Button("Print Home LA")) {
    polyscope::CameraParameters cparam = polyscope::view::getCameraParametersForCurrentView();
    glm::vec3 cpos = cparam.getPosition();
    glm::vec3 cldir  = cparam.getLookDir();
    glm::vec3 udir = cparam.getUpDir();
    std::cout << cparam.getFoVVerticalDegrees() << std::endl;
    std::cout << glm::to_string(cpos) << std::endl;
    std::cout << glm::to_string(cldir) << std::endl;
    std::cout << glm::to_string(cparam.getUpDir()) <<  std::endl;
    std::cout << glm::to_string(cparam.getRightDir()) <<  std::endl;

  }

  ImGui::SliderInt("Bone Select", &bone_sel, 0, (nm->skeleton_cycles_output).size()-1);
  if (ImGui::Button("Update Bone Select")) {
    nm->salient_cycles_output = nm->skeleton_cycles_output[bone_sel];
  }
  ImGui::SliderInt("Cycle Select", &cycle_sel, 0, (nm->salient_cycles_output).size()-1);

  if (ImGui::Button("Visualize Area")) {
    auto selected_cycle = nm->salient_cycles_output[cycle_sel];

    FaceData<bool> visited_f(*(nm->mesh)); // Visited Set of faces
    std::vector<std::array<double, 3>> cyclefaces(nm->mesh->nFaces(), {0.0, 1.0, 0.0});
    std::queue<Face> bfs_q;

    std::unordered_set<Face> banned_faces; // Banned faces from enquing in one iteration
      double area_sum = 0.0;
      for (auto he : selected_cycle)
      {
        // enqueue all the faces induced by one side of the cycle
        if (visited_f[he.face()] == false)
        {
          bfs_q.push(he.face());
          visited_f[he.face()] = true;
          cyclefaces[he.face().getIndex()] = {0.0,0.0,1.0};
        }
        // ban all the faces induced by the other side (this should prevent all crossings automatically)
        banned_faces.insert(he.twin().face());
      }

    while (!bfs_q.empty())
    {
      Face f = bfs_q.front();
      bfs_q.pop();
      area_sum += nm->geometry->faceAreas[f];
      for (Face g : f.adjacentFaces())
      {
        if (visited_f[g] == false && banned_faces.find(g) == banned_faces.end())
        {
          bfs_q.push(g);
          visited_f[g] = true;
          cyclefaces[g.getIndex()] = {0.0,0.0,1.0};
        }
      }
    }

    double c_length = 0.0;
    for (auto he : selected_cycle) {
      c_length += nm->geometry->edgeLengths[he.edge()];
    }
  
    double total_area = 0.0;
    for (size_t i = 0; i < nm->mesh->nFaces(); i++)
    {
      total_area += nm->geometry->faceAreas[i];
    }
    psMesh->addFaceColorQuantity("Cycle Faces", cyclefaces);
    std::cout << "Total Area: " << total_area << std::endl;
    std::cout << "Partial Area (Blue): " << area_sum << " Partial Area (Green): " << total_area - area_sum << std::endl;
    std::cout << "Cycle Length: " << c_length << std::endl;
    std::cout << "Tightness:" << min(area_sum, total_area - area_sum) / (c_length*c_length) << std::endl;
  }

  if (ImGui::Button("See Cycle")) {
    // std::cout << cycle_sel << std::endl;

    auto test_cycle = nm->salient_cycles_output[cycle_sel];

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
    auto surf = polyscope::getSurfaceMesh(mesh_name);
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

int main(int argc, char **argv)
{

  // Configure the argument parser
  args::ArgumentParser parser("geometry-central & Polyscope example project");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");
  args::Positional<std::string> shouldshow(parser, "show", "if you should show");
  // Parse args
  try
  {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help &h)
  {
    std::cout << parser;
    return 0;
  }
  catch (args::ParseError &e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename)
  {
    std::cerr << "Please specify a mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  polyscope::options::allowHeadlessBackends = true;
  // Initialize polyscope
  polyscope::init();

  // Set the callback function
  polyscope::state::userCallback = myCallback;

  //
  NeckModel nmtemp = NeckModel(args::get(inputFilename));
  nm = std::unique_ptr<NeckModel>(std::move(&nmtemp));
  // TEMP
  nm->_source = nm->mesh->vertex(0);
  // nm->_anti_source = nm->mesh->vertex(5687);
  // toe: 5687
  // finger: 21497
  // head: 17815
  // nm->_source = nm->mesh->vertex(4788);

  // teapot tip: 2696
  mesh_name = polyscope::guessNiceNameFromPath(args::get(inputFilename));
  psMesh = polyscope::registerSurfaceMesh(mesh_name,
      nm->geometry->inputVertexPositions, nm->mesh->getFaceVertexList());
      
  auto perms = polyscopePermutations(*(nm->mesh));
  psMesh->setAllPermutations(perms);
  // psMesh->centerBoundingBox();
  psMesh->setPosition(glm::vec3{0.,0.,0.});

  // std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 1.0, 0.0});
  // vcolors[nm->_source.getIndex()] = {1.0, 0.0, 0.0};
  // psMesh->addVertexColorQuantity("vcolor", vcolors)->setEnabled(false);

  // Register Overlay curve network
  auto nodes = nm->geometry->inputVertexPositions;
  // Create edge list of vector of array of size 2 of size_t values from nodes indices
  std::vector<std::array<size_t, 2>> edges;
  for (Edge e : nm->mesh->edges())
  {
    edges.push_back({e.halfedge().vertex().getIndex(), e.halfedge().twin().vertex().getIndex()});
  }
  auto curve = polyscope::registerCurveNetwork("curve", nodes, edges);
  curve->setRadius(0.0002);
  curve->setEnabled(false);

  std::cout << "Running Algo" << std::endl;
  nm->_source = nm->mesh->vertex(12756);
  // finalStretch(nm);
  // findLeaders(nm, false);
  computeSkeleton(nm);

  std::cout << "Taking Photos" << std::endl;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  {
    using namespace polyscope;
    
    // Give ourselves some valid view
    view::lookAt(glm::vec3{1., 0., 0.},glm::vec3{0., 0., 0.});
    
    // Make Polyscope compute + set the best home view
    view::resetCameraToHomeView();
    polyscope::screenshot();

    // Rotate 7 times along yvec
    // glm::vec3 frameLookDir, frameUpDir, frameRightDir;
    // view::getCameraFrame(frameLookDir, frameUpDir, frameRightDir);
    // float theta = -(2 * polyscope::PI) / 8.0;
    //   view::viewMat = glm::translate(view::viewMat, view::viewCenter);
    //   glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0), theta, view::getUpVec());
    //   view::viewMat = view::viewMat * thetaCamR;
    //   view::viewMat = glm::translate(view::viewMat, -view::viewCenter);
    // polyscope::screenshot();
/*    for (int i =0; i < 7; i++){
      view::viewMat = glm::translate(view::viewMat, view::viewCenter);
      glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0), theta, view::getUpVec());
      view::viewMat = view::viewMat * thetaCamR;
      view::viewMat = glm::translate(view::viewMat, -view::viewCenter);
      screenshot();
    }

    // Do a small rotation along xvec
    polyscope::view::resetCameraToHomeView();
    view::getCameraFrame(frameLookDir, frameUpDir, frameRightDir);
    view::viewMat = glm::translate(view::viewMat, view::viewCenter);
    glm::mat4x4 phiCamR = glm::rotate(glm::mat4x4(1.0), -theta, frameRightDir);
    view::viewMat = view::viewMat * phiCamR;
    view::viewMat = glm::translate(view::viewMat, -view::viewCenter);
    screenshot();

    // Rotate 7 more times along this vector.
    for (int i =0; i < 7; i++){
      view::viewMat = glm::translate(view::viewMat, view::viewCenter);
      glm::mat4x4 thetaCamR = glm::rotate(glm::mat4x4(1.0), theta, view::getUpVec());
      view::viewMat = view::viewMat * thetaCamR;
      view::viewMat = glm::translate(view::viewMat, -view::viewCenter);
      screenshot();
    }*/
  }
  // polyscope::view::lookAt(pos, glm::vec3{0., 0., 0.});
  // polyscope::screenshot();

  
  // polyscope::requestRedraw();
  // std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // polyscope::view::resetCameraToHomeView();
  // polyscope::view::resetCameraToHomeView();
  // polyscope::view::resetCameraToHomeView();

  // std::cout << currParams.get
  // double aX = pos[0];
  // double aY = pos[1];
  // double aZ = pos[2];

  // double mag = (sqrt(aX*aX + aZ*aZ));
  // std::cout << aX << ", " << aY << ", " << aZ << std::endl;
  // std::cout << la[0] << ", " << la[1] << ", " << la[2] << std::endl;
  // std::cout << currParams.getFoVVerticalDegrees() << std::endl;
  // std::cout << glm::to_string(currParams.getUpDir()) <<  std::endl;
  // std::cout << glm::to_string(currParams.getRightDir()) <<  std::endl;
  // polyscope::screenshot();

  // polyscope::view::setUpDir(polyscope::UpDir::YUp);
  // polyscope::view::setFrontDir(polyscope::FrontDir::XFront);
  // for (float angle; angle < 2.0 * PI; angle += ((2.0 * PI) / 8))
  // for (int i = 0; i < 8; i++)
  // {
  //   float angle = (2.0 * PI)/8.0;
  //   polyscope::CameraParameters currParams = polyscope::view::getCameraParametersForCurrentView();
  //   glm::vec3 curpos = currParams.getPosition();
  //   glm::vec3 curla  = currParams.getLookDir();

  //   glm::mat4 rotmat = glm::rotate(glm::mat4(1.0f), angle, polyscope::view::getUpVec());
  //   glm::vec3 t2c = curpos - curla;
  //   curpos = curla + glm::vec3(rotmat * glm::vec4(t2c, 1.0f));
  //   polyscope::view::lookAt(curpos, t2c, polyscope::view::getUpVec());
  //   // double _X = cos(angle) * mag;
  //   // double _Z = sin(angle) * mag;
  //   // polyscope::view::lookAt(glm::vec3{_X, aY, _Z}, polyscope::view::viewCenter);
  //   // std::cout << glm::to_string(polyscope::view::viewCenter) << std::endl;
  //   // std::cout << currParams.getFoVVerticalDegrees() << std::endl;
  //   // polyscope::view::lookAt(pos, polyscope::view::viewCenter);
  //   polyscope::screenshot();
  // }

  // // // polyscope::view::lookAt(pos,la);

  // for (double angle; angle < 2.0 * PI; angle += ((2.0 * PI) / 8))
  // {
  //   double _X = cos(angle) * mag;
  //   double _Z = sin(angle) * mag;
  //   polyscope::view::lookAt(glm::vec3{_X, -(1.1*aY), _Z}, glm::vec3{0., 0., 0.});
  //   polyscope::screenshot();
  // }

  // polyscope::view::lookAt(pos,la);
  if (shouldshow) {
    polyscope::show();
  }
  // polyscope::show();

  nm.release();
  nm = NULL;
  return EXIT_SUCCESS;
}
