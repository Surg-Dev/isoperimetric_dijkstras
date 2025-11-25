
#include "benchmarker.hpp"
#include "curve_export.hpp"
#include "geometrycentral/surface/mesh_graph_algorithms.h"
#include "geometrycentral/surface/flip_geodesics.h"

void computeSkeleton(std::unique_ptr<NeckModel> &nm){
  // srand(time(NULL));
  int64_t last_timing = 0;
  std::ofstream file("timing", std::ios::app);

  double total_area = 0.0;
  for (size_t i = 0; i < nm->mesh->nFaces(); i++)
  {
    total_area += nm->geometry->faceAreas[i];
  }

  std::vector<glm::vec3> output_ve;
  std::vector<std::array<size_t, 2>> output_ed;
  int base_count = 0;

  auto start = std::chrono::steady_clock::now();

  ///////////// Pick a Good Root Point
  // // Pick a random source vertex: X
  int ridx = std::rand() % nm->mesh->nVertices();
  Vertex Xc = nm->mesh->vertex(ridx);
  auto sssp_Xc = nm->sssp_report_furthest(Xc);
  vPair Yc = sssp_Xc.second;
  auto sssp_Yc = nm->sssp_report_furthest(Yc.second);
  vPair Zc = sssp_Yc.second;

  nm->_source = Zc.second;
  auto candidates = findLeaders(nm,true,25);

  // Find the longest distance from source
  vPair spair = candidates.back();
  candidates.pop_back();
  float maxv = 0.0;
  vPair tpair;
  auto todel = candidates.begin();
  // Get maximum and remove it from the candidate list.
  for (auto it = candidates.begin(); it < candidates.end(); it++){
    auto x = *it;
    if (x.first > maxv){
      maxv = x.first;
      tpair = x;
      todel = it;
    } 
  }
  candidates.erase(todel);


  //////////// Build the skeleton

  // We have the sssp of the diameter. We're going to perform the following loop:
  // Add each node in the path to a set (the skeleton)
  // For each candidate, compute the shortest path from it to the skeleton
  // Add all of those nodes into the skeleton, repeat until candidates are exhausted.

  std::unordered_set<Vertex> skeleton;
  std::vector<std::vector<Halfedge>> skeleton_paths;
  auto sssp = nm->st_dijkstras(spair.second, tpair.second);
  auto spine = nm->get_he_path(sssp, spair.second, tpair.second);
  skeleton_paths.push_back(spine);

  std::cout << "Start: " << spair.second << " End: " << tpair.second << std::endl;
  Vertex t_spine = spine.at(0).twin().vertex();
  std::cout << "Inserting: " << t_spine << std::endl;
  skeleton.insert(t_spine);
  for (auto he : spine){
    Vertex v = he.vertex();
    std::cout << "Inserting: " << v << std::endl;
    skeleton.insert(v);
  }

  for (auto candidate : candidates){
    Vertex s = candidate.second;
    auto stres = nm->stgroup_dijkstras(s, skeleton);
    sssp_t st_sssp = stres.first;
    Vertex t = stres.second;

    auto bone = nm->get_he_path(st_sssp, s,t);
    skeleton_paths.push_back(bone);
    Vertex t_bone = bone.at(0).twin().vertex();
    skeleton.insert(t_bone);
    for (auto he : bone){
      Vertex v = he.vertex();
      skeleton.insert(v);
    }
    std::cout << "Inserted Bone" << std::endl;
  }


  ////////// BASIC DISPLAY 
  // std::cout << "Num Verts: " << skeleton.size() << " Num Bones: " << skeleton_paths.size() << std::endl; 

  std::vector<glm::vec3> pcloud;
  for (auto x : skeleton)
  {
    Vector3 vertdat = nm->geometry->vertexPositions[x.getIndex()];
    pcloud.push_back({vertdat.x, vertdat.y, vertdat.z});
  }
  auto gen_pcloud = polyscope::registerPointCloud("paths", pcloud);
  auto curve = polyscope::getCurveNetwork("curve");

  // std::vector<std::vector<std::vector<Halfedge>>> cycle_groups;
  // std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0, 0.0, 0.0});
  // for (auto line : skeleton_paths){
  //   auto cycles = nm->get_cycles_from_path(line);
  //   for (auto c : cycles) {
  //     for (auto he : c) {
  //       Edge e = he.edge();
  //       ecolors[e.getIndex()] = {1.0, 0., 0.};
  //     }
  //   }
  //   // cycle_groups.push_back(cycles);
  // }
  // curve->addEdgeColorQuantity("path", ecolors);

  // std::cout << "endtime: " << since(start).count() << std::endl;
  //////////

  auto out_cycles = nm->get_cycles_from_skeleton(skeleton_paths);
  std::cout << "finding all cycles: " << since(start).count() << std::endl;
  // Build the "tree" of bones
  // add 
  // for (auto cycles : out_cycles){

  //   std::vector<float> tightness = std::vector<float>(cycles.size());

  //   // Lazyily compute tightness
  //   for (size_t i = 0; i < cycles.size(); i++){
  //     auto selected_cycle = cycles[i];
  //     FaceData<bool> visited_f(*(nm->mesh)); // Visited Set of faces
  //     std::vector<std::array<double, 3>> cyclefaces(nm->mesh->nFaces(), {0.0, 1.0, 0.0});
  //     std::queue<Face> bfs_q;

  //     std::unordered_set<Face> banned_faces; // Banned faces from enquing in one iteration
  //       double area_sum = 0.0;
  //       for (auto he : selected_cycle)
  //       {
  //         // enqueue all the faces induced by one side of the cycle
  //         if (visited_f[he.face()] == false)
  //         {
  //           bfs_q.push(he.face());
  //           visited_f[he.face()] = true;
  //           cyclefaces[he.face().getIndex()] = {0.0,0.0,1.0};
  //         }
  //         // ban all the faces induced by the other side (this should prevent all crossings automatically)
  //         banned_faces.insert(he.twin().face());
  //       }

  //     while (!bfs_q.empty())
  //     {
  //       Face f = bfs_q.front();
  //       bfs_q.pop();
  //       area_sum += nm->geometry->faceAreas[f];
  //       for (Face g : f.adjacentFaces())
  //       {
  //         if (visited_f[g] == false && banned_faces.find(g) == banned_faces.end())
  //         {
  //           bfs_q.push(g);
  //           visited_f[g] = true;
  //           cyclefaces[g.getIndex()] = {0.0,0.0,1.0};
  //         }
  //       }
  //     }

  //     double c_length = 0.0;
  //     for (auto he : selected_cycle) {
  //       c_length += nm->geometry->edgeLengths[he.edge()];
  //     }
  //     tightness[i] = min(area_sum, total_area - area_sum) / (c_length*c_length);
  //   }
  // // std::cout << "finding all tightness: " << since(start).count() << std::endl;


  //     std::vector<bool> local_max_cycle(cycles.size());
  //     if (cycles.size() > 7)
  //     {
  //       for (size_t i = 3; i < cycles.size() - 3; i++)
  //       {
  //         if (tightness[i] > tightness[i + 1] && tightness[i] > tightness[i - 1] && tightness[i] > tightness[i + 2] && tightness[i] > tightness[i - 2])
  //         {
  //           if (tightness[i] > tightness[i + 3] && tightness[i] > tightness[i - 3])
  //             local_max_cycle[i] = true;
  //           //std::cout << "Cycle #" << i << " is a 5-wide local max" << std::endl;
  //         }
  //         //std::cout << i << ", " << tightness[i] << std::endl; 
  //       }
  //     }
  // // Cycle path display:
  //   for (size_t i = 0; i < cycles.size(); i++)
  //     {
  //         if (!local_max_cycle[i]) {
  //           continue;
  //         }
  //         if (tightness[i] <= .16) {
  //           continue;
  //         }
  //         for (size_t j = 0; j < cycles[i].size(); j++)
  //         {
  //           Halfedge he = cycles[i][j];
  //           Vector3 vertdat = nm->geometry->vertexPositions[he.tailVertex()];
  //           output_ve.push_back({vertdat.x, vertdat.y, vertdat.z});
  //           // output_ve.push_back(nm->geometry .tailVertex().getIndex);
  //           // std::cout << j << ", " << (j+1) % 37 << std::endl;
  //           output_ed.push_back({base_count + j, base_count + ((j + 1) % (cycles[i].size()))});
  //           // ecolors[he.edge().getIndex()] = {1.0, 0.0, 0.0};
  //         }
  //         base_count += cycles[i].size();

  //     }
  //   }


    for (auto cycles : out_cycles){
      std::vector<float> cycle_lens;
      for (auto he_cycle : cycles)
      {
        float a = 0.0;
        for (auto he : he_cycle)
        {
          a += nm->geometry->edgeLengths[he.edge()];
        }
        cycle_lens.push_back(a);
      }

      // For each cycle, add each face into the queue
      // Run BFS, preventing the crossing of a cycle.

      FaceData<bool> visited(*(nm->mesh)); // Visited Set of faces
      std::queue<Face> bfs_q;
      std::vector<double> segment_lengths = std::vector<double>(cycles.size() + 1); // The table of constrained areas
      size_t ind = 0;
      for (auto he_cycle : cycles)
      {
        std::unordered_set<Face> banned_faces; // Banned faces from enquing in one iteration
        double area_sum = 0.0;
        for (auto he : he_cycle)
        {
          // enqueue all the faces induced by one side of the cycle
          if (visited[he.face()] == false)
          {
            bfs_q.push(he.face());
            visited[he.face()] = true;
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
            if (visited[g] == false && banned_faces.find(g) == banned_faces.end())
            {
              bfs_q.push(g);
              visited[g] = true;
            }
          }
        }
        segment_lengths[ind] = area_sum;
        ind++;
      }

      // Do last segment:

      {
        std::unordered_set<Face> banned_faces;
        double area_sum = 0.0;
        auto he_cycle = cycles[ind - 1];
        for (auto he : he_cycle)
        {
          auto hetw = he.twin();
          // enqueue all the faces induced by one side of the cycle
          if (visited[hetw.face()] == false)
          {
            bfs_q.push(hetw.face());
            visited[hetw.face()] = true;
          }
          // ban all the faces induced by the other side (this should prevent all crossings automatically)
          banned_faces.insert(hetw.twin().face());
        }
        while (!bfs_q.empty())
        {
          Face f = bfs_q.front();
          bfs_q.pop();
          area_sum += nm->geometry->faceAreas[f];
          for (Face g : f.adjacentFaces())
          {
            if (visited[g] == false && banned_faces.find(g) == banned_faces.end())
            {
              bfs_q.push(g);
              visited[g] = true;
            }
          }
        }
        segment_lengths[ind] = area_sum;
      }
  
      std::vector<double> segment_prefix_sums = std::vector<double>(segment_lengths.size());
      std::vector<double> tightness = std::vector<double>(cycles.size());
      segment_prefix_sums[0] = segment_lengths[0];

      for (size_t i = 1; i < segment_lengths.size(); i++)
      {
        segment_prefix_sums[i] = segment_prefix_sums[i - 1] + segment_lengths[i];
      }

      for (size_t i = 0; i < tightness.size(); i++)
      {
        tightness[i] = std::min(segment_prefix_sums[i], total_area - segment_prefix_sums[i]) / (cycle_lens[i] * cycle_lens[i]);
      }

      std::vector<bool> local_max_cycle(cycles.size());
      if (cycles.size() > 7)
      {
        for (size_t i = 3; i < cycles.size() - 3; i++)
        {
          if (tightness[i] <= .16) {
            continue;
          }
          if (tightness[i] > tightness[i + 1] && tightness[i] > tightness[i - 1] && tightness[i] > tightness[i + 2] && tightness[i] > tightness[i - 2])
          {
            if (tightness[i] > tightness[i + 3] && tightness[i] > tightness[i - 3])
              local_max_cycle[i] = true;
          }
        }
      }

      // Cycle path display:
      for (size_t i = 0; i < cycles.size(); i++)
        {
            if (!local_max_cycle[i]) {
              continue;
            }

            for (size_t j = 0; j < cycles[i].size(); j++)
            {
              Halfedge he = cycles[i][j];
              Vector3 vertdat = nm->geometry->vertexPositions[he.tailVertex()];
              output_ve.push_back({vertdat.x, vertdat.y, vertdat.z});
              output_ed.push_back({base_count + j, base_count + ((j + 1) % (cycles[i].size()))});
            }
            base_count += cycles[i].size();

        }

    }
    nm->skeleton_cycles_output = out_cycles;
    auto curve2 = polyscope::registerCurveNetwork("cyclecurve", output_ve, output_ed);
    curve2->setColor({1.0, 0.0, 0.0});
    curve2->setPosition(glm::vec3{0.,0.,0.});
    // nm->salient_cycles_output = cycles;
    std::cout << "done: " << since(start).count() << std::endl;
    ///
}

void finalStretch(std::unique_ptr<NeckModel> &nm)
{

  // std::vector<std::vector<Vector3>> polylines;

  int64_t last_timing = 0;
  std::ofstream file("timing", std::ios::app);

  double total_area = 0.0;
  for (size_t i = 0; i < nm->mesh->nFaces(); i++)
  {
    total_area += nm->geometry->faceAreas[i];
  }

  std::vector<glm::vec3> output_ve;
  std::vector<std::array<size_t, 2>> output_ed;
  int base_count = 0;

  ////// BASIC CANDIDATE PICKING
  auto start = std::chrono::steady_clock::now();
  // // Pick a random source vertex: X
  int ridx = std::rand() % nm->mesh->nVertices();
  Vertex Xc = nm->mesh->vertex(ridx);
  auto sssp_Xc = nm->sssp_report_furthest(Xc);
  vPair Yc = sssp_Xc.second;
  auto sssp_Yc = nm->sssp_report_furthest(Yc.second);
  vPair Zc = sssp_Yc.second;

  nm->_source = Zc.second;
  auto candidates = findLeaders(nm);

  // std::vector<Vertex> stripped_candidates;

  // for (auto x : candidates) {
  //   stripped_candidates.push_back(x.second);
  // }
  // computeCandidateMST(nm, stripped_candidates);

  for (auto Y : candidates)
  {
    // // Run SSSP from X, Select the furthest vertex from X, Y

    // Run SSSP from Y, Select the furthest vertex from Y, Z
    auto sssp_Y = nm->sssp_report_furthest(Y.second);
    vPair Z = sssp_Y.second;

    // Run SSSP from Z
    auto sssp_Z = nm->sssp_report_furthest(Z.second);
    vPair antinode = sssp_Z.second; // This should match Y

    // std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 0.0, 0.0});

    // vcolors[X.getIndex()] = {1.0, 1.0, 0.0};
    // vcolors[Y.second.getIndex()] = {1.0, 0.0, 0.0};
    // vcolors[Z.second.getIndex()] = {0.0, 1.0, 0.0};
    // vcolors[antinode.second.getIndex()] = {0.0, 1.0, 1.0};
    // // std::cout << "X: " << "0.0" << " " << X << std::endl;
    // // std::cout << "Y: " << Y.first << " " << Y.second << std::endl;
    // // std::cout << "Z: " << Z.first << " " << Z.second << std::endl;
    // // std::cout << "A: " << antinode.first << " " << antinode.second << std::endl;
    // auto curve = polyscope::getCurveNetwork("curve");
    // curve->addNodeColorQuantity("leaders", vcolors);

    // Find candidate points with r-leaders, report

    ////// CYCLE FINDING ON SALIENT PATH

    // from Y-Z, run the salient line bottle neck algorithm
    auto he_path = nm->get_he_path(sssp_Y.first, Y.second, Z.second);
    auto cycles = nm->get_cycles_from_path(he_path);

    nm->salient_cycles_output = cycles;

    std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0, 0.0, 0.0});

    for (auto he : he_path)
    {
      Edge e = he.edge();
      ecolors[e.getIndex()] = {1.0, 1.0, .12};
    }
    // std::cout << "V: " << nm->mesh->nVertices() << ", E: " << nm->mesh->nEdges() << ", F: " << nm->mesh->nFaces() << std::endl;

    ////// TIGHTNESS ANALYSIS
    // Get the cycle lengths
    std::vector<float> cycle_lens;

    for (auto he_cycle : cycles)
    {
      float a = 0.0;
      for (auto he : he_cycle)
      {
        a += nm->geometry->edgeLengths[he.edge()];
      }
      cycle_lens.push_back(a);
    }

    // For each cycle, add each face into the queue
    // Run BFS, preventing the crossing of a cycle.

    FaceData<bool> visited(*(nm->mesh)); // Visited Set of faces
    std::queue<Face> bfs_q;
    std::vector<double> segment_lengths = std::vector<double>(cycles.size() + 1); // The table of constrained areas
    size_t ind = 0;
    for (auto he_cycle : cycles)
    {
      std::unordered_set<Face> banned_faces; // Banned faces from enquing in one iteration
      double area_sum = 0.0;
      for (auto he : he_cycle)
      {
        // enqueue all the faces induced by one side of the cycle
        if (visited[he.face()] == false)
        {
          bfs_q.push(he.face());
          visited[he.face()] = true;
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
          if (visited[g] == false && banned_faces.find(g) == banned_faces.end())
          {
            bfs_q.push(g);
            visited[g] = true;
          }
        }
      }
      segment_lengths[ind] = area_sum;
      ind++;
    }

    // Do last segment:

    {
      std::unordered_set<Face> banned_faces;
      double area_sum = 0.0;
      auto he_cycle = cycles[ind - 1];
      for (auto he : he_cycle)
      {
        auto hetw = he.twin();
        // enqueue all the faces induced by one side of the cycle
        if (visited[hetw.face()] == false)
        {
          bfs_q.push(hetw.face());
          visited[hetw.face()] = true;
        }
        // ban all the faces induced by the other side (this should prevent all crossings automatically)
        banned_faces.insert(hetw.twin().face());
      }
      while (!bfs_q.empty())
      {
        Face f = bfs_q.front();
        bfs_q.pop();
        area_sum += nm->geometry->faceAreas[f];
        for (Face g : f.adjacentFaces())
        {
          if (visited[g] == false && banned_faces.find(g) == banned_faces.end())
          {
            bfs_q.push(g);
            visited[g] = true;
          }
        }
      }
      segment_lengths[ind] = area_sum;
    }

    // Get the total area of the mesh

    // Verify that these two values are roughly equal
    // std::cout << "Total Area: " << total_area << " Segment Sum Area: " << std::accumulate(segment_lengths.begin(), segment_lengths.end(), 0.0) << std::endl;

    // Compute the prefix sums of the mesh/cycle separation

    std::vector<double> segment_prefix_sums = std::vector<double>(segment_lengths.size());
    std::vector<double> tightness = std::vector<double>(cycles.size());
    segment_prefix_sums[0] = segment_lengths[0];

    for (size_t i = 1; i < segment_lengths.size(); i++)
    {
      segment_prefix_sums[i] = segment_prefix_sums[i - 1] + segment_lengths[i];
    }

    for (size_t i = 0; i < tightness.size(); i++)
    {
      tightness[i] = std::min(segment_prefix_sums[i], total_area - segment_prefix_sums[i]) / (cycle_lens[i] * cycle_lens[i]);
    }

    // std::cout << "Tightness: " << std::endl;
    // for (size_t i = 0; i < tightness.size(); i++)
    // {
    //   std::cout << tightness[i] << " ";
    // }
    // std::cout << std::endl;

    
    // auto test_cycle = cycles[30];
    // std::vector<std::array<double, 3>> cyclefaces(nm->mesh->nFaces(), {0.0, 0.0, 0.0});
    
    // for (auto he : test_cycle)
    // {
      //   Face f = he.face();
      //   cyclefaces[f.getIndex()] = {0.0, 1.0, 0.0};
      // }
      
      // for (auto face : Y.second.adjacentFaces())
      // {
        //   cyclefaces[face.getIndex()] = {1.0, 0.0, 0.0};
        // }
        
        // for (auto face : Z.second.adjacentFaces())
        // {
          //   cyclefaces[face.getIndex()] = {0.0, 0.0, 1.0};
          // }
          // auto surf = polyscope::getSurfaceMesh("human_tri");
          // surf->addFaceColorQuantity("cyclefaces", cyclefaces);
          
          //std::cout << "Loop" << std::endl;
    std::vector<bool> local_max_cycle(cycles.size());
    if (cycles.size() > 7)
    {
      for (size_t i = 3; i < cycles.size() - 3; i++)
      {
        if (tightness[i] <= .159) {
          continue;
        }
        if (tightness[i] > tightness[i + 1] && tightness[i] > tightness[i - 1] && tightness[i] > tightness[i + 2] && tightness[i] > tightness[i - 2])
        {
          if (tightness[i] > tightness[i + 3] && tightness[i] > tightness[i - 3])
            local_max_cycle[i] = true;
          //std::cout << "Cycle #" << i << " is a 5-wide local max" << std::endl;
        }
        //std::cout << i << ", " << tightness[i] << std::endl; 
      }
    }

    // for (int i = 2; i < cycles.size() - 2; i++)
    // {
    //   if (tightness[i] > tightness[i + 1] && tightness[i] > tightness[i - 1])
    //   {
    //     local_max_cycle[i] = true;
    //     // std::cout << "Cycle #" << i << " is a 5-wide local max" << std::endl;
    //   }
    // }

    // for (int i = 2; i < cycles.size() - 2; i++)
    // {
    //   if (tightness[i] < tightness[i + 1] && tightness[i] < tightness[i - 1] && tightness[i] < tightness[i + 2] && tightness[i] < tightness[i - 2])
    //   {
    //     local_min_cycle[i] = true;
    //     std::cout << "Cycle #" << i << " is a 5-wide local min" << std::endl;
    //   }
    // }
    // for (int i = 1; i < cycles.size()-1; i++) {
    //   if (cycle_lens[i] >= cycle_lens[i+1] && cycle_lens[i] < cycle_lens[i-1]) {
    //     local_min_cycle[i] = true;
    //   }
    // }
    // local_min_cycle[30] = true;
    // local_min_cycle[0] = true;
    // local_min_cycle[152] = true;
    // local_min_cycle[218] = true;
    // local_min_cycle[258] = true;

    // std::vector<glm::vec3> output_ve_min;
    // std::vector<std::array<size_t, 2>> output_ed_min;

    // for (int i =0; i < cycles.size(); i++) {
    //   if (local_min_cycle[i]){
    //     for (auto he : cycles[i]){
    //       ecolors[he.edge().getIndex()] = {1.0,0.0,0.0};
    //     }
    //   }
    // }
    

    // ///// FLIP GEODESICS
    // {
    //   for (size_t i =0; i < cycles.size(); i++) {
    //     if (local_max_cycle[i]){
    //         std::unique_ptr<FlipEdgeNetwork> flipNet(new FlipEdgeNetwork(*(nm->mesh), *(nm->geometry), {cycles[i]}));
    //         flipNet->posGeom = nm->geometry.get();
    //         flipNet->iterativeShorten();
    //         std::vector<Vector3> path3D = flipNet->getPathPolyline3D().front();
    //         polylines.push_back(path3D);
    //         flipNet.reset();
    //     }
    //   }
    // }

    //////////

    for (size_t i = 0; i < cycles.size(); i++)
    {
      if (local_max_cycle[i])
      {
        for (size_t j = 0; j < cycles[i].size(); j++)
        {
          Halfedge he = cycles[i][j];
          Vector3 vertdat = nm->geometry->vertexPositions[he.tailVertex()];
          output_ve.push_back({vertdat.x, vertdat.y, vertdat.z});
          // output_ve.push_back(nm->geometry .tailVertex().getIndex);
          // std::cout << j << ", " << (j+1) % 37 << std::endl;
          output_ed.push_back({base_count + j, base_count + ((j + 1) % (cycles[i].size()))});
          // ecolors[he.edge().getIndex()] = {1.0, 0.0, 0.0};
        }
        base_count += cycles[i].size();
      }
    }
    int64_t cand_timing = since(start).count();
    file << cand_timing - last_timing << std::endl;
    last_timing = cand_timing;
    // break;
  }
  file << "total " << since(start).count() << std::endl;
  std::cout << "Elapsed(ms)=" << since(start).count() << std::endl;
  auto curve2 = polyscope::registerCurveNetwork("cyclecurve", output_ve, output_ed);
  curve2->setColor({1.0, 0.0, 0.0});
  // curve2->centerBoundingBox();
  curve2->setPosition(glm::vec3{0.,0.,0.});

  export_curve_network_obj(output_ve, output_ed, "curvenet.obj");

  // int i = 100;
  // for (auto line : polylines){
  //     auto subcurve = polyscope::registerCurveNetworkLoop(std::to_string(i), line);
  //     subcurve->setColor({1.0, 0.5, 1.0});
  //     // curve2->centerBoundingBox();
  //     subcurve->setPosition(glm::vec3{0.,0.,0.});
  //     i++;
  // }

  // int base_count_min = 0;
  // for (size_t i = 0; i < cycles.size(); i++) {
  //   if (local_min_cycle[i]) {
  //     for (size_t j = 0 ; j < cycles[i].size(); j++) {
  //       Halfedge he = cycles[i][j];
  //       Vector3 vertdat = nm->geometry->vertexPositions[he.tailVertex()];
  //       output_ve_min.push_back({vertdat.x, vertdat.y, vertdat.z});
  //       // output_ve.push_back(nm->geometry .tailVertex().getIndex);
  //       // std::cout << j << ", " << (j+1) % 37 << std::endl;
  //       output_ed_min.push_back({base_count_min + j, base_count_min +((j+1) % (cycles[i].size()))});
  //       // ecolors[he.edge().getIndex()] = {1.0, 0.0, 0.0};
  //     }
  //     base_count_min += cycles[i].size();
  //   }
  // }
  // auto curve3 = polyscope::registerCurveNetwork("cyclecurve_min", output_ve_min, output_ed_min);
  // curve3->setColor({0.0,1.0,0.0});
  // // std::cout << output_ve.size() << std::endl;
  // std::cout << output_ed.size() << std::endl;

  // for (auto a : local_min_cycle) {
  //   std::cout << a << ", ";
  // }
  // std::cout << std::endl;
  // curve->addEdgeColorQuantity("path", ecolors);
}

std::vector<vPair> findLeaders(std::unique_ptr<NeckModel> &nm, bool trim, int r)
{
  auto path = nm->sssp(nm->_source);
  auto prev = path.first;
  auto dists = path.second;

  // Find all local candidates
  std::vector<std::pair<float, Vertex>> candidates;

  for (Vertex v : nm->mesh->vertices())
  {
    bool is_candidate = true;
    for (Vertex u : v.adjacentVertices())
    {
      if (dists[v] < dists[u])
      {
        is_candidate = false;
        break;
      }
    }
    if (is_candidate)
    {
      candidates.push_back({dists[v], v});
    }
  }
  std::cout << "# of Candidates/Verts " << candidates.size() << "/" << nm->mesh->nVertices() << std::endl;

  // Sort candidates by distance
  struct
  {
    bool operator()(std::pair<float, Vertex> a, std::pair<float, Vertex> b) const { return a.first > b.first; }
  } pair_ge;

  std::sort(candidates.begin(), candidates.end(), pair_ge);
  // for (size_t i = 0; i < candidates.size(); i++)
  // {
  //   std::cout << candidates[i].first << ", " << candidates[i].second << std::endl;
  // }

  // Remove All Candidates within r-hops
  if (trim){
    typedef std::pair<int, Vertex> VHop;
    for (size_t i = 0; i < candidates.size(); i++)
    {
      std::queue<VHop> q;
      std::unordered_set<Vertex> visited;
      Vertex s = candidates[i].second;
      q.push({0, s});
      visited.insert(s);

      while (!q.empty())
      {
        auto hop = q.front().first;
        auto curr = q.front().second;
        q.pop();
        if (hop + 1 == r)
        {
          continue;
        }
        auto it = candidates.begin();
        for (; it < candidates.end(); it++)
        {
          if ((*it).second == curr && curr != s)
          {
            candidates.erase(it);
            break;
          }
        }

        for (Vertex v : curr.adjacentVertices())
        {
          int rp = hop + 1;
          if (visited.find(v) == visited.end())
          {
            visited.insert(v);
            q.push({rp, v});
          }
        }
      }
    }
  }

  std::cout << "# of Candidates Post Trim " << candidates.size() << std::endl;
  // Mark vertices on structure
  std::vector<std::array<double, 3>>
      vcolors(nm->mesh->nVertices(), {0.0, 0.0, 0.0});

  for (size_t i = 0; i < candidates.size(); i++)
  {
    vcolors[candidates[i].second.getIndex()] = {1.0, 0.0, 0.0};
  }

  std::vector<glm::vec3> pcloud;
  for (auto x : candidates)
  {
    Vector3 vertdat = nm->geometry->vertexPositions[x.second.getIndex()];
    pcloud.push_back({vertdat.x, vertdat.y, vertdat.z});
  }
  Vector3 srcdat = nm->geometry->vertexPositions[nm->_source.getIndex()];
  pcloud.push_back({srcdat.x, srcdat.y, srcdat.z});

  auto gen_pcloud = polyscope::registerPointCloud("critpts", pcloud);
  
  auto curve = polyscope::getCurveNetwork("curve");
  curve->addNodeColorQuantity("leaders", vcolors);
  
  candidates.push_back({0.0, nm->_source});

  // gen_pcloud->centerBoundingBox();
  // curve->centerBoundingBox();
  gen_pcloud->setPosition(glm::vec3{0.,0.,0.});
  curve->setPosition(glm::vec3{0.,0.,0.});
  return candidates;
}

void salientLine(std::unique_ptr<NeckModel> &nm)
{
  auto path = nm->st_dijkstras(nm->_source, nm->_anti_source);
  auto prev = path.first;
  auto dists = path.second;

  // Mark edges that are on this path
  std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0, 0.0, 0.0});
  Vertex v = nm->_source;
  Vertex cur = nm->_anti_source;

  std::vector<Halfedge> he_path;

  while (cur != v)
  {
    Halfedge he = prev[cur];
    he_path.push_back(he);
    Edge e = he.edge();
    ecolors[e.getIndex()] = {1.0, 0.0, 0.0};
    cur = he.tailVertex();
  }
  auto curve = polyscope::getCurveNetwork("curve");
  curve->addEdgeColorQuantity("path", ecolors);

  // get len
  float len = dists[he_path[0].tipVertex()];
  float test_len = len;
  size_t index = 0;
  while (test_len > len / 6.0)
  {
    index++;
    test_len = dists[he_path[index].tipVertex()];
  }

  // // Find cycle at n/2
  // size_t len = he_path.size();
  size_t midpt = index + 3;

  std::cout << "midpt/len: " << midpt << " midpt/ind: " << he_path.size() / 2 << std::endl;

  Halfedge he_mid = he_path[midpt];
  Halfedge he_pred = he_path[midpt + 1];

  Vertex a = he_mid.tailVertex();
  // Vertex b = he_pred.tipVertex();

  // get list of edges on either side of cycle at POI
  // Sides are arbitrary, it switches once you encounter the hedges on the path.

  std::set<Halfedge> left, right;
  int side = 0;
  for (Halfedge he : a.outgoingHalfedges())
  {
    if (he == he_mid || he == he_pred.twin())
    {
      side = !side;
      continue;
    }
    if (side)
    {
      left.insert(he);
    }
    else
    {
      right.insert(he);
    }
  }
  // run st dijkstra's but ban any node on the path, and return halfedges on the side you started with

  // make a banned set
  // add all nodes on the o.g. path to it
  std::set<Vertex> banned_crossing;
  for (auto he : he_path)
  {
    banned_crossing.insert(he.tipVertex());
    banned_crossing.insert(nm->_source);
  }

  // pick a side, enqueue all outgoing verts
  VertexData<Halfedge> prev_c(*(nm->mesh));
  VertexData<float> dists_c(*(nm->mesh), std::numeric_limits<float>::infinity());
  std::priority_queue<vPair, std::vector<vPair>, std::greater<vPair>> pq;
  for (auto he : left)
  {
    float len = nm->geometry->edgeLengths[he.edge()];
    Vertex vert = he.tipVertex();
    pq.push(std::make_pair(len, vert));
    dists_c[vert] = len;
  }
  // run dijksta's until it reaches the same vertex again, banning the same side's twin hedges.
  while (!pq.empty())
  {
    auto curr = pq.top().second;
    if (curr == a)
    {
      break;
    }
    pq.pop();

    for (Halfedge he : curr.outgoingHalfedges())
    {
      // don't allow instant return.
      if (left.find(he.twin()) != left.end())
      {
        continue;
      }
      Vertex v = he.twin().vertex();
      if (v != a && banned_crossing.find(v) != banned_crossing.end())
      {
        continue;
      }
      float newdist = dists_c[curr] + nm->geometry->edgeLengths[he.edge()];

      if (newdist < dists_c[v])
      {
        dists_c[v] = newdist;
        prev_c[v] = he;
        pq.push(std::make_pair(newdist, v));
      }
    }
  }
  // recover the cycle from the point
  std::vector<std::array<double, 3>> ecycle(nm->mesh->nEdges(), {0.0, 0.0, 0.0});
  cur = a;

  std::vector<Halfedge> he_cycle;

  while (true)
  {

    Halfedge he = prev_c[cur];
    if (he == Halfedge())
    {
      break;
    }
    // std::cout << he << std::endl;
    he_cycle.push_back(he);
    Edge e = he.edge();
    ecycle[e.getIndex()] = {1.0, 0.0, 0.0};
    cur = he.tailVertex();
  }
  auto fin_edge = nm->get_edge(a, cur);
  if (fin_edge.halfedge().tipVertex() == a)
  {
    he_cycle.push_back(fin_edge.halfedge());
  }
  else
  {
    he_cycle.push_back(fin_edge.halfedge().twin());
  }
  ecycle[fin_edge.getIndex()] = {1.0, 0.0, 0.0};
  curve->addEdgeColorQuantity("cycle", ecycle);
  std::cout << he_cycle.size() << std::endl;
  // Recurse above and below
}

void computeCandidateMST(std::unique_ptr<NeckModel> &nm, std::vector<Vertex> candidates){
  // Compute the MST of nm
  auto edgedat = surface::spanningTreeBetweenVertices(*(nm->geometry), candidates);

  std::vector<glm::vec3> nodes;

  for (auto e : nm->mesh->edges()) {
    if (edgedat[e]){
      Vector3 vertdat = nm->geometry->vertexPositions[e.firstVertex()];
      nodes.push_back({vertdat.x, vertdat.y, vertdat.z});
      vertdat = nm->geometry->vertexPositions[e.secondVertex()];
      nodes.push_back({vertdat.x, vertdat.y, vertdat.z});
    }
  }
  polyscope::registerCurveNetworkSegments("mst tree", nodes);

  // Root (orient) the tree from the first candidate

  // Trim off all branches which aren't paths to a candidate

  // Return the set of edges 
}

