
#include "benchmarker.hpp"

void finalStretch(std::unique_ptr<NeckModel> & nm) {
  auto start = std::chrono::steady_clock::now();
      // Pick a random source vertex: X
      int ridx = std::rand() % nm->mesh->nVertices();
      Vertex X = nm->mesh->vertex(ridx);

      // Run SSSP from X, Select the furthest vertex from X, Y
      auto sssp_X = nm->sssp_report_furthest(X);
      vPair Y = sssp_X.second;

      // Run SSSP from Y, Select the furthest vertex from Y, Z
      auto sssp_Y = nm->sssp_report_furthest(Y.second);
      vPair Z = sssp_Y.second;

      // Run SSSP from Z
      auto sssp_Z = nm->sssp_report_furthest(Z.second);
      vPair antinode = sssp_Z.second; // This should match Y
      
      std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0,0.0,0.0});

      vcolors[X.getIndex()]        = {1.0, 1.0, 0.0};
      vcolors[Y.second.getIndex()] = {1.0, 0.0, 0.0};
      vcolors[Z.second.getIndex()] = {0.0, 1.0, 0.0};
      vcolors[antinode.second.getIndex()] = {0.0, 1.0, 1.0};
      // std::cout << "X: " << "0.0" << " " << X << std::endl;
      // std::cout << "Y: " << Y.first << " " << Y.second << std::endl;
      // std::cout << "Z: " << Z.first << " " << Z.second << std::endl;
      // std::cout << "A: " << antinode.first << " " << antinode.second << std::endl;
      auto curve = polyscope::getCurveNetwork("curve");
      curve->addNodeColorQuantity("leaders", vcolors);

      // Find candidate points with r-leaders, report

      // from Y-Z, run the salient line bottle neck algorithm
      auto he_path = nm->get_he_path(sssp_Y.first, Y.second, Z.second);
      auto cycles  = nm->get_cycles_from_path(he_path);

      std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});

      for (auto he : he_path){
        Edge e = he.edge();
        ecolors[e.getIndex()] = {1.0, 1.0, .12};
      }
      std::cout << "Elapsed(ms)=" << since(start).count()  << std::endl;
      std::cout << "V: " << nm->mesh->nVertices() << ", E: " << nm->mesh->nEdges() << ", F: " << nm->mesh->nFaces() << std::endl;

      // for (auto he_cycle : cycles) {
      //   for (auto he : he_cycle) {
      //     ecolors[he.edge().getIndex()] = {1.0, 0.0, 0.0};
      //   }
      // }

      


    // Compute the area between two cycles

    // Just be dumb for now:

    

      // Report best cycles

      std::vector<float> cycle_lens;

      for (auto he_cycle : cycles) {
        float a = 0.0;
        for (auto he : he_cycle) {
          a += nm->geometry->edgeLengths[he.edge()];
        }
        cycle_lens.push_back(a);
      }

      // // For each cycle, start a seed face on the left and right side of the cycle
      // for (auto he_cycle : cycles){
      //   auto he = he_cycle[0];
      //   auto f_left = he.face();
      //   auto f_right = he.twin().face();

      //   // Convert the cycle of edges into an unordered set of banned crossing edges.
      //   std::unordered_set<Edge> banned_crossings;

      //   for (auto he : he_cycle) {
      //     banned_crossings.insert(he.edge());
      //   }

      //   // Run BFS on the faces, making sure not to cross the cycle
      //   // For each face, we check the containing halfedges.
      //   // If the half edge is not in the banned 


      // }
      
      // std::vector<std::pair<float,std::vector<Halfedge>>> cycle_pairs;

      std::vector<bool> local_min_cycle(cycles.size());

      auto test_cycle = cycles[30];
      std::vector<std::array<double, 3>> cyclefaces(nm->mesh->nFaces(), {0.0,0.0,0.0});

      for (auto he : test_cycle) {
        Face f = he.face();
        cyclefaces[f.getIndex()] = {0.0, 1.0, 0.0};
      }

      for (auto face : Y.second.adjacentFaces()) {
        cyclefaces[face.getIndex()] = {1.0, 0.0, 0.0};
      }

      for (auto face : Z.second.adjacentFaces()) {
        cyclefaces[face.getIndex()] = {0.0, 0.0, 1.0};
      }
      auto surf = polyscope::getSurfaceMesh("human_tri");
      surf->addFaceColorQuantity("cyclefaces", cyclefaces);

      // for (int i = 1; i < cycles.size()-1; i++) {
      //   if (cycle_lens[i] >= cycle_lens[i+1] && cycle_lens[i] < cycle_lens[i-1]) {
      //     local_min_cycle[i] = true;
      //   }
      // }
      local_min_cycle[30] = true;
      // local_min_cycle[0] = true;
      // local_min_cycle[152] = true;
      // local_min_cycle[218] = true;
      // local_min_cycle[258] = true;
      std::vector<glm::vec3> output_ve;
      std::vector<std::array<size_t, 2>>   output_ed;


      // for (int i =0; i < cycles.size(); i++) {
      //   if (local_min_cycle[i]){
      //     for (auto he : cycles[i]){
      //       ecolors[he.edge().getIndex()] = {1.0,0.0,0.0};
      //     }
      //   }
      // }

      int base_count = 0;
      for (int i = 0; i < cycles.size(); i++) {
        if (local_min_cycle[i]) {
          for (size_t j = 0 ; j < cycles[i].size(); j++) {
            Halfedge he = cycles[i][j];
            Vector3 vertdat = nm->geometry->vertexPositions[he.tailVertex()];
            output_ve.push_back({vertdat.x, vertdat.y, vertdat.z});
            // output_ve.push_back(nm->geometry .tailVertex().getIndex);
            // std::cout << j << ", " << (j+1) % 37 << std::endl;
            output_ed.push_back({base_count + j, base_count +((j+1) % (cycles[i].size()))});
            // ecolors[he.edge().getIndex()] = {1.0, 0.0, 0.0};
          }
          base_count += cycles[i].size();
        }
      }
      auto curve2 = polyscope::registerCurveNetwork("cyclecurve", output_ve, output_ed);
      curve2->setColor({1.0,0.0,0.0});
      // std::cout << output_ve.size() << std::endl;
      // std::cout << output_ed.size() << std::endl;


      // for (auto a : local_min_cycle) {
      //   std::cout << a << ", ";
      // }
      // std::cout << std::endl;
      curve->addEdgeColorQuantity("path", ecolors);
}

void findLeaders(std::unique_ptr<NeckModel> & nm) {
    auto path = nm->sssp(nm->_source);
      auto prev = path.first;
      auto dists = path.second;

      // Find all local candidates
      std::vector<std::pair<float, Vertex>> candidates;

      for (Vertex v : nm->mesh->vertices()){
          bool is_candidate = true;
          for (Vertex u : v.adjacentVertices()) {
            if (dists[v] < dists[u]){
              is_candidate = false;
              break;
            }
          }
          if (is_candidate){
            candidates.push_back({dists[v],v});
          }
      }
      std::cout << "# of Candidates/Verts " << candidates.size() << "/" << nm->mesh->nVertices() << std::endl;

      // Sort candidates by distance
      struct{
        bool operator()(std::pair<float, Vertex> a, std::pair<float, Vertex> b) const {return a.first > b.first;}
      } pair_ge;

      std::sort(candidates.begin(), candidates.end(), pair_ge);
      for (size_t i = 0; i < candidates.size(); i++){
        std::cout << candidates[i].first << ", " << candidates[i].second << std::endl;
      }

      // Remove All Candidates within r-hops
      int r = 5;
      typedef std::pair<int, Vertex> VHop;
      for (size_t i = 0; i < candidates.size(); i++){
        std::queue<VHop> q;
        std::unordered_set<Vertex> visited;
        Vertex s = candidates[i].second;
        q.push({0,s});
        visited.insert(s);

        while (!q.empty()){
          auto hop  = q.front().first;
          auto curr = q.front().second;
          q.pop();
          if (hop + 1 == r){
            continue;
          }
          auto it = candidates.begin();
          for (; it < candidates.end(); it++){
              if ((*it).second == curr && curr != s){
                candidates.erase(it);
                break;
              }
          }

          for (Vertex v :  curr.adjacentVertices()) {
            int rp = hop + 1;
            if (visited.find(v) == visited.end()){
              visited.insert(v);
              q.push({rp, v});
            }
          }
        }
      }

      std::cout << "# of Candidates Post Trim " << candidates.size() << std::endl;
      // Mark vertices on structure
      std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0,0.0,0.0});

      for (size_t i = 0; i < candidates.size(); i++){
        vcolors[candidates[i].second.getIndex()] = {1.0, 0.0, 0.0};
      }

      std::vector<glm::vec3> pcloud;
      for (auto x : candidates) {
        Vector3 vertdat = nm->geometry->vertexPositions[x.second.getIndex()];
        pcloud.push_back({vertdat.x, vertdat.y, vertdat.z});
      }
      Vector3 srcdat = nm->geometry->vertexPositions[nm->_source.getIndex()];
      pcloud.push_back({srcdat.x, srcdat.y, srcdat.z});

      auto gen_pcloud = polyscope::registerPointCloud("critpts", pcloud);

      auto curve = polyscope::getCurveNetwork("curve");
      curve->addNodeColorQuantity("leaders", vcolors);
}

void salientLine(std::unique_ptr<NeckModel> & nm){
      auto path = nm->st_dijkstras(nm->_source, nm->_anti_source);
      auto prev = path.first;
      auto dists = path.second;
      

      // Mark edges that are on this path
      std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
      Vertex v = nm->_source;
      Vertex cur = nm->_anti_source;

      std::vector<Halfedge> he_path;

      while (cur != v){
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
      while (test_len > len /6.0){
        index++;
        test_len = dists[he_path[index].tipVertex()]; 
      }

      // // Find cycle at n/2
      // size_t len = he_path.size();
      size_t midpt = index+3;

      std::cout << "midpt/len: " << midpt << " midpt/ind: " << he_path.size()/2 << std::endl;

      Halfedge he_mid = he_path[midpt];
      Halfedge he_pred = he_path[midpt+1];

      Vertex a = he_mid.tailVertex();
      // Vertex b = he_pred.tipVertex();

      // get list of edges on either side of cycle at POI
      // Sides are arbitrary, it switches once you encounter the hedges on the path.

      std::set<Halfedge> left, right;
      int side = 0;
      for (Halfedge he : a.outgoingHalfedges()){
        if (he == he_mid || he == he_pred.twin()){
          side = !side;
          continue;
        }
        if (side){
          left.insert(he);
        } else {
          right.insert(he);
        }
      }
      // run st dijkstra's but ban any node on the path, and return halfedges on the side you started with

      // make a banned set
      // add all nodes on the o.g. path to it
      std::set<Vertex> banned_crossing;
      for (auto he : he_path){
        banned_crossing.insert(he.tipVertex());
        banned_crossing.insert(nm->_source);
      }
      
      // pick a side, enqueue all outgoing verts
      VertexData<Halfedge> prev_c(*(nm->mesh));
      VertexData<float> dists_c(*(nm->mesh), std::numeric_limits<float>::infinity());
      std::priority_queue<vPair, std::vector<vPair>, std::greater<vPair>> pq;
      for (auto he : left){
        float len = nm->geometry->edgeLengths[he.edge()];
        Vertex vert = he.tipVertex();
        pq.push(std::make_pair(len, vert));
        dists_c[vert] = len;
      }
      // run dijksta's until it reaches the same vertex again, banning the same side's twin hedges.
      while (!pq.empty()){
        auto curr = pq.top().second;
        if (curr == a){
          break;
        }
        pq.pop();

        for (Halfedge he : curr.outgoingHalfedges()){
          // don't allow instant return.
          if (left.find(he.twin()) != left.end()){
            continue;
          }
          Vertex v = he.twin().vertex();
          if (v != a && banned_crossing.find(v) != banned_crossing.end()){
            continue; 
          }
          float newdist = dists_c[curr] + nm->geometry->edgeLengths[he.edge()];

          if (newdist < dists_c[v]) {
            dists_c[v] = newdist;
            prev_c[v] = he;
            pq.push(std::make_pair(newdist, v));
          }
        }
      }
      // recover the cycle from the point
      std::vector<std::array<double, 3>> ecycle(nm->mesh->nEdges(), {0.0,0.0,0.0});
      cur = a;

      std::vector<Halfedge> he_cycle;

      while (true){
        
        Halfedge he = prev_c[cur];
        if (he == Halfedge()){
          break;
        }
        // std::cout << he << std::endl;
        he_cycle.push_back(he);
        Edge e = he.edge();
        ecycle[e.getIndex()] = {1.0, 0.0, 0.0};
        cur = he.tailVertex();
      }
      auto fin_edge = nm->get_edge(a, cur);
      if (fin_edge.halfedge().tipVertex() == a){
        he_cycle.push_back(fin_edge.halfedge());
      } else {he_cycle.push_back(fin_edge.halfedge().twin());}
      ecycle[fin_edge.getIndex()] = {1.0, 0.0, 0.0};
      curve->addEdgeColorQuantity("cycle", ecycle);
      std::cout << he_cycle.size() << std::endl;
      // Recurse above and below
}
