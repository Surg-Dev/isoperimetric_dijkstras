  // if (ImGui::Button("do work")) {
  //   nm->sweepline_dist_process();
  //   cutidxs = nm->neck_ratio_faces_list.size();
  // }

  // if (ImGui::Button("Visualize")){
  //   std::vector<std::array<double, 3>> fcolors(nm->mesh->nFaces(), {.5,.5,.5});
  //   // auto faceset = nm->neck_ratio_faces_list[cut_index];
  //   best_cut = nm->compute_candidate_cut();
  //   for (Face f : best_cut){
  //     fcolors[f.getIndex()] = {1.0, 0.0, 0.0};
  //   }
  //   psMesh->addFaceColorQuantity("fcolor", fcolors);
  // }

  // if (ImGui::Button("Visualize Selected")){
  //   std::vector<std::array<double, 3>> fcolors(nm->mesh->nFaces(), {.5,.5,.5});
  //   best_cut = nm->get_candidate_cut(cut_index);
  //   for (Face f : best_cut){
  //     fcolors[f.getIndex()] = {1.0, 0.0, 0.0};
  //   }
  //   psMesh->addFaceColorQuantity("fcolor", fcolors);

  // }

  // if (ImGui::Button("Set Source")){
  //   // Get selection from polyscope
  //   auto x = polyscope::pick::getSelection();
  //   polyscope::Structure * strc = x.first;
  //   auto ind = x.second;
  //   if (ind < psMesh->nVertices()){
  //     nm->_source = nm->mesh->vertex(ind);
  //     std::vector<std::array<double, 3>> vcolors(nm->mesh->nVertices(), {0.0, 1.0, 0.0});
  //     vcolors[nm->_source.getIndex()] = {1.0, 0.0, 0.0};
  //     psMesh->addVertexColorQuantity("vcolor", vcolors);
  //   }
  // }

  // if (ImGui::Button("Get Dijkstra Cyc from Edge")){
  //   auto x = polyscope::pick::getSelection();
  //   polyscope::Structure * strc = x.first;
    
  //   // auto ind = x.second;
  //   // std::cout << strc->typeName() << " " << ind << std::endl;
  //   // std::cout << nm->mesh->nEdges() << std::endl;
  //   // std::cout << nm->mesh->nVertices() << std::endl;
  //   // std::cout << psMesh->nEdges() << std::endl;
  //   // std::cout << psMesh->nVertices() << std::endl;
  //   // std::cout << psMesh->vertexDataSize << " " << psMesh->edgeDataSize << " " << psMesh->faceDataSize << " " << psMesh->halfedgeDataSize << std::endl;
  //   auto ind = x.second- (psMesh->vertexDataSize + psMesh->edgeDataSize + psMesh->faceDataSize);
  //   // if (ind < psMesh->nEdges()){
  //     auto e = nm->mesh->halfedge(ind).edge();
  //     // auto e = nm->mesh->edge(ind);
  //     auto v1 = e.halfedge().vertex();
  //     auto v2 = e.halfedge().twin().vertex();

  //     Vertex curr = v1;

  //     std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
  //     ecolors[ e.getIndex() ] = {1.0, 0.0, 0.0};
  //     while(curr != nm->_source){
  //       auto prev = nm->_prev[curr];

  //       e = nm->get_edge(prev, curr);

  //       ecolors[e.getIndex()] = {1.0, 0.0, 0.0};
  //       curr = nm->_prev[curr];
  //     }
  //     curr = v2;
  //     while(curr != nm->_source){
  //       auto prev = nm->_prev[curr];

  //       e = nm->get_edge(prev, curr);

  //       ecolors[e.getIndex()] = {1.0, 0.0, 0.0};
  //       curr = nm->_prev[curr];
  //     }
  //     auto curve = polyscope::getCurveNetwork("curve");
  //     curve->addEdgeColorQuantity("cycle", ecolors);
  //   // }
  // }

  // if (ImGui::Checkbox("Restricted Search", &nm->_restricted_search)){}

  // if (ImGui::Button("Show Edge Cycle")){
  //   std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
  //   // auto faceSet = nm->compute_candidate_cut();
  //   auto edgeSet = nm->determine_cycle_edgeset(best_cut);
  //   auto cycle = nm->determine_single_cycle(edgeSet);
  //   best_cycle = nm->orient_cycle(cycle);
  //   std::cout << std::endl;
  //   for (Halfedge e : best_cycle){
  //     ecolors[e.edge().getIndex()] = {1.0, 0.0, 0.0};
  //   }
  //   auto curve = polyscope::getCurveNetwork("curve");
  //   curve->addEdgeColorQuantity("ecolor", ecolors);
  //   // psMesh->addEdgeColorQuantity("ecolor", ecolors);
  // }

  // if (ImGui::Button("Improve")){
  //   std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
  //   for (size_t i = 0; i < param1; i++){
  //       best_cycle = nm->optimize_oriented_cycle_single(best_cycle);
  //   }
  //   // std::cout << std::endl;
  //   for (Halfedge e : best_cycle){
  //     ecolors[e.edge().getIndex()] = {1.0, 0.0, 0.0};
  //   }
  //   auto curve = polyscope::getCurveNetwork("curve");
  //   curve->addEdgeColorQuantity("ecolor", ecolors);
  //   // psMesh->addEdgeColorQuantity("ecolor", ecolors);
  // }
  // ImGui::SliderInt("Improve Iters", &param1, 1, 40);
  // ImGui::SliderInt("Cut Index", &cut_index, 0, cutidxs);

  // if (ImGui::Button("Generate Middle Edges")){
  //   std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
  //   for (Edge e : nm->mesh->edges()){
  //     if (nm->_middle_color[e]){
  //       ecolors[e.getIndex()] = {1.0, 0.0, 1.0};
  //     }
  //   }
  //   auto curve = polyscope::getCurveNetwork("curve");
  //   curve->addEdgeColorQuantity("middle", ecolors);
  // }
  // if (ImGui::Button("Full Scan")){
  //   nm->do_everything();
  //   cycles_made = nm->good_cycles.size();

  // }

  // if (ImGui::Button("Visualize Good Cycle")){
  //   std::vector<std::array<double, 3>> ecolors(nm->mesh->nEdges(), {0.0,0.0,0.0});
  //   for (Edge e : nm->good_cycles[cycle_sel]){
  //     ecolors[e.getIndex()] = {0.0,0.0,1.0};
  //     prints(e);
  //   }
  //   auto curve = polyscope::getCurveNetwork("curve");
  //   curve->addEdgeColorQuantity("good_cycle", ecolors);
  // }
  // ImGui::SliderInt("Cycle View", &cycle_sel, 0, cycles_made);