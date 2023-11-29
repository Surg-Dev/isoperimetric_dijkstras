#include "neck_model.hpp"

using namespace geometrycentral;
using namespace geometrycentral::surface;


// Constructors
NeckModel::NeckModel(std::string filename){
    std::tie(mesh, geometry) = readManifoldSurfaceMesh(filename);
    _source = mesh->vertex(0);
    geometry->requireVertexTangentBasis();
    geometry->requireFaceAreas();

    for (Edge e : mesh->edges()){
        sum_mesh_dist += geometry->edgeLengths[e];
    }
    _dists = VertexData<float>(*mesh, 0.0);
    _prev = VertexData<Vertex>(*mesh);
    _middle = EdgeData<bool>(*mesh, false);
    _visited = EdgeData<bool>(*mesh, false);
}

void NeckModel::compute_shortest_paths(Vertex s){
  VertexData<Vertex> prev(*mesh);
  VertexData<float> dists(*mesh);
  std::priority_queue<vPair, std::vector<vPair>, std::greater<vPair>> pq;
  pq.push(std::make_pair(0.0, s));
  for (Vertex v : mesh->vertices()){
    dists[v] = std::numeric_limits<float>::infinity();
  }
  dists[s] = 0.0;
  while (!pq.empty()){
    auto curr = pq.top().second;
    pq.pop();

    for (Halfedge he : curr.outgoingHalfedges()){
      Vertex v = he.twin().vertex();
      float alt = dists[curr] + geometry->edgeLengths[he.edge()];
      if (alt < dists[v]){
        dists[v] = alt;
        prev[v] = curr;
        pq.push(std::make_pair(alt, v));
      }
    }
  }

    _dists = dists;
    _prev = prev;
}

void NeckModel::compute_delete_events(){
  // Initialize data structures
  _middle = EdgeData<bool>(*mesh, false);
  _visited = EdgeData<bool>(*mesh, false);
  
  _events = eventQueue();

  // Comute 
  for (Edge e : mesh->edges()){
      Vertex v1 = e.halfedge().vertex();
      Vertex v2 = e.halfedge().twin().vertex();
      float dist_v1 = _dists[v1];
      float dist_v2 = _dists[v2];
      float length = geometry->edgeLengths[e];
      // "Orient" v1 to be the closer vertex
      if (dist_v1 > dist_v2){
          std::swap(v1, v2);
          std::swap(dist_v1, dist_v2);
      }

      // If edge points don't follow each other, then the wavefront collapses in the middle of the edge.
      // The computation is better understood as (len  - (d2 - d1))/2.0 + d2
      if (_prev[v1] != v2 && _prev[v2] != v1){
          _events.push(ePair( (dist_v1 + dist_v2 + length)/2, e));
          _middle[e] = true;
      }
      // Otherwise, just push the deletion event when v2 is reached.
      else if (_prev[v2] == v1){
          _events.push(ePair(dist_v2, e));
      }
      // Something went wrong if we fall down here, but it's probably OK.
  }
}

void NeckModel::sweepline_dist_process(){
  // Set up dists and events
  compute_shortest_paths(_source);
  compute_delete_events();
  // Clear/init any hold variables.
  clean_sweepline_vars();
  _middle_area = EdgeData<double>(*mesh, 0.0);
  _middle_faces = EdgeData<size_t>(*mesh, 0);
  _middle_color = EdgeData<bool>(*mesh, false);
  std::set<Halfedge> wavefront;
  std::set<Edge> processed;

  // Queue all halfedges out of source
  for (Halfedge he : _source.outgoingHalfedges()){
      wavefront.insert(he);
      ++wavefront_size;
  }

  float curr_dist = 0;
  float prev_dist = 0;

  // Process the wavefront
  while (wavefront_size > 0){
    // Get the next event.
    ePair event = _events.top();
    float curr_dist = event.first;
    Edge e = event.second;
    if (_restricted_search){
      processed.insert(e);
    } else{
      if (_middle[e] == false){
        processed.insert(e);
      }
    }
    // if (_middle[e] == false){
      
    // }

    _events.pop();
    // Track distance covered from last event.
    total_dist_covered += (curr_dist - prev_dist) * wavefront_size;
    prev_dist = curr_dist;

    // If the event is a middle edge, we need to remove both from wavefront.
    _visited[e] = true;
    if (_middle[e]){
      _middle_color[e] = true;
      wavefront.erase(e.halfedge());
      wavefront.erase(e.halfedge().twin());
      wavefront_size -= 2;

      // TODO: clean
      // Do BFS on either face of the middle edge, and count the two regions contained.
      // Dont cross any edge in processed.
      std::set<Face> visited;
      // Get the two faces
      Face f1 = e.halfedge().face();
      Face f2 = e.halfedge().twin().face();

      size_t f1size = 0;
      size_t f2size = 0;

      double f1area = 0.0;
      double f2area = 0.0;

      std::queue<Face> q;
      std::queue<Face> q2;
      q.push(f1);
      visited.insert(f1);
      q2.push(f2);
      visited.insert(f2);
      f1size++;
      f2size++;

      // Face f1test = q.front();
      // q.pop();

      // Parallel BFS
      std::queue<Face>* qptr = &q;
      size_t* face_size = &f1size;
      double* area_covered = &f1area;
      size_t iters = 10;

      while (!qptr->empty()){
        Face f = qptr->front();
        qptr->pop();
        (*area_covered)+=geometry->faceAreas[f];
        for (Halfedge he : f.adjacentHalfedges()){
          // Don't allow crossing of processed edges
          if (processed.find(he.edge()) == processed.end()){
            Face f_twin = he.twin().face();
            if (visited.find(f_twin) == visited.end()){
              qptr->push(f_twin);
              visited.insert(f_twin);
              (*face_size)++;
            }
          }
        }
        iters--;
        if (iters==0){
          qptr = (qptr==&q) ? &q2 : &q;
          face_size = (face_size==&f1size) ? &f2size : &f1size;
          area_covered = (face_size==&f1size) ? &f2area : &f1area;
          iters=10;
        }
      }

      // for (Halfedge he : f1test.adjacentHalfedges()){
      //   // Don't allow crossing of processed edges
      //   if (processed.find(he.edge()) == processed.end()){
      //     Face f_twin = he.twin().face();
      //     if (visited.find(f_twin) == visited.end()){
      //       q.push(f_twin);
      //       visited.insert(f_twin);
      //       f1size++;
      //     }
      //   }
      // }

      // Face f2test = q2.front();
      // q2.pop();

      // for (Halfedge he : f2test.adjacentHalfedges()){
      //   // Don't allow crossing of processed edges
      //   if (processed.find(he.edge()) == processed.end()){
      //     Face f_twin = he.twin().face();
      //     if (visited.find(f_twin) == visited.end()){
      //       q2.push(f_twin);
      //       visited.insert(f_twin);
      //       f2size++;
      //     }
      //   }
      // }

      // if (!q.empty() and !q2.empty()){
      //   while (!q.empty()){
      //     Face f = q.front();
      //     q.pop();

      //     for (Halfedge he : f.adjacentHalfedges()){
      //       // Don't allow crossing of processed edges
      //       if (processed.find(he.edge()) == processed.end()){
      //         Face f_twin = he.twin().face();
      //         if (visited.find(f_twin) == visited.end()){
      //           q.push(f_twin);
      //           visited.insert(f_twin);
      //           f1size++;
      //         }
      //       }
      //     }            
      //   }
      //   while (!q2.empty()){
      //       Face f = q2.front();
      //       q2.pop();
      //       for (Halfedge he : f.adjacentHalfedges()){
      //         // Don't allow crossing of processed edges
      //         if (processed.find(he.edge()) == processed.end()){
      //           Face f_twin = he.twin().face();
      //           if (visited.find(f_twin) == visited.end()){
      //             visited.insert(f_twin);
      //             q2.push(f_twin);
      //             f2size++;
      //           }
      //         }
      //       }
      //   } 
      // }
    int min_size = 10;
    if (!_restricted_search){
      min_size = 1000;
    }

    if (std::min(f1size,f2size) > min_size){
      // std::cout << "Found big cycle? Region 1 size: " << f1size << " Region 2 size: " << f2size << std::endl;
      _middle_area[e] = std::min(f1area,f2area);
    } else{
      _middle_color[e] = false;
    }




    } else {
      // Otherwise handle deletion of single edge.
      wavefront_size -= 1;
      // We don't know which halfedge might be in the wavefront.
      wavefront.erase(e.halfedge());
      wavefront.erase(e.halfedge().twin());

      Vertex v1 = e.halfedge().vertex();
      Vertex v2 = e.halfedge().twin().vertex();
      float d1 = _dists[v1];
      float d2 = _dists[v2];

      // Orient edge calcs so that d1 < d2
      if (d1 > d2){
        std::swap(v1, v2);
        std::swap(d1, d2);
      }

      // Add new edges to wavefront
      for (Halfedge he : v2.outgoingHalfedges()){
        if (!_visited[he.edge()]){
          wavefront.insert(he);
          ++wavefront_size;
        }
      }

    }

    // If we've reached the end of the wavefront, break now.
    if (wavefront_size == 0){
      break;
    }

    // Compute neck ratio
    double remaining_dist = (sum_mesh_dist - total_dist_covered);
    if (remaining_dist < 0) remaining_dist = 0;
    double neck_cut_ratio_potential = (double)(total_dist_covered >= remaining_dist ? remaining_dist : total_dist_covered) / (double)(wavefront_size*wavefront_size);

    // Update neck ratio if improved.
    if (std::max(neck_cut_ratio_potential, neck_ratio) == neck_cut_ratio_potential){
      // qprint("New Neck Ratio: " + std::to_string(neck_cut_ratio_potential));
      neck_ratio = neck_cut_ratio_potential;
      neck_ratio_pos = curr_dist;

      std::set<Face> faces;
      for (auto h : wavefront){
        faces.insert(h.face());
        faces.insert(h.twin().face());
      }
      neck_ratio_faces_list.push_back(faces);

      // Compute cycle and push back cycle
      
      // Push back dist marker
      neck_ratios_pos_list.push_back(curr_dist);
    }
  }
  // 
    
}

void NeckModel::clean_sweepline_vars(){
    total_dist_covered = 0;
    wavefront_size = 0;
    neck_ratio = 0;
    neck_ratio_pos = 0;
    neck_ratios_pos_list.clear();
    neck_ratio_vertices_list.clear();
    neck_ratio_cycle_list.clear();
    neck_ratio_faces_list.clear();
}

std::set<Face>& NeckModel::compute_candidate_cut(){
  double max_gap = 0;
  size_t index = 0;
  qprint( "Neck Ratios: " + std::to_string(neck_ratios_pos_list.size()));
  for (size_t i = 0; i < neck_ratios_pos_list.size()-1; i++){
    double start = neck_ratios_pos_list[i];
    double end = neck_ratios_pos_list[i+1];
    if (end - start > max_gap){
      max_gap = end - start;
      index = i;
    }
  }
  qprint("Cut Index: " + std::to_string(index));
  return neck_ratio_faces_list[index];
}

std::set<Face>& NeckModel::get_candidate_cut(size_t cut_index){
  return neck_ratio_faces_list[cut_index];
}

std::set<Edge> NeckModel::determine_cycle_edgeset(std::set<Face> &faces){
  std::set<Edge> edgeSet;
  for (Face f : faces){
    for (Halfedge he : f.adjacentHalfedges()){
      // Get twin of halfedge
      Halfedge he_twin = he.twin();
      // Get twin face
      Face f_twin = he_twin.face();
      // If twin face is not in the face set, then this edge is on the boundary cycle.
      if (faces.find(f_twin) == faces.end()){
        edgeSet.insert(he.edge());
      }
    }
  }
  return edgeSet;
}

std::vector<Edge> NeckModel::determine_single_cycle(std::set<Edge> edgeSet){
  // Determine the minimum distance edge from the source
  float min_dist = std::numeric_limits<float>::infinity();
  Edge min_edge;
  for (Edge e : edgeSet){
    // Get vertices from edge
    Vertex v1 = e.halfedge().vertex();
    Vertex v2 = e.halfedge().twin().vertex();
    // Get distances from vertices
    float d1 = _dists[v1];
    float d2 = _dists[v2];
    // Take min and check against current min
    float min = std::min(d1, d2);
    if (min < min_dist){
      min_dist = min;
      min_edge = e;
    }
  }

  // Find adjacent edges until complete cycle
  std::vector<Edge> cycle;
  // Insert the min edge as the first edge in the cycle
  cycle.push_back(min_edge);
  // Remove min_edge from edgeSet
  edgeSet.erase(min_edge);

  // Find the next edge in the cycle
  Edge curr_edge = min_edge;
  // If we see the min edge we either:
  // Just left it, or its the last edge to encounter, and use as our stopping point
  bool saw_min_edge = false;

  while (!saw_min_edge){
    // If we didnt find the next edge, but saw the min edge, then we are done.
    bool found_next_edge = false;
    // We can't guarantee anything about the orientation of the edge, so we need to check both vertices
    // Effectively, whatever edge we encounter first, we recover the whole cycle in that direction.
    for (Vertex v : curr_edge.adjacentVertices()){
      for (Edge e : v.adjacentEdges()){
        // Mark if it was the min edge
        if (e==min_edge){
          saw_min_edge = true;
        }
        // If edge is in edgeSet, then it is part of the cycle
        if (edgeSet.find(e) != edgeSet.end()){
          cycle.push_back(e);
          edgeSet.erase(e);
          found_next_edge = true;
          // Update curr edge to be e and continue
          curr_edge = e;
          break;
        }
      }
      if (found_next_edge) break;
    }

    if (found_next_edge and saw_min_edge){
      saw_min_edge = false;
    }
  }
  return cycle;
}

std::vector<Halfedge> NeckModel::orient_cycle(std::vector<Edge> edgeVec){
  std::vector<Halfedge> heVec;
  for (size_t i = 0; i < edgeVec.size(); i++){
    Edge e = edgeVec[i];

    // Get Halfedges
    Halfedge he1 = e.halfedge();
    Halfedge he2 = e.halfedge().twin();

    // Get next edge, or if this is the last edge, get the first edge
    Edge next_e = edgeVec[(i+1)%edgeVec.size()];

    // Get next Halfedges
    Halfedge next_he1 = next_e.halfedge();
    Halfedge next_he2 = next_e.halfedge().twin();

    if (he2.vertex() == next_he1.vertex()){
      heVec.push_back(he1);
    } else if (he1.vertex() == next_he2.vertex()){
      heVec.push_back(he2);
    } else if (he1.vertex() == next_he1.vertex()){
      heVec.push_back(he2);
    } else if (he2.vertex() == next_he2.vertex()){
      heVec.push_back(he1);
    } else {
      qprint("Something went wrong in orient_cycle");
    }
  }
  return heVec;
}

// std::vector<Halfedge> NeckModel::optimize_oriented_cycle(std::vector<Halfedge> oriented_cycle){
//   // Compute current cycle length
//   float curr_length = 0;
//   for (Halfedge he : oriented_cycle){
//     curr_length += geometry->edgeLengths[he.edge()];
//   }

//   // Seed rand
//   srand(time(NULL));

//   // pick a random section of the cycle, 1/6th of the total edges:
//   size_t start = rand() % oriented_cycle.size();
//   size_t end = (start + oriented_cycle.size()/6) % oriented_cycle.size();

//   // Pick the next adjecent region, after cycle:
//   size_t next_start = (end + 1) % oriented_cycle.size();
//   size_t next_end = (next_start + oriented_cycle.size()/6) % oriented_cycle.size();

//   // For each half edge in the first region, compute dijkstras to each halfedge in the second region
//   // and find the shortest path. If the shortest path is shorter than the current distance between both halfedges in the cycle, take it
//   // and update the cycle.
//   for (size_t i = start; i < end; i++){
//     if (i == oriented_cycle.size()) i = 0 ;
//     Halfedge he1 = oriented_cycle[i];
//     for (size_t j = next_start; j < next_end; j++){
//       if (j == oriented_cycle.size()) j = 0;
//       Halfedge he2 = oriented_cycle[j];
//       // Compute shortest path between he1 and he2
//       Vertex v1 = he1.vertex();
//       Vertex v2 = he2.vertex();

//       auto retdata = this->st_dijkstras(v1, v2);

      
//       // If the distance is shorter than the current distance between he1 and he2, then update the cycle
//       if (dist < curr_length){
//         // Get the path from v1 to v2
//         std::vector<Halfedge> path;
//         Vertex curr = v2;
//         while (curr != v1){
//           Vertex prev = _prev[curr];
//           Halfedge he = prev.halfedge();
//           path.push_back(he);
//           curr = prev;
//         }
//         // Update the cycle
//         std::vector<Halfedge> new_cycle;
//         for (size_t k = 0; k < i; k++){
//           new_cycle.push_back(oriented_cycle[k]);
//         }
//         for (Halfedge he : path){
//           new_cycle.push_back(he);
//         }
//         for (size_t k = j; k < oriented_cycle.size(); k++){
//           new_cycle.push_back(oriented_cycle[k]);
//         }
//         oriented_cycle = new_cycle;
//         // Update the current length
//         curr_length = dist;
//       }
//     }
//   }
// }


std::vector<Halfedge> NeckModel::optimize_oriented_cycle_single(std::vector<Halfedge> oriented_cycle){
  // Compute legnth of cycle
  float curr_length = 0;
  for (Halfedge he : oriented_cycle){
    curr_length += geometry->edgeLengths[he.edge()];
  }

  // Pick a random edge
  size_t start = rand() % oriented_cycle.size();
  // Pick the edge 1/6th distance, looping
  size_t end = (start + oriented_cycle.size()/6) % oriented_cycle.size();

  // Compute st_dijkstras between the two vertices
  Vertex v1 = oriented_cycle[start].vertex();
  Vertex v2 = oriented_cycle[end].vertex();
  auto retdata = this->st_dijkstras(v1, v2);

  // Check to see if the distance is shorter than the current distance between the two vertices
  auto l_dists = retdata.second;
  auto l_prev = retdata.first;
  float dist = l_dists[v2];
  float cur_dist = 0;

  size_t i = start;
  while (i != end){
    cur_dist += geometry->edgeLengths[oriented_cycle[i].edge()];
    i++;
    if (i == oriented_cycle.size()) i = 0;
  }

  if (dist < cur_dist){
    // Get the sequence of half edges from v1 to v2 and update the cycle
    std::vector<Edge> path;
    Vertex curr = v2;
    while (curr != v1){
      // Halfedge that points *to* curr
      Halfedge prev = l_prev[curr];
      path.push_back(prev.edge());
      curr = prev.vertex();
    }
    std::reverse(path.begin(), path.end());
    size_t j = end;
    while (j != start){
      path.push_back(oriented_cycle[j].edge());
      j++;
      if (j == oriented_cycle.size()) j = 0;
    }
    return orient_cycle(path);
  }
  return oriented_cycle;
}

std::pair<VertexData<Halfedge>, VertexData<float>> NeckModel::st_dijkstras(Vertex s, Vertex t){
  VertexData<Halfedge> prev(*mesh);
  VertexData<float> dists(*mesh);
  std::priority_queue<vPair, std::vector<vPair>, std::greater<vPair>> pq;
  pq.push(std::make_pair(0.0, s));
  for (Vertex v : mesh->vertices()){
    dists[v] = std::numeric_limits<float>::infinity();
  }
  dists[s] = 0.0;
  while (!pq.empty()){
    auto curr = pq.top().second;
    if (curr == t){
      break;
    }
    pq.pop();

    for (Halfedge he : curr.outgoingHalfedges()){
      Vertex v = he.twin().vertex();
      float alt = dists[curr] + geometry->edgeLengths[he.edge()];
      if (alt < dists[v]){
        dists[v] = alt;
        prev[v] = he;
        pq.push(std::make_pair(alt, v));
      }
    }
  }
  return std::make_pair(prev, dists);
}

Edge NeckModel::get_edge(Vertex v1, Vertex v2){
  for (Halfedge e : v1.outgoingHalfedges()){
    if (e.twin().vertex() == v2){
      return e.edge();
    }
  }
  return Edge();
}

void NeckModel::do_everything(){
  //Later... get set of vertices to run restricted search on.
  // Run the restricted search to get a series of edges
  _restricted_search = true;
  sweepline_dist_process();
  std::vector<Edge> candidate_edges;

  for (Edge e : mesh->edges()){
    if (_middle_color[e]){
      candidate_edges.push_back(e);
    }
  }
   std::cout << "Number of Good Edges: " << candidate_edges.size() << std::endl;

  // for (Edge e : candidate_edges){
  //   std::cout << _middle_area[e] << std::endl;
  // }

  // For each "good" edge, run the unrestricted search and find a maximal loop which maximizes A/L^2
  std::vector<std::set<Edge>> cycles;
  _restricted_search = false;
  for (Edge e : candidate_edges){
    _source = e.firstVertex();
    sweepline_dist_process();
    double max_ratio = 0;
    Edge candidate_cycle_edge;
    for (Edge e : mesh->edges()){
      if (!_middle_color[e]) continue;

      double length = _dists[e.firstVertex()] + _dists[e.secondVertex()] + geometry->edgeLengths[e];
      double area = _middle_area[e];
      double ratio = area/(length*length);
      if (ratio > max_ratio){
        max_ratio = ratio;
        candidate_cycle_edge = e;
      }
    }
    std::cout << "Candidate Cycle Edge: " << candidate_cycle_edge << std::endl;
    std::set<Edge> cycle;
    cycle.insert(candidate_cycle_edge);
    Vertex u = candidate_cycle_edge.firstVertex();
    while (u != _source){
      Vertex v = _prev[u];
      for (Halfedge he : u.outgoingHalfedges()){
        if (he.tipVertex() == v){
          cycle.insert(he.edge());
        }
      }
      u = v;
    }
    u = candidate_cycle_edge.secondVertex();
    while (u != _source){
      Vertex v = _prev[u];
      cycle.insert(get_edge(u,v));
      u = v;
    }
    std::cout << "Cycle Size: " << cycle.size() << std::endl;
    cycles.push_back(cycle);
    // break;

  }
  std::cout << "Made cycles: " << cycles.size() << std::endl;
  good_cycles = cycles;
  // Return the set of maximal loops for each good edge.
}