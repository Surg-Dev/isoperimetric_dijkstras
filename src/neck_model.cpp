#include "neck_model.hpp"

using namespace geometrycentral;
using namespace geometrycentral::surface;


// Constructors
NeckModel::NeckModel(std::string filename){
    std::tie(mesh, geometry) = readManifoldSurfaceMesh(filename);
    _source = mesh->vertex(0);
    geometry->requireVertexTangentBasis();

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
    std::set<Halfedge> wavefront;

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
      _events.pop();
      // Track distance covered from last event.
      total_dist_covered += (curr_dist - prev_dist) * wavefront_size;
      prev_dist = curr_dist;

      // If the event is a middle edge, we need to remove both from wavefront.
      _visited[e] = true;
      if (_middle[e]){
        wavefront.erase(e.halfedge());
        wavefront.erase(e.halfedge().twin());
        wavefront_size -= 2;
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
