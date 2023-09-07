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

}
