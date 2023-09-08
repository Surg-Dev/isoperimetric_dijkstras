#pragma once
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include <queue>
#include <iostream>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// Float, Obj Pairs
typedef std::pair<float, Vertex> vPair;
typedef std::pair<float, Edge> ePair;

// Event Queue
typedef std::priority_queue<ePair, std::vector<ePair>, std::greater<ePair>> eventQueue;

class NeckModel{
    public:
        // Constructors
        NeckModel() = default;
        NeckModel(std::string filename);

        // Function Defs
        void compute_shortest_paths(Vertex s);
        void compute_delete_events();
        void sweepline_dist_process();
        void clean_sweepline_vars();

        // Helpers
        std::set<Face>& compute_candidate_cut();

        // Mesh Variables
        std::unique_ptr<ManifoldSurfaceMesh> mesh; // The mesh data structure
        std::unique_ptr<VertexPositionGeometry> geometry; // The geometry data structure
        float sum_mesh_dist = 0; // Sum of all mesh edge lengths

        // Wavefront Init Variables
        Vertex _source; // Source vertex to run wavefront from

        // Shortest Path Props
        VertexData<float> _dists; //Vertex Dict of shortest path dists from _source
        VertexData<Vertex> _prev; //Vertex Dict of previous vertex in shortest path from _source

        // Delete Events
        EdgeData<bool> _middle; // Edge dict of where wavefront collapses in an edge
        EdgeData<bool> _visited; // Edge dict of whether an edge has been visited
        eventQueue _events; // Total dict of events to process edges

        // Neck Ratio Values
        float total_dist_covered = 0;  // Total distance covered by wavefront at any given time
        long wavefront_size = 0; // Number of edges in wavefront at any given time
        double neck_ratio = 0; // Maximum neck ratio during wavefront growth
        float neck_ratio_pos = 0; //Distance along wavefront where neck ratio occurs

        // Neck Ratio Containers
        std::vector<double> neck_ratios_pos_list; // List of neck ratios
        std::vector<std::set<Vertex>> neck_ratio_vertices_list; // List of vertices that make up the cycle at a max neck ratio
        std::vector<std::set<Edge>> neck_ratio_cycle_list; // List of edges that make up the cycle at a max neck ratio
        std::vector<std::set<Face>> neck_ratio_faces_list; // List of faces that make up the cycle at a max neck ratio

};


// Random Utils
inline void qprint(std::string printstr){
    std::cout << printstr << std::endl;
}
