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

        // Compute the shortest path from s to all other vertices and store in _dists, _prev
        void compute_shortest_paths(Vertex s);
        // Compute the necessary delete edge events and store in _events
        void compute_delete_events();
        // Compute the neck ratio and store in neck_ratio* containers
        void sweepline_dist_process();
        // Clean up variables to reuse algorithm w/ different source.
        void clean_sweepline_vars();
        // Given a set of faces on the wavefront (presumably a cycle), determine the edges on the boundary of the covered faces.
        std::set<Edge> determine_cycle_edgeset(std::set<Face> &faces);
        // Given a set of edges which presumably make a cycle, determine the nearest one to source and returned an ordered list of edges.
        std::vector<Edge> determine_single_cycle(std::set<Edge> edgeSet);
        // Optimize a cycle by computing shortest distance between random adjacent sectors and taking the most aggressive improvement.

        // Helpers

        /** Compute the most ideal candidate cut for out of all computed neck cut ratios
            This trims the first and last markers for neck cut (as they are usually extremes)
            then computes the cut which has the furthest distance from the next cut in the list. */
        std::set<Face>& compute_candidate_cut();

        // Vars

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
