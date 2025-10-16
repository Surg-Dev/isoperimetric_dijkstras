#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <glm/glm.hpp>
#include <array>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/constants.hpp>

// struct Vertex {
//     float x, y, z;
// };

// struct Edge {
//     int v1, v2; // indices into vertex array
// };


void add_sphere(
    const glm::vec3& center,
    float radius,
    size_t rings,
    size_t segments,
    std::vector<glm::vec3>& out_vertices,
    std::vector<std::array<size_t, 3>>& out_faces,
    size_t& vertex_offset)
{
    for (size_t i = 0; i <= rings; ++i) {
        float v = float(i) / rings;
        float phi = glm::pi<float>() * v;
        for (size_t j = 0; j <= segments; ++j) {
            float u = float(j) / segments;
            float theta = 2.0f * glm::pi<float>() * u;
            float x = std::cos(theta) * std::sin(phi);
            float y = std::sin(theta) * std::sin(phi);
            float z = std::cos(phi);
            glm::vec3 p = center + radius * glm::vec3(x, y, z);
            out_vertices.push_back({p.x, p.y, p.z});
        }
    }

    for (size_t i = 0; i < rings; ++i) {
        for (size_t j = 0; j < segments; ++j) {
            size_t a = vertex_offset + i * (segments + 1) + j;
            size_t b = a + segments + 1;
            size_t c = b + 1;
            size_t d = a + 1;
            out_faces.push_back({a + 1, b + 1, c + 1});
            out_faces.push_back({a + 1, c + 1, d + 1});
        }
    }

    vertex_offset += (rings + 1) * (segments + 1);
}


// generate a cylinder mesh between two points
// returns vertices and faces (as triples of indices)
void generate_cylinder_between(
    const glm::vec3& p1,
    const glm::vec3& p2,
    float radius,
    size_t segments,
    std::vector<glm::vec3>& out_vertices,
    std::vector<std::array<size_t, 3>>& out_faces,
    size_t& vertex_offset)
{
    glm::vec3 axis = p2 - p1;
    float length = glm::length(axis);
    if (length < 1e-8f) return;

    glm::vec3 dir = axis / length;

    float overlap = radius * 1.5f;
    glm::vec3 p1_ext = p1 - dir * overlap;
    glm::vec3 p2_ext = p2 + dir * overlap;
    axis = p2_ext - p1_ext;
    length = glm::length(axis);
    axis /= length;

    // find orthonormal basis (u,v,axis)
    glm::vec3 tmp = (std::abs(axis.z) < 0.99f) ? glm::vec3(0,0,1) : glm::vec3(1,0,0);
    glm::vec3 u = glm::normalize(glm::cross(axis, tmp));
    glm::vec3 v = glm::normalize(glm::cross(axis, u));

    // make circle points
    std::vector<glm::vec3> circle1, circle2;
    for (size_t i=0; i<segments; ++i) {
        float theta = 2.0f * glm::pi<float>() * i / segments;
        glm::vec3 dir = std::cos(theta)*u + std::sin(theta)*v;
        circle1.push_back(p1 + radius * dir);
        circle2.push_back(p2 + radius * dir);
    }

    // write vertices
    for (auto& c : circle1) out_vertices.push_back({c.x, c.y, c.z});
    for (auto& c : circle2) out_vertices.push_back({c.x, c.y, c.z});

    // faces: connect quads between rings
    for (size_t i=0; i<segments; ++i) {
        size_t next = (i+1) % segments;
        size_t a = vertex_offset + i;
        size_t b = vertex_offset + next;
        size_t c = vertex_offset + segments + next;
        size_t d = vertex_offset + segments + i;
        // two triangles: (a,b,c) and (a,c,d)
        out_faces.push_back({a+1, b+1, c+1});
        out_faces.push_back({a+1, c+1, d+1});
    }

    vertex_offset += 2 * segments;
}

void export_curve_network_obj(
    const std::vector<glm::vec3>& vertices,
    const std::vector<std::array<size_t, 2>>& edges,
    const std::string& filename,
    float radius = 0.005f,
    size_t segments = 24)
{
    std::vector<glm::vec3> all_vertices;
    std::vector<std::array<size_t,3>> all_faces;
    size_t vertex_offset = 0;

    for (auto& e : edges) {
        glm::vec3 p1 = vertices[e[0]];
        glm::vec3 p2 = vertices[e[1]];;
        generate_cylinder_between(p1, p2, radius, segments, all_vertices, all_faces, vertex_offset);
    }

    for (auto& v : vertices) {
        glm::vec3 pos(v.x, v.y, v.z);
        add_sphere(pos, radius, 8, 12, all_vertices, all_faces, vertex_offset);
    }

    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    for (auto& v : all_vertices)
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    for (auto& f : all_faces)
        out << "f " << f[0] << " " << f[1] << " " << f[2] << "\n";

    out.close();
    // std::cout << "✅ Saved curve network to " << filename 
    //           << " (" << all_vertices.size() << " vertices, " 
    //           << all_faces.size() << " faces)\n";
}
