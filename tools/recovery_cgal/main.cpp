// Offline only. No repairs, approximations, hardware, or production admission.
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/version.h>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using Mesh = CGAL::Surface_mesh<K::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
using Tree = CGAL::AABB_tree<CGAL::AABB_traits_3<K, Primitive>>;
struct Cell { K::Point_3 center; K::FT radius; };

// Hexadecimal binary64 tokens embed the exact input doubles into EPECK.
double number() {
    std::string token;
    if (!(std::cin >> token) || token.size() > 64) throw std::runtime_error("bad_number");
    size_t used = 0;
    double value = std::stod(token, &used);
    if (used != token.size() || !std::isfinite(value)) throw std::runtime_error("bad_number");
    return value;
}
size_t count(size_t maximum) {
    std::string token;
    if (!(std::cin >> token) || token.empty() || token.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("bad_count");
    auto result = std::stoull(token);
    if (result > maximum) throw std::runtime_error("count_limit");
    return result;
}

int main() {
    auto started = std::chrono::steady_clock::now();
    std::vector<Cell> cells;
    std::string reason = "invalid_input";
    size_t components = 0;
    try {
        std::string magic;
        std::cin >> magic;
        if (magic != "recovery-cgal-v1") throw std::runtime_error("protocol_version");
        const size_t nf = count(250000), nc = count(100000);
        if (nf == 0) throw std::runtime_error("empty_mesh");
        std::array<K::FT,3> scale;
        for (auto& v : scale) { v = K::FT(number()); if (v <= 0) throw std::runtime_error("invalid_scale"); }
        Mesh mesh;
        std::map<std::array<double,3>, Mesh::Vertex_index> welded;
        bool topology_ok = true, degenerate = false;
        for (size_t i=0; i<nf; ++i) {
            std::array<Mesh::Vertex_index,3> ids;
            for (size_t j=0; j<3; ++j) {
                std::array<double,3> xyz;
                for (auto& v : xyz) v=number();
                auto found = welded.find(xyz);
                if (found == welded.end()) {
                    auto id = mesh.add_vertex(K::Point_3(K::FT(xyz[0])*scale[0],K::FT(xyz[1])*scale[1],K::FT(xyz[2])*scale[2]));
                    found=welded.emplace(xyz,id).first;
                }
                ids[j]=found->second;
            }
            if (K::Triangle_3(mesh.point(ids[0]),mesh.point(ids[1]),mesh.point(ids[2])).is_degenerate()) degenerate=true;
            if (mesh.add_face(ids[0],ids[1],ids[2]) == Mesh::null_face()) topology_ok=false;
        }
        for (size_t i=0; i<nc; ++i) {
            double x=number(), y=number(), z=number(), p=number(), u=number();
            if (p <= 0 || u < 0) throw std::runtime_error("invalid_cell");
            // 867/1000 > sqrt(3)/2; covers every point in the full cube plus uncertainty.
            cells.push_back({K::Point_3(x,y,z),K::FT(p)*K::FT(867)/K::FT(1000)+K::FT(u)});
        }
        std::string extra;
        if (std::cin >> extra) throw std::runtime_error("extra_input");
        if (!topology_ok || mesh.number_of_faces()!=nf) throw std::runtime_error("invalid_manifold_or_winding");
        if (degenerate) throw std::runtime_error("degenerate_triangle");
        if (!CGAL::is_valid_polygon_mesh(mesh) || !CGAL::is_triangle_mesh(mesh) || !CGAL::is_closed(mesh))
            throw std::runtime_error("invalid_or_open_mesh");
        // Volume/orientation APIs require absence of self intersections first.
        if (PMP::does_self_intersect(mesh)) throw std::runtime_error("self_intersection");
        auto cc=mesh.add_property_map<Mesh::Face_index,size_t>("f:component",0).first;
        components=PMP::connected_components(mesh,cc);
        std::vector<K::FT> volumes(components,K::FT(0));
        const K::Point_3 origin(0,0,0);
        for (auto f:mesh.faces()) {
            auto h=mesh.halfedge(f);
            volumes[cc[f]] += CGAL::volume(origin,mesh.point(mesh.source(h)),mesh.point(mesh.target(h)),mesh.point(mesh.target(mesh.next(h))));
        }
        for (const auto& volume:volumes) if (volume == 0) throw std::runtime_error("zero_volume_component");
        if (!PMP::does_bound_a_volume(mesh)) throw std::runtime_error("invalid_volume_orientation");
        Tree tree(faces(mesh).first,faces(mesh).second,mesh);
        tree.build();
        tree.accelerate_distance_queries();
        // Whole-mesh parity preserves nested cavities; never union components.
        CGAL::Side_of_triangle_mesh<Mesh,K> side(tree);
        std::cout << "{\"status\":\"proved\",\"reason\":\"exact_mesh_and_cell_checks\",\"components\":" << components
                  << ",\"cgal_version\":\"" << CGAL_VERSION_STR << "\",\"cells\":[";
        bool first=true;
        for (const auto& c:cells) {
            bool inside = side(c.center)==CGAL::ON_BOUNDED_SIDE;
            bool safe = inside && tree.squared_distance(c.center)>c.radius*c.radius;
            if (!first) std::cout << ',';
            first=false;
            std::cout << "{\"certified\":" << (safe?"true":"false") << ",\"reason\":\""
                      << (safe?"strict_full_cell_interior":(inside?"boundary_clearance_unproven":"not_material_interior")) << "\"}";
        }
        auto seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        std::cout << "],\"backend_elapsed_s\":" << seconds << "}\n";
        return 0;
    } catch (const std::exception& e) {
        // Fixed protocol reasons only; arbitrary exception text never enters JSON.
        reason=e.what();
        static const std::vector<std::string> allowed={"invalid_manifold_or_winding","degenerate_triangle","invalid_or_open_mesh","self_intersection","zero_volume_component","invalid_volume_orientation"};
        bool known=std::find(allowed.begin(),allowed.end(),reason)!=allowed.end();
        std::cout << "{\"status\":\"" << (known?"rejected":"unknown") << "\",\"reason\":\"" << (known?reason:"invalid_input_or_backend_exception")
                  << "\",\"components\":" << components << ",\"cgal_version\":\"" << CGAL_VERSION_STR << "\",\"cells\":[";
        for (size_t i=0;i<cells.size();++i) { if(i)std::cout<<','; std::cout<<"{\"certified\":false,\"reason\":\"mesh_not_proven\"}"; }
        std::cout << "]}\n";
        return 0;
    }
}
