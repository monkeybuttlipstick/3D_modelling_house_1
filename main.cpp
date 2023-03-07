#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <list>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_2.h>

struct FaceInfo {
    bool processed;
    bool interior;
    FaceInfo() {
        processed = false;
        interior = false;
    }
};
struct FaceInfo2
{
    FaceInfo2(){}
    int nesting_level;
    bool in_domain(){
        return nesting_level%2 == 1;
    }
};
typedef CGAL::Exact_predicates_tag Tag;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Triangulation_vertex_base_2<Kernel> VertexBase;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel> FaceBase;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, Kernel, FaceBase> FaceBaseWithInfo;
typedef CGAL::Triangulation_data_structure_2<VertexBase, FaceBaseWithInfo> TriangulationDataStructure;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TriangulationDataStructure, Tag> Triangulation;




typedef Triangulation::Face_handle       Face_handle;
typedef Kernel::Plane_3                  Plane;
typedef Kernel::Point_3                  Point;
typedef Kernel::Point_2                  Point_2;
typedef Kernel::FT FT;

const std::string input_file = "../station.hw1";
const std::string output_file = "../station.obj";

struct Vertex {
    int id;
    double x, y, z;
};

struct Face {
    std::list<int> outer_ring;
    std::list<std::list<int>> inner_rings;
    Kernel::Plane_3 best_plane;
    Triangulation triangulation;
};
//ken
//void label_triangles(Face face) {
//    std::list<Triangulation::Face_handle> to_check;
//    face.triangulation.infinite_face()->info().processed = true;
//    CGAL_assertion(face.triangulation.infinite_face()->info().processed == true);
//    CGAL_assertion(face.triangulation.infinite_face()->info().interior == false);
//    to_check.push_back(face.triangulation.infinite_face());
//    while (!to_check.empty()) {
//        CGAL_assertion(to_check.front()->info().processed == true);
//        for (int neighbour = 0; neighbour < 3; ++neighbour) {
//            if (to_check.front()->neighbor(neighbour)->info().processed == true) {
//                // Note: validation code.
////          if (triangulation.is_constrained(Triangulation::Edge(to_check.front(), neighbour))) CGAL_assertion(to_check.front()->neighbor(neighbour)->info().interior != to_check.front()->info().interior);
////          else CGAL_assertion(to_check.front()->neighbor(neighbour)->info().interior == to_check.front()->info().interior);
//            } else {
//                to_check.front()->neighbor(neighbour)->info().processed = true;
//                CGAL_assertion(to_check.front()->neighbor(neighbour)->info().processed == true);
//                if (face.triangulation.is_constrained(Triangulation::Edge(to_check.front(), neighbour))) {
//                    to_check.front()->neighbor(neighbour)->info().interior = !to_check.front()->info().interior;
//                    to_check.push_back(to_check.front()->neighbor(neighbour));
//                } else {
//                    to_check.front()->neighbor(neighbour)->info().interior = to_check.front()->info().interior;
//                    to_check.push_back(to_check.front()->neighbor(neighbour));
//                }
//            }
//        } to_check.pop_front();
//    }
//}
//CGAL

void mark_domains(Triangulation& ct,
                  Face_handle start,
                  int index,
                  std::list<Triangulation::Edge>& border )
{
    if(start->info().nesting_level != -1){
        return;
    }
    std::list<Face_handle> queue;
    queue.push_back(start);
    while(! queue.empty()){
        Face_handle fh = queue.front();
        queue.pop_front();
        if(fh->info().nesting_level == -1){
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++){
                Triangulation::Edge e(fh,i);
                Face_handle n = fh->neighbor(i);
                if(n->info().nesting_level == -1){
                    if(ct.is_constrained(e)) border.push_back(e);
                    else queue.push_back(n);
                }
            }
        }
    }
}
//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains(Triangulation& cdt)
{
    for(Triangulation::Face_handle f : cdt.all_face_handles()){
        f->info().nesting_level = -1;
    }
    std::list<Triangulation::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while(! border.empty()){
        Triangulation::Edge e = border.front();
        border.pop_front();
        Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1){
            mark_domains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}



int main(int argc, const char * argv[]) {

    std::map<int, Vertex> vertices;
    std::map<int, Face> faces;

    // Read file
    std::ifstream input_stream;
    input_stream.open(input_file, std::ios::in);
    if (input_stream.is_open()) {
        std::string line;

        // Read vertex header
        getline(input_stream, line);
        std::cout << "Vertex header: " << line << std::endl;
        std::istringstream vertex_header_stream(line);
        int number_of_vertices;
        vertex_header_stream >> number_of_vertices;
        std::cout << "Parsing " << number_of_vertices << " vertices..." << std::endl;

        // Read vertices
        for (int i = 0; i < number_of_vertices; ++i) {
            getline(input_stream, line);
            std::istringstream line_stream(line);
            int id;
            double x, y, z;
            line_stream >> id >> x >> y >> z;
//            std::cout << "Vertex " << id << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
            vertices[id].x = x;
            vertices[id].y = y;
            vertices[id].z = z;
        }
        // Read faces header
        getline(input_stream, line);
        std::cout << "Face header: " << line << std::endl;
        std::istringstream face_header_stream(line);
        int number_of_faces;
        face_header_stream >> number_of_faces;
        std::cout << "Parsing " << number_of_faces << " faces..." << std::endl;
        // Read faces
        int pt_count=0;
        for (int i = 0; i < number_of_faces; ++i) {
            getline(input_stream, line);
            std::istringstream line_stream(line);
            int outer_no,inner_no;
            line_stream >> outer_no >> inner_no ;
            int total_no = outer_no + inner_no;
            for (int j = 0; j < total_no;++j){
                getline(input_stream, line);
                std::istringstream line_stream1(line);
                int num;
                line_stream1 >> num;
                if (j ==0){
                    for (int k = 0; k < num; ++k) {
                        int vertice_id;
                        line_stream1 >> vertice_id;
                        faces[i].outer_ring.emplace_back(vertice_id);
                    }
                }
                else{
                    std::list<int> inner_ring;
                    for (int k = 0; k < num; ++k) {
                        int vertice_id;
                        line_stream1 >> vertice_id;
                        inner_ring.emplace_back(vertice_id);
                    }
                    faces[i].inner_rings.emplace_back(inner_ring);
                }

            }

        }
        // easy and fast iteration through all vertices of a face

        std::vector<int> count_list;
        for (auto const &face: faces) {
            std::vector<Point> vertices_face{};
            //list for outer and inner ring point
            std::vector<Point> vertices_face_outer{};
            std::vector<std::vector<Point>> vertices_face_inner{};
            std::cout << "Face " << face.first << ": " << std::endl;
//                std::cout << "\touter:";
            for (auto const &vertex: face.second.outer_ring){
                vertices_face.emplace_back(Point(vertices[vertex].x,vertices[vertex].y,vertices[vertex].z));
                vertices_face_outer.emplace_back(Point(vertices[vertex].x,vertices[vertex].y,vertices[vertex].z));
            }
//                    std::cout << " " << vertex;
//                std::cout << std::endl;
            for (auto const &ring: face.second.inner_rings) {
                std::cout << "\tinner:";
                std::vector<Point> vertice_face_inner{};
                for (auto const &vertex: ring){
                    vertices_face.emplace_back(Point(vertices[vertex].x,vertices[vertex].y,vertices[vertex].z));
                    vertice_face_inner.emplace_back(Point(vertices[vertex].x,vertices[vertex].y,vertices[vertex].z));
                }
                vertices_face_inner.emplace_back(vertice_face_inner);
//                        std::cout << " " << vertex;
//                    std::cout << std::endl;
            }
            FT rmse = linear_least_squares_fitting_3(vertices_face.begin(),vertices_face.end(),faces[face.first].best_plane,CGAL::Dimension_tag<0>());
            //output check
//            std::cout << "\n\tnumber of vertices: " << vertices_face.size();
//            std::cout << "\n\tPlane fitting: " << faces[face.first].best_plane << std::endl;
//            std::cout << "\tRMSE error: " << rmse << std::endl;
//project and triangulation constrainted the outer and inner faces
//            if (std::isnan(rmse)) {
//                std::cout << "\n\tFACE IS INVALID";
//                continue;
//            }
//            else {
            std::vector<Point_2> projected_pts_outer;
            for (auto const &pt: vertices_face_outer) {
                projected_pts_outer.emplace_back(face.second.best_plane.to_2d(pt));
                //check
//                std::cout << face.first << faces[face.first].best_plane.to_2d(pt)<< std::endl;
            }
//                faces[face.first].triangulation.insert(projected_pts_outer.begin(), projected_pts_outer.end());
            faces[face.first].triangulation.insert(projected_pts_outer.begin(),
                                                   projected_pts_outer.end());
            faces[face.first].triangulation.insert_constraint(projected_pts_outer.begin(),
                                                              projected_pts_outer.end(),true);

            for (auto const &rings: vertices_face_inner) {
                std::vector<Point_2> projected_pts_inner;
                for (auto const &pt2: rings) {
                    projected_pts_inner.emplace_back(faces[face.first].best_plane.to_2d(pt2));
                }
//                    faces[face.first].triangulation.insert(projected_pts_inner.begin(), projected_pts_inner.end());
                faces[face.first].triangulation.insert(projected_pts_inner.begin(),
                                                       projected_pts_inner.end());
                faces[face.first].triangulation.insert_constraint(projected_pts_inner.begin(),
                                                                  projected_pts_inner.end(),true);

            }
//            label_triangles(faces[face.first]);


            mark_domains(faces[face.first].triangulation);
            int count=0;
            for (Face_handle f : faces[face.first].triangulation.finite_face_handles())
            {
                if ( f->info().in_domain() ) ++count;}
            std::cout << "There are " << count << " facets in the domain." << std::endl;
            //output the file
            std::ofstream output_stream;
            output_stream.open(output_file,  std::ios::out | std::ios::app);


            if (output_stream.is_open()){
                std::vector<Kernel::Point_2> point_vector;
                for (auto v = faces[face.first].triangulation.finite_vertices_begin();v!=faces[face.first].triangulation.finite_vertices_end();++v){
                    Point vertex_3d = face.second.best_plane.to_3d(v->point());
                    point_vector.push_back(v->point());
                    pt_count+=1;
                    std::cout << vertex_3d <<" "<<std::endl;
                    output_stream << "v " <<vertex_3d<<std::endl;

                }
                count_list.push_back(pt_count);
                int size = count_list.size();
//                int n = pt_index.size();
//                output_stream << "f " << pt_index[num];
//                for(int i = num+1; i < n; i++){output_stream << " "<<pt_index[i];}
//                output_stream <<std::endl;
            for (auto f = faces[face.first].triangulation.all_faces_begin();f != faces[face.first].triangulation.all_faces_end(); ++f){
                if(f->info().in_domain()){
                    int index;
                    Kernel::Point_2 p = f->vertex(0)->point();
                    auto pt = std::find(point_vector.begin(), point_vector.end(), p);
                    index = int(std::distance(point_vector.begin(), pt))+1;
                    if(size == 1){
                        output_stream << "f " << index;
                    }
                    else{
                        output_stream << "f " << index+count_list[size - 2];
                        std::cout<<count_list[size - 2]<<std::endl;
                    }
                    for (int i = 1; i < 3; ++i){
                        int index;
                        Kernel::Point_2 p = f->vertex(i)->point();
                        auto pt = std::find(point_vector.begin(), point_vector.end(), p);
                        index = int(std::distance(point_vector.begin(), pt))+1;
                        if(size == 1) { output_stream << " " << index; }
                        else{output_stream << " " << index+count_list[size - 2];}

                    }
                    output_stream <<std::endl;
                }
            }

            }
            output_stream.close();
        }

    }
    return 0;
}