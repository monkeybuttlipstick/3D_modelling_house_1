
cmake_minimum_required(VERSION 3.20)
project(learning)
set(CMAKE_CXX_STANDARD 14)
set(BOOST_ROOT "C:/dev/boost")
find_package(CGAL PATHS "C:/Program Files (x86)/CGAL")
add_executable(learning main.cpp)
target_link_libraries(learning CGAL::CGAL)