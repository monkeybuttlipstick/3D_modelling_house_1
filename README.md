To use this code, all you need to do is to change the input and output into the files name.
	In line 47 and 48:
const std::string input_file = "../station.hw1";
const std::string output_file = "../station.obj";
change the station.hw1 and station.obj



CmakeList: (Need to change according to file)
cmake_minimum_required(VERSION 3.20)
project(learning)
set(CMAKE_CXX_STANDARD 14)
set(BOOST_ROOT "C:/dev/boost")
find_package(CGAL PATHS "C:/Program Files (x86)/CGAL")
add_executable(learning main.cpp)
target_link_libraries(learning CGAL::CGAL)