cmake_minimum_required (VERSION 2.8.11)
project (CGAL)

find_package(CGAL REQUIRED COMPONENTS Core) # If the dependency is required, use REQUIRED option - if it's not found CMake will issue an error
include( ${CGAL_USE_FILE} )

include( CGAL_CreateSingleSourceCGALProgram )


find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

create_single_source_cgal_program( "main.cpp" )
