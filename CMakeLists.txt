cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Openings_based__registration)


find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(include/)

file(GLOB HEADERS
    include/*.hpp
    
    

)


add_executable(O_Registration  ./src/O_Registration.cpp ${HEADERS} ${SOURCES})
target_link_libraries ( O_Registration ${PCL_LIBRARIES})
#add_executable(asc2ply ./src/asc2ply.cpp )
#target_link_libraries ( asc2ply ${PCL_LIBRARIES}${CGAL_LIBRARY})


