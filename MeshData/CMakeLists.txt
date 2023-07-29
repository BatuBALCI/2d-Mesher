add_library(Mesh_Lib Domain.cpp Mesh.cpp)
target_include_directories(Mesh_Lib PUBLIC ../DCEL $ENV{BOOST_DIR})