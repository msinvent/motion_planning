add_library(hybrid_bfs_library include/hybrid_breadth_first.cpp)

add_executable(bfs bfs.cpp)
target_link_libraries(bfs hybrid_bfs_library)
