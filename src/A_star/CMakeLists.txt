add_library(hybrid_a_star_library include/hybrid_a_star.cpp)

add_executable(a_star a_star.cpp)
target_link_libraries(a_star hybrid_a_star_library)
