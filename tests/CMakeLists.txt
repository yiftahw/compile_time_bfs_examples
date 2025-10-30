add_executable(bfs_tests
    test_boost_mpl_graph_bfs_route_finder.cpp
    test_constexpr_graph_bfs_route_finder.cpp
)

target_link_libraries(bfs_tests
    PRIVATE
    compile_time_graph_shortest_path
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(bfs_tests)
