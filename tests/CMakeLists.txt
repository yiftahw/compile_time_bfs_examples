add_executable(bfs_tests
    test_bfs_route_finder.cpp
)

target_link_libraries(bfs_tests
    PRIVATE
    boost_mpl_graph_bfs
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(bfs_tests)
