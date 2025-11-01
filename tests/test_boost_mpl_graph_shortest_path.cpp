#include <type_traits>

#include <gtest/gtest.h>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/size.hpp>
#include <boost/mpl/at.hpp>

#include "boost_mpl_graph_shortest_path.hpp"

// Define the test graph structure from the example in the header
// adjacency list test graph:
// A -> B -> C -\--> D
//        \     |--> E
//         \    \--> F
//          \-----/
// G (disconnected)

// vertices
struct A{}; 
struct B{}; 
struct C{}; 
struct D{}; 
struct E{}; 
struct F{}; 
struct G{};

// edges
struct A_B{}; 
struct B_C{}; 
struct C_D{}; 
struct C_E{}; 
struct C_F{}; 
struct B_F{};

// define the graph structure
typedef mpl::vector<
    mpl::vector<A_B, A, B>,
    mpl::vector<B_C, B, C>,
    mpl::vector<C_D, C, D>,
    mpl::vector<C_E, C, E>,
    mpl::vector<C_F, C, F>,
    mpl::vector<B_F, B, F>
> my_graph_data_type;

typedef mpl_graph::incidence_list_graph<my_graph_data_type> my_graph_type;

// Test that a route exists from A to F and is the shortest path A -> B -> F
TEST(BFSRouteFinderTest, RouteFromAToFExists) {
    using route_A_to_F = bfs_route_query_result_t<my_graph_type, A, F>;
    
    // Check that route was found
    static_assert(bfs_route_found_v<route_A_to_F>);
    
    // Check that the path has 3 elements
    constexpr auto path_size = bfs_route_length_v<route_A_to_F>;
    static_assert(path_size == 3);

    // Check that the route is actually A -> B -> F
    static constexpr bool path_correct = mpl::equal<bfs_route_path_t<route_A_to_F>, mpl::vector3<A,B,F>>::value;
    static_assert(path_correct);
}

// no route from A to G
TEST(BFSRouteFinderTest, NoRouteFromAToG) {
    using route_A_to_G = bfs_route_query_result_t<my_graph_type, A, G>;
    // Check that route was not found
    static_assert(!bfs_route_found_v<route_A_to_G>);
}
