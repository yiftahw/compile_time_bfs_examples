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
    typedef mpl::first<bfs_route_finder<my_graph_type, A, F>::type>::type route_A_to_F;
    
    // Check that route was found
    constexpr bool route_found = std::is_same<bfs_route_found_t<route_A_to_F>, mpl::true_>::value;
    EXPECT_TRUE(route_found);
    
    // Check that the path has 3 elements (A, B, F)
    typedef bfs_route_path_t<route_A_to_F> path_type;
    constexpr int path_size = mpl::size<path_type>::value;
    EXPECT_EQ(path_size, 3);
    
    // Check individual elements in the path
    constexpr bool first_is_A = std::is_same<typename mpl::at_c<path_type, 0>::type, A>::value;
    constexpr bool second_is_B = std::is_same<typename mpl::at_c<path_type, 1>::type, B>::value;
    constexpr bool third_is_F = std::is_same<typename mpl::at_c<path_type, 2>::type, F>::value;
    
    EXPECT_TRUE(first_is_A);
    EXPECT_TRUE(second_is_B);
    EXPECT_TRUE(third_is_F);
}

// Test that no route exists from A to G (G is disconnected)
TEST(BFSRouteFinderTest, NoRouteFromAToG) {
    typedef mpl::first<bfs_route_finder<my_graph_type, A, G>::type>::type route_A_to_G;
    
    // Check that no route was found
    constexpr bool route_not_found = std::is_same<bfs_route_found_t<route_A_to_G>, mpl::false_>::value;
    EXPECT_TRUE(route_not_found);
}

// Test route from A to D through the graph
TEST(BFSRouteFinderTest, RouteFromAToD) {
    typedef mpl::first<bfs_route_finder<my_graph_type, A, D>::type>::type route_A_to_D;
    
    // Check that route was found
    constexpr bool route_found = std::is_same<bfs_route_found_t<route_A_to_D>, mpl::true_>::value;
    EXPECT_TRUE(route_found);
    
    // Check that the path has 4 elements (A, B, C, D)
    typedef bfs_route_path_t<route_A_to_D> path_type;
    constexpr int path_size = mpl::size<path_type>::value;
    EXPECT_EQ(path_size, 4);
    
    // Check individual elements
    constexpr bool first_is_A = std::is_same<typename mpl::at_c<path_type, 0>::type, A>::value;
    constexpr bool second_is_B = std::is_same<typename mpl::at_c<path_type, 1>::type, B>::value;
    constexpr bool third_is_C = std::is_same<typename mpl::at_c<path_type, 2>::type, C>::value;
    constexpr bool fourth_is_D = std::is_same<typename mpl::at_c<path_type, 3>::type, D>::value;
    
    EXPECT_TRUE(first_is_A);
    EXPECT_TRUE(second_is_B);
    EXPECT_TRUE(third_is_C);
    EXPECT_TRUE(fourth_is_D);
}

// Test route from A to E
TEST(BFSRouteFinderTest, RouteFromAToE) {
    typedef mpl::first<bfs_route_finder<my_graph_type, A, E>::type>::type route_A_to_E;
    
    // Check that route was found
    constexpr bool route_found = std::is_same<bfs_route_found_t<route_A_to_E>, mpl::true_>::value;
    EXPECT_TRUE(route_found);
    
    // Check that the path has 4 elements (A, B, C, E)
    typedef bfs_route_path_t<route_A_to_E> path_type;
    constexpr int path_size = mpl::size<path_type>::value;
    EXPECT_EQ(path_size, 4);
    
    // Check individual elements
    constexpr bool first_is_A = std::is_same<typename mpl::at_c<path_type, 0>::type, A>::value;
    constexpr bool second_is_B = std::is_same<typename mpl::at_c<path_type, 1>::type, B>::value;
    constexpr bool third_is_C = std::is_same<typename mpl::at_c<path_type, 2>::type, C>::value;
    constexpr bool fourth_is_E = std::is_same<typename mpl::at_c<path_type, 3>::type, E>::value;
    
    EXPECT_TRUE(first_is_A);
    EXPECT_TRUE(second_is_B);
    EXPECT_TRUE(third_is_C);
    EXPECT_TRUE(fourth_is_E);
}

// Test route from B to F (should be direct)
TEST(BFSRouteFinderTest, RouteFromBToF) {
    typedef mpl::first<bfs_route_finder<my_graph_type, B, F>::type>::type route_B_to_F;
    
    // Check that route was found
    constexpr bool route_found = std::is_same<bfs_route_found_t<route_B_to_F>, mpl::true_>::value;
    EXPECT_TRUE(route_found);
    
    // Check that the path has 2 elements (B, F)
    typedef bfs_route_path_t<route_B_to_F> path_type;
    constexpr int path_size = mpl::size<path_type>::value;
    EXPECT_EQ(path_size, 2);
    
    // Check individual elements
    constexpr bool first_is_B = std::is_same<typename mpl::at_c<path_type, 0>::type, B>::value;
    constexpr bool second_is_F = std::is_same<typename mpl::at_c<path_type, 1>::type, F>::value;
    
    EXPECT_TRUE(first_is_B);
    EXPECT_TRUE(second_is_F);
}

// Test that a node has a route to itself (source == target)
TEST(BFSRouteFinderTest, RouteFromAToA) {
    typedef mpl::first<bfs_route_finder<my_graph_type, A, A>::type>::type route_A_to_A;
    
    // Check that route was found
    constexpr bool route_found = std::is_same<bfs_route_found_t<route_A_to_A>, mpl::true_>::value;
    EXPECT_TRUE(route_found);
    
    // Check that the path contains only A
    typedef bfs_route_path_t<route_A_to_A> path_type;
    constexpr int path_size = mpl::size<path_type>::value;
    EXPECT_EQ(path_size, 1);
    
    // Check individual element
    constexpr bool first_is_A = std::is_same<typename mpl::at_c<path_type, 0>::type, A>::value;
    EXPECT_TRUE(first_is_A);
}
