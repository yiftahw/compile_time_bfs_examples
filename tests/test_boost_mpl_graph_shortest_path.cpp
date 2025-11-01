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
struct H{};

// edges
struct A_B{};
struct A_C{};
struct B_C{}; 
struct B_D{};
struct B_F{};
struct C_D{}; 
struct C_E{}; 
struct C_F{};
struct D_F{};
struct F_H{};
struct E_G{};

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

#define EDGE(src,dst) mpl::vector<src##_##dst, src, dst>

TEST(BFSRouteFinderTest, BacktrackingMetaFunction) {
    // simple testcase for backtrack
    struct V1{};
    struct V2{};
    struct V3{};
    using parent_map_example = mpl::map<
        mpl::pair<V3, V2>,
        mpl::pair<V2, V1>
    >;
    // check that if vertex not in map, we get mpl::void_
    static_assert(boost::is_same<mpl::at<parent_map_example, V1>::type, mpl::void_>::value, "parent map test failed");

    using backtrack_test_no_parent = typename backtrack<V1, parent_map_example>::type;
    static_assert(mpl::equal<backtrack_test_no_parent, mpl::vector1<V1>>::value, "backtrack test failed");

    using backtrack_test_1_step = typename backtrack<V2, parent_map_example>::type;
    static_assert(mpl::equal<backtrack_test_1_step, mpl::vector2<V1, V2>>::value, "backtrack test failed");

    using backtrack_test = typename backtrack<V3, parent_map_example>::type;
    static_assert(mpl::equal<backtrack_test, mpl::vector3<V1, V2, V3>>::value, "backtrack test failed");
}


template <typename Map1, typename Map2>
struct map_subset_of
  : mpl::fold<
        Map1,
        mpl::true_,
        mpl::and_<
            _1,
            mpl::and_<
                mpl::has_key<Map2, typename _2::first>,
                boost::is_same<
                    typename mpl::at<Map2, typename _2::first>::type,
                    typename _2::second
                >
            >
        >
    >
{};

// Equality check = subset both ways
template <typename Map1, typename Map2>
struct map_equal
  : mpl::and_<
        map_subset_of<Map1, Map2>,
        map_subset_of<Map2, Map1>
    >
{};

// small test for compare key-value maps
using map1 = mpl::map<mpl::pair<A, B>, mpl::pair<C, D>>;
using map2 = mpl::map<mpl::pair<C, D>, mpl::pair<A, B>>;
using map3 = mpl::map<mpl::pair<A, B>, mpl::pair<C, D>>;
static_assert(map_equal<map1, map2>::value, "map equal test failed");


// // Test that a route exists from A to F and is the shortest path A -> B -> F
TEST(BFSRouteFinderTest, RouteFromAToFExists) {
    using route_A_to_F = bfs_route_query_result_t<my_graph_type, A, F>;

    /*
    parents:
    B <- A
    F <- B
    C <- B
    D <- C
    E <- C
    F <- C
    */
    using ref_parent_map = mpl::map<
        mpl::pair<B, A>,
        mpl::pair<F, B>,
        mpl::pair<C, B>,
        mpl::pair<D, C>,
        mpl::pair<E, C>,
        mpl::pair<F, C>>;
    
    static_assert(mpl::equal<bfs_parent_map_t<route_A_to_F>, ref_parent_map>::value);
    
    // Check that route was found
    static_assert(bfs_route_found_v<route_A_to_F>);
    
    // Check that the path has 3 elements
    constexpr auto path_size = bfs_route_length_v<route_A_to_F>;
    static_assert(path_size == 3);

    // Check that the route is actually A -> B -> F
    static constexpr bool path_correct = mpl::equal<bfs_route_path_t<route_A_to_F>, mpl::vector<A,B,F>>::value;
    static_assert(path_correct);
}

// // no route from A to G
// TEST(BFSRouteFinderTest, NoRouteFromAToG) {
//     using route_A_to_G = bfs_route_query_result_t<my_graph_type, A, G>;
//     // Check that route was not found
//     static_assert(!bfs_route_found_v<route_A_to_G>);
// }

// TEST(BFSRouteFinderTest, Test2) {
//     using graph_2_edges_t = mpl::vector<
//         EDGE(A,B),
//         EDGE(B,D),
//         EDGE(D,F),
//         EDGE(F,H),
//         EDGE(A,C),
//         EDGE(C,E),
//         EDGE(E,G)
//     >;
//     using graph_2_t = mpl_graph::incidence_list_graph<graph_2_edges_t>;

//     using route_A_to_H = bfs_route_query_result_t<graph_2_t, A, H>;
//     static_assert(bfs_route_found_v<route_A_to_H>);
//     using route = bfs_route_path_t<route_A_to_H>;
//     static_assert(mpl::equal<route, mpl::vector<A,B,D,F,H>>::value);
// }
