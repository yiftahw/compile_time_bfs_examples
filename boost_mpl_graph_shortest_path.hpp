#pragma once

/*
    For the example graph

    adjacency list test graph:
    A -> B -> C -\--> D
           \     |--> E
            \    \--> F
             \-----/
    G
*/

/*
// vertices
struct A{}; struct B{}; struct C{}; struct D{}; struct E{}; struct F{}; struct G{};

// edges
struct A_B{}; struct B_C{}; struct C_D{}; struct C_E{}; struct C_F{}; struct B_F{};

// define the graph structure
typedef mpl::vector<
                mpl::vector<A_B,A,B>,
                mpl::vector<B_C,B,C>,
                mpl::vector<C_D,C,D>,
                mpl::vector<C_E,C,E>,
                mpl::vector<C_F,C,F>,
                mpl::vector<B_F,B,F>
            > my_graph_data_type;
typedef mpl_graph::incidence_list_graph<my_graph_data_type> my_graph_type;

// best route from A to F is A -> B -> F and NOT A -> B -> C -> F
typedef mpl::first<bfs_route_finder<my_graph_type, A, F>::type>::type route_A_to_F;
BOOST_MPL_ASSERT((boost::is_same<bfs_route_found_t<route_A_to_F>, mpl::true_>));
BOOST_MPL_ASSERT((boost::is_same<bfs_route_path_t<route_A_to_F>, mpl::vector3<A, B, F>>));

// there is no route from A to G
typedef mpl::first<bfs_route_finder<my_graph_type, A, G>::type>::type route_A_to_G;
BOOST_MPL_ASSERT((boost::is_same<bfs_route_found_t<route_A_to_G>, mpl::false_>));
*/

#include <boost/msm/mpl_graph/incidence_list_graph.hpp>
#include <boost/msm/mpl_graph/breadth_first_search.hpp>
#include <boost/msm/mpl_graph/depth_first_search.hpp>

namespace mpl_graph = boost::msm::mpl_graph;
namespace mpl = boost::mpl;

// visitor strategy to build the path on traversal
template<typename TargetNode>
struct route_finder_visitor : mpl_graph::bfs_default_visitor_operations {
    using StateType = 
        mpl::pair<
            mpl::vector<>, // path stack
            mpl::pair<TargetNode, mpl::false_> // target node and found flag
        >;

    // examine a new vertex (after already been discovered earlier)
    // if not yet found, push to path stack
    template<typename Vertex, typename Graph, typename State>
    struct examine_vertex : mpl::if_<mpl::not_<typename State::second::second>,
        // if not found - push to path stack, leave metadata unchanged
        mpl::pair<
            typename mpl::push_back<typename State::first, Vertex>::type,
            typename State::second
        >,
        // if found - do nothing
        State
    > {};

    // first time we encounter a new vertex
    template<typename Vertex, typename Graph, typename State>
    struct discover_vertex : mpl::if_<boost::is_same<Vertex, typename State::second::first>,
        // if found - set found flag to true and push to path stack
        mpl::pair<
            typename mpl::push_back<typename State::first, Vertex>::type,
            mpl::pair<typename State::second::first, mpl::true_>>,
        // if not found - do nothing
        State
    > {};
};

/**
 * @brief A compile-time breadth-first search route finder that finds paths between nodes in a graph.
 * 
 * This template class specializes the mpl_graph::breadth_first_search algorithm to find
 * routes from a source node to a target node within a given graph structure. It uses
 * a route_finder_visitor to track the search progress and maintain state during traversal.
 * 
 * @tparam Graph The graph type to search within
 * @tparam SourceNode The starting node for the route search
 * @tparam TargetNode The destination node for the route search
 */
template <typename Graph, typename SourceNode, typename TargetNode>
struct bfs_route_finder : mpl_graph::breadth_first_search<
    Graph,
    route_finder_visitor<TargetNode>,
    typename route_finder_visitor<TargetNode>::StateType,
    SourceNode
> {};

// Helper type to extract whether a route was found
// TODO: add a concept on the type of QueryResult
template <typename QueryResult>
using bfs_route_found_t = typename mpl::second<typename mpl::second<QueryResult>::type>::type;

// Helper type to extract the route path
template <typename QueryResult>
using bfs_route_path_t = typename mpl::first<QueryResult>::type;
