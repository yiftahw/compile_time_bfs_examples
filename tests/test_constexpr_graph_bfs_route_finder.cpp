#include <gtest/gtest.h>

#include "constexpr_graph_bfs_query.hpp"

enum Node { A, B, C, D, E, F, G };
static constexpr size_t NodeCount = G + 1;

struct EdgeData {};

constexpr std::array<Edge<Node, EdgeData>, 6> edges = {{
    {A, B, {}},
    {B, C, {}},
    {C, D, {}},
    {C, E, {}},
    {C, F, {}},
    {B, F, {}}
}};

// ---------- compile time test ----------

TEST(ConstexprBFS, RouteAtoF) {
    constexpr auto route_A_to_F = bfs_edges<Node, EdgeData, NodeCount>(edges, A, F);

    static_assert(route_A_to_F.found);
    static_assert(route_A_to_F.length == 2);
    static_assert(route_A_to_F.path[0].src == A);
    static_assert(route_A_to_F.path[1].dst == F);
}
