#include <gtest/gtest.h>

#include "constexpr_graph_shortest_path.hpp"

enum Node { A, B, C, D, E, F, G };
static constexpr size_t NodeCount = G + 1;

struct Edge {
    Node src;
    Node dst;
};
static_assert(EdgeConcept<Edge>);

constexpr std::array<Edge, 6> edges = {{
    {A, B},
    {B, C},
    {C, D},
    {C, E},
    {C, F},
    {B, F}
}};

// ---------- compile time test ----------

TEST(ConstexprBFS, RouteAtoF) {
    constexpr auto route_A_to_F = bfs_edges<NodeCount>(edges, A, F);
    static_assert(route_A_to_F.found);
    static_assert(route_A_to_F.length == 2);
    static_assert(compare_routes(route_A_to_F, std::array{A, B, F})); // 3 nodes are covered in 2 edges
}
