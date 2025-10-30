#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdio>

template <typename Node, typename Meta>
struct Edge {
    Node src;
    Node dst;
    Meta meta;
};

// ---------- constexpr queue ----------
template <typename T, size_t Capacity>
struct ConstexprQueue {
    std::array<T, Capacity> buf{};
    size_t head = 0, tail = 0, sz = 0;

    constexpr bool empty() const { return sz == 0; }
    constexpr void push(const T& v) {
        buf[tail] = v;
        tail = (tail + 1) % Capacity;
        ++sz;
    }
    constexpr T pop() {
        T v = buf[head];
        head = (head + 1) % Capacity;
        --sz;
        return v;
    }
};

// ---------- adjacency builder ----------
template <typename Node, typename Meta, size_t NumEdges, size_t NodeCount>
constexpr auto adjacency_list(const std::array<Edge<Node, Meta>, NumEdges>& edges) {
    std::array<std::array<size_t, NumEdges>, NodeCount> out{};
    std::array<size_t, NodeCount> counts{};
    for (size_t i = 0; i < NumEdges; ++i) {
        Node u = edges[i].src;
        out[u][counts[u]++] = i;
    }
    return std::make_pair(out, counts);
}

// ---------- result ----------
template <typename Node, typename Meta, size_t MaxEdges>
struct BFSResult {
    std::array<Edge<Node, Meta>, MaxEdges> path{};
    size_t length = 0;
    bool found = false;
};

// ---------- constexpr BFS ----------
template <typename Node, typename Meta, size_t NumNodes, size_t NumEdges>
constexpr BFSResult<Node, Meta, NumNodes - 1>
bfs_edges(const std::array<Edge<Node, Meta>, NumEdges>& edges, Node start, Node goal) {
    auto [adj, counts] = adjacency_list<Node, Meta, NumEdges, NumNodes>(edges);
    std::array<int, NumNodes> prev{};
    std::array<int, NumNodes> via_edge{};
    std::array<bool, NumNodes> visited{};

    for (size_t i = 0; i < NumNodes; ++i) {
        prev[i] = -1;
        via_edge[i] = -1;
        visited[i] = false;
    }

    ConstexprQueue<Node, NumNodes> q{};
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        Node u = q.pop();
        if (u == goal) break;
        for (size_t i = 0; i < counts[u]; ++i) {
            size_t eidx = adj[u][i];
            Node v = edges[eidx].dst;
            if (!visited[v]) {
                visited[v] = true;
                prev[v] = static_cast<int>(u);
                via_edge[v] = static_cast<int>(eidx);
                q.push(v);
            }
        }
    }

    BFSResult<Node, Meta, NumNodes - 1> result{};
    if (!visited[goal]) return result;

    // reconstruct path (edges)
    size_t len = 0;
    for (int at = goal; at != start; at = prev[at]) {
        result.path[len++] = edges[via_edge[at]];
    }
    std::reverse(result.path.begin(), result.path.begin() + len);
    result.length = len;
    result.found = true;
    return result;
}
