#pragma once

#include <array>
#include <algorithm>
#include <cstddef>
#include <concepts>
#include <span>

template <typename E>
concept EdgeConcept = requires(E e) {
    { e.src };
    { e.dst };
    requires std::same_as<std::remove_cvref_t<decltype(e.src)>, std::remove_cvref_t<decltype(e.dst)>>;
};

template <typename E>
using NodeType = std::remove_cvref_t<decltype(E::src)>;

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
template <typename EdgeType, size_t NumEdges, size_t NodeCount>
    requires EdgeConcept<EdgeType>
constexpr auto adjacency_list(const std::array<EdgeType, NumEdges>& edges) {
    std::array<std::array<size_t, NumEdges>, NodeCount> out{};
    std::array<size_t, NodeCount> counts{};
    for (size_t i = 0; i < NumEdges; ++i) {
        NodeType<EdgeType> u = edges[i].src;
        out[u][counts[u]++] = i;
    }
    return std::make_pair(out, counts);
}

// ---------- result ----------
template <typename EdgeType, size_t MaxEdges>
    requires EdgeConcept<EdgeType>
struct BFSResult {
    std::array<EdgeType, MaxEdges> path{};
    size_t length = 0;
    bool found = false;

    constexpr auto view() const {
        return std::span(path.data(), length);
    }
};

// ---------- comparison operator ----------
template <typename EdgeType, size_t MaxEdges, size_t RouteLength>
requires EdgeConcept<EdgeType>
constexpr bool compare_routes(const BFSResult<EdgeType, MaxEdges>& a,
                          const std::array<NodeType<EdgeType>, RouteLength>& b) {
    if (a.length != RouteLength - 1) return false;
    for (size_t i = 0; i < a.length; ++i) {
        if (a.path[i].src != b[i] || a.path[i].dst != b[i + 1]) {
            return false;
        }
    }
    return true;
}

// ---------- constexpr BFS ----------
template <size_t NumNodes, typename EdgeType, size_t NumEdges>
    requires EdgeConcept<EdgeType>
constexpr BFSResult<EdgeType, NumNodes - 1>
bfs_edges(const std::array<EdgeType, NumEdges>& edges, NodeType<EdgeType> start, NodeType<EdgeType> goal) {
    auto [adj, counts] = adjacency_list<EdgeType, NumEdges, NumNodes>(edges);
    std::array<int, NumNodes> prev{};
    std::array<int, NumNodes> via_edge{};
    std::array<bool, NumNodes> visited{};

    for (size_t i = 0; i < NumNodes; ++i) {
        prev[i] = -1;
        via_edge[i] = -1;
        visited[i] = false;
    }

    ConstexprQueue<NodeType<EdgeType>, NumNodes> q{};
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        NodeType<EdgeType> u = q.pop();
        if (u == goal) break;
        for (size_t i = 0; i < counts[u]; ++i) {
            size_t eidx = adj[u][i];
            NodeType<EdgeType> v = edges[eidx].dst;
            if (!visited[v]) {
                visited[v] = true;
                prev[v] = static_cast<int>(u);
                via_edge[v] = static_cast<int>(eidx);
                q.push(v);
            }
        }
    }

    BFSResult<EdgeType, NumNodes - 1> result{};
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
