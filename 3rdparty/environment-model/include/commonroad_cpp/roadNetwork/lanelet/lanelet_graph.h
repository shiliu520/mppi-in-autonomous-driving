#pragma once

#include "dijkstra.h"
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

class Lanelet;

class LaneletGraph {
    std::unordered_map<size_t, size_t> verticesAdjSuc;
    std::unordered_map<size_t, size_t> verticesSuc;
    std::unordered_map<size_t, size_t> verticesAdjSucRes;
    std::unordered_map<size_t, size_t> verticesSucRes;
    graph<std::size_t, size_t> graphAdjSuc;
    graph<std::size_t, size_t> graphSuc;
    std::map<std::array<size_t, 2>, std::vector<size_t>> queries;

  public:
    explicit LaneletGraph(const std::vector<std::shared_ptr<Lanelet>> &lanelets);
    std::vector<size_t> findPaths(size_t src, size_t dst, bool considerAdjacency);
};
