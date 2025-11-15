#include "commonroad_cpp/roadNetwork/lanelet/dijkstra.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

#include <array>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_graph.h>
#include <vector>

std::vector<size_t> LaneletGraph::findPaths(size_t src, const size_t dst, bool considerAdjacency) {
    if (queries.find({src, dst}) != queries.end())
        return queries[{src, dst}];

    std::vector<size_t> path;

    if (src == dst)
        path = {src};
    else {
        const auto srcId = static_cast<ptrdiff_t>(verticesAdjSuc.at(src));
        const auto dstId = static_cast<ptrdiff_t>(verticesAdjSuc.at(dst));

        const auto &graph = considerAdjacency ? graphAdjSuc : graphSuc;
        const auto &vertices = considerAdjacency ? verticesAdjSucRes : verticesSucRes;

        const dijkstra<size_t, size_t> searcher{graph, srcId};

        using result_type = std::optional<std::pair<std::vector<std::ptrdiff_t>, std::ptrdiff_t>>;

        if (result_type result = searcher.search_path(dstId); result.has_value()) {
            for (const auto &res : result.value().first)
                path.push_back(vertices.at(static_cast<const unsigned long>(res)));
        }
    }
    queries[{src, dst}] = path;
    return path;
}

LaneletGraph::LaneletGraph(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    for (const auto &let : lanelets) {
        verticesAdjSuc.insert({let->getId(), graphAdjSuc.add_vertex(let->getId())});
        verticesSuc.insert({let->getId(), graphSuc.add_vertex(let->getId())});
        verticesAdjSucRes.insert({verticesAdjSuc.at(let->getId()), let->getId()});
        verticesSucRes.insert({verticesSuc.at(let->getId()), let->getId()});
    }

    for (const auto &let : lanelets) {
        if (let->getAdjacentLeft().adj != nullptr and !let->getAdjacentLeft().oppositeDir) {
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentLeft().adj->getId())), 4);
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentLeft().adj->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())), 4);
        }
        if (let->getAdjacentRight().adj != nullptr and !let->getAdjacentRight().oppositeDir) {
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentRight().adj->getId())), 4);
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getAdjacentRight().adj->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())), 4);
        }
        for (const auto &suc : let->getSuccessors()) {
            graphAdjSuc.add_edge(static_cast<ptrdiff_t>(verticesAdjSuc.at(let->getId())),
                                 static_cast<ptrdiff_t>(verticesAdjSuc.at(suc->getId())), 1);
            graphSuc.add_edge(static_cast<ptrdiff_t>(verticesSuc.at(let->getId())),
                              static_cast<ptrdiff_t>(verticesSuc.at(suc->getId())), 1);
        }
    }
}
