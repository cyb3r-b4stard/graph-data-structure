#ifndef EDGE_H
#define EDGE_H

#include "vertex.h"

template <typename V, typename E>
class Edge {
public:
    Vertex<V> from, to;
    E         weight;

    Edge(Vertex<V> _from, Vertex<V> _to, E weight) : from(_from), to(_to), weight(weight) {}
    Edge() = default;

    bool operator==(const Edge& edge) {
        return (this->from == edge.from && this->to == edge.to
                && this->weight == edge.weight);
    }
};

#endif