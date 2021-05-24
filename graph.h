#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <limits.h>
#include <stack>
#include "edge.h"
#define inf INT_MAX

enum class Stage {
    unprocessed,
    processing,
    processed
};



/* Graph ADT */
template <typename V, typename E>
class Graph {
public:
    /**
     * @brief Returns list of edges, going from given one
     * 
     * @complexity O(V)
     * 
     * @param v - given vertex
     * 
     * @returns list of outgoing edges
     * 
     */
    virtual std::vector<Edge<V, E>> edgesFrom    (Vertex<V>& v)                             = 0;

    /**
     * @brief Returns list of edges, goint to given one
     * 
     * @complexity O(V)
     * 
     * @param v - given vertex
     * 
     * @returns list of ingoing edges
     * 
     */
    virtual std::vector<Edge<V, E>> edgesTo      (Vertex<V>& v)                             = 0;

    /**
     * @brief Adds edge to graph
     * 
     * @complexity O(1)
     * 
     * @param from   - starting vertex of edge
     * @param to     - ending vertex of edge
     * @param weight - weight of edge
     * 
     * @returns reference to added edge
     */
    virtual Edge<V, E>&             addEdge      (Vertex<V>& from, Vertex<V>& to, E weight) = 0;

    /**
     * @brief Finds edge in graph
     * 
     * @complexity O(V)
     * 
     * @param from_value - value of starting vertex of edge
     * @param to_value   - value of ending vertex of graph
     * 
     * @throws std::runtime_error if edge does not exist
     * 
     * @return reference to edge
     */
    virtual Edge<V, E>&             findEdge     (V from_value, V to_value)                 = 0;

    /**
     * @brief Determines existence of edge in graph
     * 
     * @complexity O(1)
     * 
     * @param from - reference to starting vertex in edge
     * @param to   - reference to ending vertex in edge
     * 
     * @returns true if edge exists, false otherwise
     */
    virtual bool                    hasEdge      (Vertex<V>& from, Vertex<V>& to)           = 0;

    /**
     * @brief Remove vertex from graph
     * 
     * @complexity O(V^2)
     * 
     * @param v - reference to vertex
     */
    virtual void                    removeVertex (Vertex<V>& v)                             = 0;

    /**
     * @brief Finds vertex in graph
     * 
     * @complexity O(V)
     * 
     * @param value - value of required vertex
     * 
     * @throws std::runtime_error if vertex is not in the graph
     * 
     * @returns reference to vertex
     */
    virtual Vertex<V>&              findVertex   (V value)                                  = 0;

    /**
     * @brief Removes edge from graph 
     * 
     * @complexity O(1)
     * 
     * @param e - reference to edge
     */
    virtual void                    removeEdge   (Edge<V, E>& e)                            = 0;

    /**
     * @brief Adds vertex to graph
     * 
     * @complexity O(V)
     * 
     * @param value - value of vertex
     * 
     * @returns reference to created vertex
     */
    virtual Vertex<V>&              addVertex    (V value)                                  = 0;

    /**
     * @brief Changes directions of all edges in graph
     * 
     */
    virtual void                    transpose    ()                                         = 0;

    /**
     * @brief Determines if graph if acyclic; Prints cycle if there is one
     */
    virtual void                    isAcyclic    ()                                         = 0;

    /**
     * @brief Computes shortes path between two vertices
     * 
     * @param start  - starting vertex
     * @param finish - ending vertex
     */
    virtual void                    dijkstra     (Vertex<V>& start, Vertex<V>& finish)        = 0;
};


template <typename V, typename E>
class AdjacencyMatrixGraph : public Graph<V, E> {
public:
    std::vector<std::vector<Edge<V, E>*>> matrix;
    std::vector<Vertex<V>>                vertices;

    AdjacencyMatrixGraph() = default;

    ~AdjacencyMatrixGraph() {
        for (auto vector : matrix) {
            for (auto element : vector) {
                delete element;
            }
            vector.clear();
        }
    }

    Vertex<V>& addVertex(V value) {
        vertices.push_back(Vertex<V> (matrix.size(), value));

        if (!matrix.empty()) {
            for (size_t i = 0; i < matrix.size(); ++i) {
                matrix[i].push_back(nullptr);
            }
            matrix.push_back(std::vector<Edge<V, E>*> (vertices.size(), nullptr));
        } else {
            matrix.push_back(std::vector<Edge<V, E>*> {nullptr});
        }

        return vertices.back();
    }

    void removeVertex(Vertex<V>& v) {
        /* Remove all incoming edges to vertex */
        for (size_t i = 0; i < vertices.size(); ++i) {
            delete matrix[i][v.id];
            matrix[i].erase(matrix[i].begin() + v.id);
        }

        /* Remove all outcoming edges from vertex */
        for (size_t i = 0; i < vertices.size() - 1; ++i) {
            delete matrix[v.id][i];
        }
        matrix[v.id].clear();
        matrix.erase(matrix.begin() + v.id);

        /* Remove vertex */
        vertices.erase(vertices.begin() + v.id);

        /* Update id-s of vertices */
        for (size_t i = 0; i < vertices.size(); ++i) {
            vertices[i].id = i;
        }

        /* Update id-s of vertices in edges */
        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t j = 0; j < vertices.size(); ++j) {
                if (matrix[i][j]) {
                    if (matrix[i][j]->from.id > v.id) matrix[i][j]->from.id--;
                    if (matrix[i][j]->to.id > v.id) matrix[i][j]->to.id--;
                }
            }
        }
    }

    Edge<V, E>& addEdge(Vertex<V>& from, Vertex<V>& to, E weight) {
        return *(matrix[from.id][to.id] = new Edge<V, E> (from, to, weight));
    }

    void removeEdge(Edge<V, E>& e) {
        matrix[e.from.id][e.to.id] = nullptr;
    }

    std::vector<Edge<V, E>> edgesFrom(Vertex<V>& v) {
        std::vector<Edge<V, E>> output;

        for (size_t i = 0; i < vertices.size(); ++i) {
            if (matrix[v.id][i]) output.push_back(*matrix[v.id][i]);
        }

        return output;
    }

    std::vector<Edge<V, E>> edgesTo(Vertex<V>& v) {
        std::vector<Edge<V, E>> output;

        for (size_t i = 0; i < vertices.size(); ++i) {
            if (matrix[i][v.id]) output.push_back(*matrix[v.id][i]);
        }

        return output;
    }

    Vertex<V>& findVertex(V value) {
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (vertices[i].value == value) return vertices[i];
        }
        throw std::runtime_error("Vertex not found");
    }

    Edge<V, E>& findEdge(V from_value, V to_value) {
        Vertex<V> vertex_from, vertex_to;

        vertex_from = findVertex(from_value);
        vertex_to = findVertex(to_value);

        if (matrix[vertex_from.id][vertex_to.id]) {
            return *matrix[vertex_from.id][vertex_to.id];
        }

        throw std::runtime_error("Edge not found");
    }

    bool hasEdge(Vertex<V>& vertex_from, Vertex<V>& vertex_to) {
        return (matrix[vertex_from.id][vertex_to.id] != nullptr);
    }

    void transpose() {
        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t j = i + 1; j < vertices.size(); ++j) {
                std::swap(matrix[i][j], matrix[j][i]);
                if (matrix[i][j]) std::swap(matrix[i][j]->from, matrix[i][j]->to);
                if (matrix[j][i]) std::swap(matrix[j][i]->from, matrix[j][i]->to);
            }
        }
    }

    void isAcyclic() {
        std::vector<Stage> visited (vertices.size(), Stage::unprocessed);
        std::vector<int>   cycle;
        std::stack<int>    stack;
        Edge<V, E>*        edge;
        bool               acyclic = true;
        int                current;
        E                  weight  = 0;

        /* Standard iterative DFS search with additional check to detect cycles */
        for (size_t i = 0; i < vertices.size(); ++i) {
            /* Push unvisited vertex onto the stack */
            if (visited[i] == Stage::unprocessed) stack.push(i);

            while (!stack.empty()) {
                current = stack.top();
                visited[current] = Stage::processing;

                /* Iterating over all edges, which are incident to current vertex */
                for (size_t j = 0; j < vertices.size(); ++j) {
                    edge = matrix[current][j];
                    if (edge) {
                        /* Push unvisited vertices onto the stack */
                        if (visited[edge->to.id] == Stage::unprocessed) {
                            stack.push(edge->to.id);
                            break;
                        /* If from current vertex we see a path to vertex,
                         * which is still processing - it means that we have found a cycle */
                        } else if (visited[edge->to.id] == Stage::processing) {
                            /* Until our stack is empty or until we found a sequence like: A->B->C->A
                             * we pop elements from stack to vector "cycle" */
                            while (!stack.empty() && stack.top() != edge->to.id) {
                                cycle.push_back(stack.top());
                                stack.pop();
                            }

                            /* Push final element to the vector "cycle" */
                            cycle.push_back(edge->to.id);
                            acyclic = false;
                            break;
                        }
                    }
                }
                /* If we have already discovered a cycle, then we do not need to continue calculations */
                if (!acyclic) break;

                /* If we found a "dead end" vertex, then we mark it as processed and pop from the stack */
                if (stack.top() == current) {
                    visited[current] = Stage::processed;
                    stack.pop();
                }
            }
            if (!acyclic) break;
        }

        if (acyclic) std::cout << "ACYCLIC\n";

        else {
            /* Calculate total weight of cycle */
            for (size_t i = 0; i < cycle.size(); ++i) {
                weight += matrix[ cycle[ (i + 1) % cycle.size() ] ][ cycle[i] ]->weight;
            }

            std::cout << "Total weight of cycle: " << weight << "\n";


            /* Print vertices in a way they from a cycle */
            std::cout << "Cycle: ";
            for (int i = (cycle.size() - 1); i >= 0; i--) {
                if (i > 0)
                    std::cout << vertices[ cycle[i] ].value << " -> ";
                else
                    std::cout << vertices[ cycle[i] ].value << "\n";
            }
        }

        delete edge;
    }


    void dijkstra(Vertex<V>& start, Vertex<V>& finish) {
        std::vector<Vertex<V>> path;
        std::vector<bool>      visited (vertices.size(), false);
        std::vector<E>         distance (vertices.size(), inf);
        std::vector<V>         previous (vertices.size(), -1);
        Vertex<V>              current = start;

        /* Initialise starting conditions */
        distance[start.id] = 0;
        visited[start.id]  = true;

        for (size_t j = 0; j < vertices.size(); ++j) {
            int min_distance = inf;

            /* Find unvisited vertex with minimal distance to it */
            for (size_t i = 0; i < vertices.size(); ++i) {
                if (!visited[i] && (distance[i] < min_distance)) {
                    min_distance = distance[i];
                    current = vertices[i];
                }
            }

            /* Iterate through edges, which are incident to this vertex */
            for (size_t i = 0; i < this->vertices.size(); ++i) {
                /* We are interested only in existing edges */
                if (matrix[current.id][i]) {
                    /* true if our vertex is unvisited AND we can update distance to it */
                    if (!visited[i] && (distance[i] > (distance[current.id] + matrix[current.id][i]->weight))) {

                        /* Update distance to this vertex */
                        distance[i] = distance[current.id] + matrix[current.id][i]->weight;
                        /* Save predecessor of this vertex, so that we can reconstruct our shortes path later */
                        previous[i] = current.id;
                    }
                }
            }
            /* Visit this vertex */
            visited[current.id] = true;
        }

        /* Check, whether there exist a path to our finish vertex */
        if (distance[finish.id] != inf) {

            int parent    = previous[finish.id]; // Predecessor of finish vertex
            path.push_back(finish);              // Push final vertex to "path" vector

            /* while vertex has a predecessor (previous[v] = -1 means that there is no predecessor to vertex "v") */
            while (parent != -1) {
                /* Push this vertex to "path" vector */
                path.push_back(vertices[parent]);
                /* Go to it's predecessor */
                parent = previous[parent];
            }

            /* We have to reverse back our "path" vector because we reversed the order
             * of path, when we were poping from the stack */
            std::reverse(path.begin(), path.end());
            std::cout << "Vertices in path " << path.size() << "\nTotal distance: " << distance[finish.id] << "\n";

            std::cout << "Path: ";
            for (size_t j = 0; j < path.size(); ++j) {   
                if (j < path.size() - 1)
                    std::cout << path[j].value << " -> ";
                else
                    std::cout << path[j].value << "\n";
            }
        } else std::cout << "IMPOSSIBLE\n";
    }

};

#endif