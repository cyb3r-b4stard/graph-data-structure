/**
 * @name main
 * 
 * @brief 
 *  
 * @author - Ivan Lomikovskiy github.com/cyb3r-b4stard
 * 
 * @project DSA 3 Assignment
 */
#include <string>
#include "graph.h"

void graphOperations();
void calculateShortestPath();

int main()
{
    graphOperations();
    calculateShortestPath();
    return 0;
}

void calculateShortestPath() {
    AdjacencyMatrixGraph<int, int> graph;
    int n, m, from, to, length, start, finish;
    Vertex<int> vertex_from, vertex_to;
 
    std::cin >> n >> m;
 
    for (int i = 0; i < n; ++i) {
        graph.addVertex(i+1);
    }
    for (int i = 0; i < m; ++i) {
        std::cin >> from >> to >> length;
        vertex_from = graph.findVertex(from);
        vertex_to = graph.findVertex(to);
        graph.addEdge(vertex_from, vertex_to, length);
    }
    std::cin >> start >> finish;
    vertex_from = graph.findVertex(start);
    vertex_to   = graph.findVertex(finish);
    graph.dijkstra(vertex_from, vertex_to);

}
void graphOperations() {
    AdjacencyMatrixGraph<std::string, int> graph;
    Vertex<std::string> vertex, vertex_from, vertex_to;

    int weight;
    std::string instruction, name, from_name, to_name;
    Edge<std::string, int> edge;

    while (std::cin >> instruction) {

        if (instruction == "ADD_VERTEX") {
            std::cin >> name;
            graph.addVertex(name);
        } else if (instruction == "REMOVE_VERTEX") {
            std::cin >> name;
            try {
                vertex = graph.findVertex(name);
                graph.removeVertex(vertex);
            } catch (std::exception) {}
        } else if (instruction == "ADD_EDGE") {
            std::cin >> from_name >> to_name >> weight;
            try {
                vertex_from = graph.findVertex(from_name);
                vertex_to = graph.findVertex(to_name);
                graph.addEdge(vertex_from, vertex_to, weight);
            }  catch (std::exception) {}
        } else if (instruction == "REMOVE_EDGE") {
            std::cin >> from_name >> to_name;
            try {
                edge = graph.findEdge(from_name, to_name);
                graph.removeEdge(edge);
            }  catch (std::exception) {}
        } else if (instruction == "HAS_EDGE") {
            std::cin >> from_name >> to_name;
            try {
                vertex_from = graph.findVertex(from_name);
                vertex_to = graph.findVertex(to_name);
                if (graph.hasEdge(vertex_from, vertex_to)) std::cout << "TRUE\n";
                else std::cout << "FALSE\n";
            }  catch (std::exception) {
                std::cout << "FALSE\n";
            }
        } else if (instruction == "TRANSPOSE") {
            graph.transpose();
        } else if (instruction == "IS_ACYCLIC") {
            graph.isAcyclic();
        }
    }
}