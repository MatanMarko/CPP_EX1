#pragma once

#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <vector>

using namespace std;
using std::vector;

namespace ariel {
    class Graph {

    private:
        vector<vector<int>> adjacencyMatrix; // The adjacency matrix of the graph.
        size_t numVertices;    // The number of vertices in the graph.

    public:
        //Graph(); // Constructor.
    
        void loadGraph(const vector<vector<int>> &graph);

        void printGraph();

        bool isDirectedGraph();

        bool isWeightedGraph();

        bool isPositiveWeightedGraph();

        vector<vector<int>> getAdjacencyMatrix();
       
        unsigned int getNumVertices();

        vector<size_t> getNeighbors(size_t vertex);

        Graph reverse();
    };
} // namespace ariel

#endif // GRAPH_HPP
