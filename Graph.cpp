#include "Graph.hpp"

using namespace std;
using std::vector;

namespace ariel {


// private:
//     vector<vector<int>> adjacencyMatrix; // The adjacency matrix of the graph.
//     int numVertices;    // The number of vertices in the graph.

//public:
    /*
    Gets a matrix of neighbors, checks if the input matrix is valid and loads it into the graph.
    */
    void Graph::loadGraph(const vector<vector<int>> &graph) {
        if (graph.size() == 0) {
            throw invalid_argument("The input matrix is empty.");
        }

        for (unsigned int i = 0; i < graph.size(); i++) {
            if (graph[i].size() != graph.size()) {
                throw invalid_argument("Invalid graph: The graph is not a square matrix.");
            }
            for (unsigned int j = 0; j < graph[i].size(); j++) {
                if (i == j && graph[i][j] != 0) {
                    throw invalid_argument("The diagonal of the matrix must be zero.");
                }
            }
        }

        adjacencyMatrix = graph;
        numVertices = graph.size();
    }

    /*
    Prints the graph.
    */
    void Graph::printGraph() {
        for (const auto& row : adjacencyMatrix) {
            for (const auto& value : row) {
                cout << value << " ";
            }
            cout << endl;
        }
    }

    /*
    Inputed graph:
        - Directed graph
            - unweighted graph
            - weighted graph
                - positive weights
                - includes negative weights
                    - negative cycle (in Algotiyhms.cpp)
                    - no negative cycle (in Algotiyhms.cpp)
        
        - Undirected graph
            - unweighted graph
            - weighted graph
                - positive weights
                - includes negative weights
    */


   /*
    Check if the graph is directed.
    If the input matrix is symmetric, the graph is undirected.
   */
    bool Graph::isDirectedGraph() {
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] != adjacencyMatrix[j][i]) {
                    return true;
                }
            }
        }
        return false;
    }

    /*
    Check if the graph is weighted.
    */
    bool Graph::isWeightedGraph() {
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[i][j] != 1) {
                    return true;
                }
            }
        }
        return false;
    }

    /*
    Check if the graph is positive weighted.
    */
    bool Graph::isPositiveWeightedGraph() {
        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] < 0) {
                    return false;
                }
            }
        }
        return true;
    }
    
    /*
    get the adjacency matrix of the graph.
    */
    vector<vector<int>> Graph::getAdjacencyMatrix() {
        return adjacencyMatrix;
    }

    /*
    Get the number of vertices in the graph.
    */
    int Graph::getNumVertices() {
        return numVertices;
    }

    /*
    Get the neighbors of a given vertex.
    @param vertex The vertex to get its neighbors.
    @return A vector of the neighbors of the given vertex.
    */
    vector<size_t> Graph::getNeighbors(size_t vertex) {
        vector<size_t> neighbors;
        for (size_t i = 0; i < numVertices; i++) {
            if (adjacencyMatrix[vertex][i] != 0) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    Graph Graph::reverse() {
        Graph reversedGraph;
        vector<vector<int>> reversedMatrix(static_cast<size_t>(numVertices), vector<int>(static_cast<size_t>(numVertices), 0));

        for (size_t i = 0; i < numVertices; i++) {
            for (size_t j = 0; j < numVertices; j++) {
                reversedMatrix[i][j] = adjacencyMatrix[j][i];
            }
        }
        reversedGraph.loadGraph(reversedMatrix);
        return reversedGraph;
    }

}
//}