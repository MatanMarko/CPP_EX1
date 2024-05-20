#include "Algorithms.hpp"
#define NO_PARENT (unsigned int)-1

using namespace std;

namespace ariel {
//class Algorithms {
    //public:

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

    
    QUESTIONS:
    1. Is there a shortest path if the graph contains negative cycle?
    2.

    */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Check if the graph is connected, AKA there is a path between every pair of vertices.
@return 1 if the graph is connected, 0 otherwise.
*/
int Algorithms::isConnected(Graph &graph){
if (graph.isDirectedGraph()){
    return isStronglyConnected(graph);
}
else{
    return isConnectedUndirected(graph);
}
}

/*
Check if the UNDIRECTED graph is connected.
*/
int Algorithms::isConnectedUndirected(Graph &graph){
    unsigned int numVertices = graph.getNumVertices();
    vector<bool> visited(numVertices, false);

    // Perform DFS starting from the first vertex
    DFS(graph, 0, visited);

    // Check if all vertices were visited
    for (bool v : visited) {
        if (!v) {
            return 0; // Graph is not connected
        }
    }

    return 1; // Graph is connected
}

// Depth-first search algorithm
void Algorithms::DFS(Graph &graph, unsigned int vertex, vector<bool>& visited) {
    visited[vertex] = true;

    // Visit all adjacent vertices
    for (unsigned int neighbor : graph.getNeighbors(vertex)) {
        if (!visited[neighbor]) {
            DFS(graph, neighbor, visited);
        }
    }
}

/*
Check if the DIRECTED graph is strongly connected.
*/
int Algorithms::isStronglyConnected(Graph &graph){
    unsigned int numVertices = graph.getNumVertices();
    vector<bool> visited(numVertices, false);
    // Perform DFS starting from the first vertex
    DFS(graph, 0, visited);
    // Check if all vertices were visited
    for (bool v : visited) {
        if (!v) {
            return 0; // Graph is not strongly connected
        }
    }
    // Reverse the graph to perform Kosaraju algorithm
    Graph reversedGraph = graph.reverse();
    // Reset visited array
    visited.assign(numVertices, false);
    // Perform DFS starting from the first vertex in the reversed graph
    DFS(reversedGraph, 0, visited);
    // Check if all vertices were visited
    for (bool v : visited) {
        if (!v) {
            return 0; // Graph is not strongly connected
        }
    }
    return 1; // Graph is strongly connected
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
@return print the cycle if there is one, otherwise return 0.
*/
string Algorithms::isContainsCycle(Graph &graph){
    if (graph.isDirectedGraph()){
        return isContainsCycleDirected(graph);
    }
    else{
        return isContainsCycleUndirected(graph);
    }
}

/*
Check if the DIRECTED graph contains a cycle.
*/
string Algorithms::isContainsCycleDirected(Graph &graph){
    unsigned int numVertices = graph.getNumVertices();
    vector<int> color(numVertices, 0); // 0 - white, 1 - gray, 2 - black
    vector<unsigned int> pi(numVertices, NO_PARENT);
    stack<unsigned int> path;

    for (unsigned int i = 0; i < numVertices; i++){
        if (color[i] == 0){
            if (DFSVisit(graph, i, color, pi, path) == 1){
                string result;
                while (!path.empty()){
                    result += to_string(path.top());
                    path.pop();
                    if (!path.empty()){
                        result += "->";
                    }
                }
                return result;
            }
        }
    }
    return "0";

}
/*
Function that perform DFS on an DIRECTED graph and check if there is a vertex that is already visited, AKA there is a cycle.
*/
int Algorithms::DFSVisit(Graph &graph, unsigned int u, vector<int>& color, vector<unsigned int>& pi, stack<unsigned int>& path){
    color[u] = 1;
    for (unsigned int v : graph.getNeighbors(u)){
        if (color[v] == 0){
            pi[v] = u;
            if (DFSVisit(graph, v, color, pi, path)){
                return true;
            }
        }
        else if (color[v] == 1){
            path.push(v);
            unsigned int i = u;
            while (i != v){
                path.push(i);
                i = pi[i];
            }
            path.push(v);
            return 1;   // There is a cycle
        }
    }
    color[u] = 2;
    return 0;   // No cycle
} 

/*
Check if the UNDIRECTED graph contains a cycle.
*/
string Algorithms::isContainsCycleUndirected(Graph &graph){
    unsigned int numVertices = graph.getNumVertices();
    vector<int> color(numVertices, 0); // 0 - white, 1 - gray, 2 - black
    vector<unsigned int> pi(numVertices, NO_PARENT);
    stack<unsigned int> path;

    for (unsigned int i = 0; i < numVertices; i++){
        if (color[i] == 0){
            if (DFSVisitUndirected(graph, i, color, pi, path) == 1){
                string result;
                while (!path.empty()){
                    result += to_string(path.top());
                    path.pop();
                    if (!path.empty()){
                        result += "->";
                    }
                }
                return result;
            }
        }
    }
    return "0";
}

/*
Function that perform DFS on an UNDIRECTED graph and check if there is a vertex that is already visited, AKA there is a cycle.
*/
int Algorithms::DFSVisitUndirected(Graph &graph, unsigned int u, vector<int>& color, vector<unsigned int>& pi, stack<unsigned int>& path){
    color[u] = 1;
    for (unsigned int v : graph.getNeighbors(u)){
        if (color[v] == 0){
            pi[v] = u;
            if (DFSVisitUndirected(graph, v, color, pi, path)){
                return true;
            }
        }
        else if (color[v] == 1 && pi[u] != v){
            path.push(v);
            unsigned int i = u;
            while (i != v){
                path.push(i);
                i = pi[i];
            }
            path.push(v);
            return 1;   // There is a cycle
        }
    }
    color[u] = 2;
    return 0;   // No cycle
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Calculate the cheapest path between two vertices, if the graph is not weighted return the shortest path.
For a weighted graph with no-negative weights, use Dijkstra's algorithm.
For a weighted graph with negative weights, use Bellman-Ford algorithm.
If there is a negative cycle or there is no path return -1.
*/

string Algorithms::shortestPath(Graph &graph, unsigned int start, unsigned int end){
    if (graph.isWeightedGraph()){
        if (graph.isPositiveWeightedGraph()){
            return Dijkstra(graph, start, end);     // Dijkstra's algorithm for positive weights
        }
        else{
            return BellmanFord(graph, start, end);  // Bellman-Ford algorithm for negative weights
        }
    }
    else{
        return BFS(graph, start, end);
    }
}

string Algorithms::BFS(Graph &graph, unsigned int start, unsigned int end){
    unsigned int numVertices = graph.getNumVertices();
    vector<int> color(numVertices, 0); // 0 - white, 1 - gray, 2 - black
    vector<int> d(numVertices, INT_MAX);
    vector<unsigned int> pi(numVertices, NO_PARENT);
    stack<unsigned int> path;

    color[start] = 1;
    d[start] = 0;
    pi[start] = NO_PARENT;

    queue<unsigned int> q;
    q.push(start);
    while (!q.empty()){
        unsigned int u = q.front();
        q.pop();
        for (unsigned int v : graph.getNeighbors(u)){
            if (color[v] == 0){
                color[v] = 1;
                d[v] = d[u] + 1;
                pi[v] = u;
                q.push(v);
            }
        }
        color[u] = 2;
    }
    if (d[end] == INT_MAX){
        return "-1";
    }
    unsigned int i = end;
    while (i != (unsigned int)-1){    // father is not null
        path.push(i);
        i = pi[i];
    }
    string result;
    while (!path.empty()){
        result += to_string(path.top());
        path.pop();
        if (!path.empty()){
            result += "->";
        }
    }
    return result;
}

string Algorithms::Dijkstra(Graph &graph, unsigned int start, unsigned int end){
    unsigned int numVertices = graph.getNumVertices();
    vector<int> d(numVertices, INT_MAX);
    vector<unsigned int> pi(numVertices, NO_PARENT);
    vector<bool> visited(numVertices, false);
    
    d[start] = 0;
    for (unsigned int i = 0; i < numVertices; i++){
        unsigned int u = minDistance(d, visited);
        visited[u] = true;

        // Relaxation
        for (unsigned int v : graph.getNeighbors(u)){
            int disFromUtoV = graph.getAdjacencyMatrix()[u][v];
            if (!visited[v] && d[u] + disFromUtoV < d[v]){
                d[v] = d[u] + disFromUtoV;
                pi[v] = u;
            }
        }
    }
    if (d[end] == INT_MAX){
        return "-1";
    }

    unsigned int i = end;
    stack<unsigned int> path;
    while (i != (unsigned int)-1){
        path.push(i);
        i = pi[i];
    }
    string result;
    while (!path.empty()){
        result += to_string(path.top());
        path.pop();
        if (!path.empty()){
            result += "->";
        }
    }
    return result;
}

string Algorithms::BellmanFord(Graph &graph, unsigned int start, unsigned int end){
    unsigned int numVertices = graph.getNumVertices();
    vector<int> d(numVertices, INT_MAX);
    vector<unsigned int> pi(numVertices, NO_PARENT);
    d[start] = 0;

    // Relaxation
    for (unsigned int i = 0; i < numVertices - (unsigned int) 1; i++){  // (n - 1) iterations
        for (unsigned int u = 0; u < (unsigned int)numVertices; u++){
            for (unsigned int v : graph.getNeighbors(u)){
                int disFromUtoV = graph.getAdjacencyMatrix()[u][v];
                if (d[u] + disFromUtoV < d[v]){
                    d[v] = d[u] + disFromUtoV;
                    pi[v] = u;
                }
            }
        }
    }

    // Check for negative cycle
    for (unsigned int u = 0; u < (unsigned int)numVertices; u++){  // last iteration
        for (unsigned int v : graph.getNeighbors(u)){
            int disFromUtoV = graph.getAdjacencyMatrix()[u][v];
            if (d[u] + disFromUtoV < d[v]){
                return "-1"; // There is a negative cycle
            }
        }
    }
    if(d[end] == INT_MAX){
        return "-1";
    }

    unsigned int i = end;
    stack<unsigned int> path;
    while (i != (unsigned int)-1){
        path.push(i);
        i = pi[i];
    }
    string result;
    while (!path.empty()){
         result += to_string(path.top());
        path.pop();
        if (!path.empty()){
           result += "->";
        }
    }
    return result;
}

/*
A utility function to find the vertex with minimum distance value,
 from the set of vertices not yet included in shortest path
*/
unsigned int Algorithms::minDistance(vector<int> d, vector<bool> visited){
    int min = INT_MAX;
    unsigned int minIndex;
    for (unsigned int i = 0; i < d.size(); i++){
        if (!visited[i] && d[i] <= min){
            min = d[i];
            minIndex = i;
        }
    }
    return minIndex;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Convert the graph to a double sided graph, if the graph can't be divided return 0.
@param graph - the graph to check if it can be divided to two sets.
@return print the two sets if the graph can be divided, otherwise return 0.
*/          
string Algorithms::isBipartite(Graph &graph){
    unsigned int numVertices = graph.getNumVertices();
    vector<int> color(numVertices, 0); // 0 - white, 1 - red, 2 - blue
    vector<unsigned int> pi(numVertices, NO_PARENT);
    stack<unsigned int> path;

    for (unsigned int i = 0; i < numVertices; i++){
        if (color[i] == 0){
            color[i] = 1;
            if (DFSVisitBipartite(graph, i, color, pi) == 0){
                return "0";
            }
        }
    }

    vector<unsigned int> setA;
    vector<unsigned int> setB;
    for (unsigned int i = 0; i < numVertices; i++){
        if (color[i] == 1){
            setA.push_back(i);
        }
        else{
            setB.push_back(i);
        }
    }

    string result = "The graph is bipartite: A={";
    for (unsigned int i = 0; i < setA.size(); i++){
        result += to_string(setA[i]);
        if (i != setA.size() - 1){
            result += ", ";
        }
    }
    result += "}, B={";
    for (unsigned int i = 0; i < setB.size(); i++){
        result += to_string(setB[i]);
        if (i != setB.size() - 1){
            result += ", ";
        }
    }
    result += "}";
    return result;
}

/*
Function that perform DFS on the graph and check if the graph can be colored with two colors.
*/
int Algorithms::DFSVisitBipartite(Graph &graph, unsigned int u, vector<int>& color, vector<unsigned int>& pi){
    for (unsigned int v : graph.getNeighbors(u)){
        if (color[v] == 0){
            color[v] = 3 - color[u];
            pi[v] = u;
            if (DFSVisitBipartite(graph, v, color, pi) == 0){
                return 0;
            }
        }
        else if (color[v] == color[u]){
            return 0;
        }
    }
    return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Print the negative cycle in the graph if there is one, otherwise print "No negative cycle".
@param graph - the graph to check if there is a negative cycle in it.
@return print the negative cycle if there is one, otherwise print "No negative cycle".
*/
string Algorithms::negativeCycle(Graph graph){
    string result;
    if (!graph.isWeightedGraph()) {
        return "No negative cycle found.";
    }
    if (graph.isPositiveWeightedGraph()) {
        return "No negative cycle found.";
    }

    // Find negative cycle using Floyd-Warshall algorithm
    vector<vector<int>> dist(graph.getNumVertices(), vector<int>(graph.getNumVertices(), INT_MAX));
    vector<vector<unsigned int>> next(graph.getNumVertices(), vector<unsigned int>(graph.getNumVertices(), NO_PARENT));

    // Initialize distance matrix and next matrix
    for (unsigned int i = 0; i < graph.getNumVertices(); i++) {
        dist[i][i] = 0;
        for (unsigned int neighbor : graph.getNeighbors(i)) {
            dist[i][neighbor] = graph.getAdjacencyMatrix()[i][neighbor];
            next[i][neighbor] = neighbor;
        }
    }

    // Floyd-Warshall algorithm
    for (unsigned int k = 0; k < graph.getNumVertices(); k++) {
        for (unsigned int i = 0; i < graph.getNumVertices(); i++) {
            for (unsigned int j = 0; j < graph.getNumVertices(); j++) {
                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX && dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    next[i][j] = next[i][k];
                }
            }
        }
    }

    // Check for negative cycle
    for (unsigned int i = 0; i < graph.getNumVertices(); i++) {
        if (dist[i][i] < 0) {
            unsigned int vertex = i;
            do {
                result += std::to_string(vertex) + "->";
                vertex = next[vertex][i];
            } while (vertex != i);
            result += std::to_string(vertex);
            return result;
        }
    }

    return "No negative cycle found.";
}

}
