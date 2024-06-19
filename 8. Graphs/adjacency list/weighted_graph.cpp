#include <iostream>
#include <map>
#include <list>
#include <queue>
#include <stack>
#include <set>
#include <vector>

using namespace std;

// Struct to represent an edge with weight
struct Edge {
    int destination;
    int weight;

    Edge(int dest, int w) : destination(dest), weight(w) {}
};

// Class for representing a weighted graph using adjacency list representation
class WeightedGraph {
private:
    map<int, list<Edge>> adjList;
    bool isDirected;

public:
    // Constructor
    WeightedGraph(bool directed) : isDirected(directed) {}

    // Add vertex to the graph
    void addVertex(int vertex) {
        if (adjList.find(vertex) == adjList.end()) {
            adjList[vertex] = list<Edge>();
        }
    }

    // Remove vertex from the graph
    void removeVertex(int vertex) {
        adjList.erase(vertex);
        for (auto& pair : adjList) {
            auto& neighbors = pair.second;
            neighbors.remove_if([vertex](const Edge& e) { return e.destination == vertex; });
        }
    }

    // Add weighted edge to the graph
    void addEdge(int u, int v, int weight) {
        addVertex(u);
        addVertex(v);
        adjList[u].push_back(Edge(v, weight));
        if (!isDirected && u != v) {
            adjList[v].push_back(Edge(u, weight));  // Avoid self-loop for undirected graphs
        }
    }

    // Remove weighted edge from the graph
    void removeEdge(int u, int v) {
        if (adjList.find(u) != adjList.end()) {
            adjList[u].remove_if([v](const Edge& e) { return e.destination == v; });
        }
        if (!isDirected && adjList.find(v) != adjList.end()) {
            adjList[v].remove_if([u](const Edge& e) { return e.destination == u; });
        }
    }

    // Perform Breadth-First Search (BFS) from a given vertex
    vector<int> BFS(int startVertex) {
        vector<int> traversal;
        set<int> visited;
        queue<int> q;

        q.push(startVertex);
        visited.insert(startVertex);

        while (!q.empty()) {
            int vertex = q.front();
            q.pop();
            traversal.push_back(vertex);

            for (const auto& edge : adjList[vertex]) {
                int destination = edge.destination;
                if (visited.find(destination) == visited.end()) {
                    q.push(destination);
                    visited.insert(destination);
                }
            }
        }
        return traversal;
    }

    // Perform Depth-First Search (DFS) from a given vertex (iterative)
    vector<int> DFS(int startVertex) {
        vector<int> traversal;
        set<int> visited;
        stack<int> s;

        s.push(startVertex);
        visited.insert(startVertex);

        while (!s.empty()) {
            int vertex = s.top();
            s.pop();
            traversal.push_back(vertex);

            for (const auto& edge : adjList[vertex]) {
                int destination = edge.destination;
                if (visited.find(destination) == visited.end()) {
                    s.push(destination);
                    visited.insert(destination);
                }
            }
        }
        return traversal;
    }

    // Perform Depth-First Search (DFS) from a given vertex (recursive)
    vector<int> DFSRec(int startVertex) {
        vector<int> traversal;
        set<int> visited;
        DFSRecHelper(startVertex, visited, traversal);
        return traversal;
    }

    // Display the graph
    void display() {
        for (auto& pair : adjList) {
            int vertex = pair.first;
            list<Edge>& neighbors = pair.second;
            cout << vertex << ": ";
            for (const auto& edge : neighbors) {
                cout << "(" << edge.destination << "," << edge.weight << ") ";
            }
            cout << endl;
        }
    }

    // Check if the graph is directed
    bool isGraphDirected() const {
        return isDirected;
    }

private:
    // Utility function for recursive DFS
    void DFSRecHelper(int vertex, set<int>& visited, vector<int>& traversal) {
        visited.insert(vertex);
        traversal.push_back(vertex);

        for (const auto& edge : adjList[vertex]) {
            int destination = edge.destination;
            if (visited.find(destination) == visited.end()) {
                DFSRecHelper(destination, visited, traversal);
            }
        }
    }
};

int main() {
    // Create an undirected weighted graph
    WeightedGraph undirectedGraph(false);
    undirectedGraph.addVertex(1);
    undirectedGraph.addVertex(2);
    undirectedGraph.addVertex(3);
    undirectedGraph.addVertex(4);
    undirectedGraph.addVertex(5);
    undirectedGraph.addVertex(6);

    undirectedGraph.addEdge(1, 2, 2);
    undirectedGraph.addEdge(1, 3, 4);
    undirectedGraph.addEdge(2, 4, 1);
    undirectedGraph.addEdge(3, 4, 3);
    undirectedGraph.addEdge(3, 5, 7);
    undirectedGraph.addEdge(4, 6, 5);
    undirectedGraph.addEdge(5, 6, 2);

    cout << "Undirected Weighted Graph:" << endl;
    undirectedGraph.display();

    cout << endl;

    cout << "BFS from vertex 1: ";
    vector<int> bfsTraversal = undirectedGraph.BFS(1);
    for (int vertex : bfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (iterative) from vertex 1: ";
    vector<int> dfsTraversal = undirectedGraph.DFS(1);
    for (int vertex : dfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (recursive) from vertex 1: ";
    vector<int> dfsRecTraversal = undirectedGraph.DFSRec(1);
    for (int vertex : dfsRecTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    // Remove a vertex and an edge and display the graph again
    undirectedGraph.removeEdge(1, 2);
    undirectedGraph.removeVertex(4);

    cout << "\nUndirected Weighted Graph after removing edge (1, 2) and vertex 4:" << endl;
    undirectedGraph.display();

    cout << endl;

    // Create a directed weighted graph
    WeightedGraph directedGraph(true);
    directedGraph.addVertex(1);
    directedGraph.addVertex(2);
    directedGraph.addVertex(3);
    directedGraph.addVertex(4);
    directedGraph.addVertex(5);
    directedGraph.addVertex(6);

    directedGraph.addEdge(1, 2, 2);
    directedGraph.addEdge(1, 3, 4);
    directedGraph.addEdge(2, 4, 1);
    directedGraph.addEdge(3, 4, 3);
    directedGraph.addEdge(3, 5, 7);
    directedGraph.addEdge(4, 6, 5);
    directedGraph.addEdge(5, 6, 2);

    cout << "Directed Weighted Graph:" << endl;
    directedGraph.display();

    cout << endl;

    cout << "BFS from vertex 1: ";
    bfsTraversal = directedGraph.BFS(1);
    for (int vertex : bfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (iterative) from vertex 1: ";
    dfsTraversal = directedGraph.DFS(1);
    for (int vertex : dfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (recursive) from vertex 1: ";
    dfsRecTraversal = directedGraph.DFSRec(1);
    for (int vertex : dfsRecTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    // Remove a vertex and an edge and display the graph again
    directedGraph.removeEdge(1, 2);
    directedGraph.removeVertex(4);

    cout << "\nDirected Weighted Graph after removing edge (1, 2) and vertex 4:" << endl;
    directedGraph.display();

    return 0;
}
