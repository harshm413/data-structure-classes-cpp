#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <stack>
#include <set>

using namespace std;

// Struct to represent an edge with weight
struct Edge {
    int destination;
    int weight;

    Edge(int dest, int w) : destination(dest), weight(w) {}
};

class WeightedGraphMatrix {
private:
    vector<vector<Edge*>> adjacencyMatrix;
    int numVertices;
    bool isDirected;

public:
    // Constructor
    WeightedGraphMatrix(int n, bool directed) : numVertices(n), isDirected(directed) {
        // Initialize adjacency matrix with nullptrs (no edges initially)
        adjacencyMatrix.resize(n, vector<Edge*>(n, nullptr));
    }

    // Destructor to free memory used by edges
    ~WeightedGraphMatrix() {
        for (int i = 0; i < numVertices; ++i) {
            for (int j = 0; j < numVertices; ++j) {
                delete adjacencyMatrix[i][j];
            }
        }
    }

    // Add edge to the graph
    void addEdge(int u, int v, int weight) {
        if (adjacencyMatrix[u][v] == nullptr) {
            adjacencyMatrix[u][v] = new Edge(v, weight);
        } else {
            // Update weight if edge already exists
            adjacencyMatrix[u][v]->weight = weight;
        }
        if (!isDirected && u != v) {
            if (adjacencyMatrix[v][u] == nullptr) {
                adjacencyMatrix[v][u] = new Edge(u, weight);
            } else {
                // Update weight if edge already exists
                adjacencyMatrix[v][u]->weight = weight;
            }
        }
    }

    // Remove edge from the graph
    void removeEdge(int u, int v) {
        delete adjacencyMatrix[u][v];
        adjacencyMatrix[u][v] = nullptr;
        if (!isDirected) {
            delete adjacencyMatrix[v][u];
            adjacencyMatrix[v][u] = nullptr;
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

            for (int v = 0; v < numVertices; ++v) {
                if (adjacencyMatrix[vertex][v] != nullptr && visited.find(v) == visited.end()) {
                    q.push(v);
                    visited.insert(v);
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

            for (int v = 0; v < numVertices; ++v) {
                if (adjacencyMatrix[vertex][v] != nullptr && visited.find(v) == visited.end()) {
                    s.push(v);
                    visited.insert(v);
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
        for (int i = 0; i < numVertices; ++i) {
            cout << i << ": ";
            for (int j = 0; j < numVertices; ++j) {
                if (adjacencyMatrix[i][j] != nullptr) {
                    cout << "(" << adjacencyMatrix[i][j]->destination << "," << adjacencyMatrix[i][j]->weight << ") ";
                }
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

        for (int v = 0; v < numVertices; ++v) {
            if (adjacencyMatrix[vertex][v] != nullptr && visited.find(v) == visited.end()) {
                DFSRecHelper(v, visited, traversal);
            }
        }
    }
};

int main() {
    // Example usage
    WeightedGraphMatrix undirectedGraph(7, false);
    undirectedGraph.addEdge(0, 1, 2);
    undirectedGraph.addEdge(0, 2, 4);
    undirectedGraph.addEdge(1, 3, 1);
    undirectedGraph.addEdge(2, 3, 3);
    undirectedGraph.addEdge(2, 4, 7);
    undirectedGraph.addEdge(3, 5, 5);
    undirectedGraph.addEdge(4, 5, 2);
    undirectedGraph.addEdge(5, 6, 1);

    cout << "Undirected Weighted Graph:" << endl;
    undirectedGraph.display();

    cout << endl;

    cout << "BFS from vertex 0: ";
    vector<int> bfsTraversal = undirectedGraph.BFS(0);
    for (int vertex : bfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (iterative) from vertex 0: ";
    vector<int> dfsTraversal = undirectedGraph.DFS(0);
    for (int vertex : dfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (recursive) from vertex 0: ";
    vector<int> dfsRecTraversal = undirectedGraph.DFSRec(0);
    for (int vertex : dfsRecTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    // Remove an edge and display the graph again
    undirectedGraph.removeEdge(2, 3);

    cout << "\nUndirected Weighted Graph after removing edge (2, 3):" << endl;
    undirectedGraph.display();

    cout << endl;

    // Example of a directed graph
    WeightedGraphMatrix directedGraph(6, true);
    directedGraph.addEdge(0, 1, 2);
    directedGraph.addEdge(0, 2, 4);
    directedGraph.addEdge(1, 3, 1);
    directedGraph.addEdge(2, 3, 3);
    directedGraph.addEdge(2, 4, 7);
    directedGraph.addEdge(3, 5, 5);
    directedGraph.addEdge(4, 5, 2);

    cout << "Directed Weighted Graph:" << endl;
    directedGraph.display();

    cout << endl;

    cout << "BFS from vertex 0: ";
    bfsTraversal = directedGraph.BFS(0);
    for (int vertex : bfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (iterative) from vertex 0: ";
    dfsTraversal = directedGraph.DFS(0);
    for (int vertex : dfsTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    cout << "DFS (recursive) from vertex 0: ";
    dfsRecTraversal = directedGraph.DFSRec(0);
    for (int vertex : dfsRecTraversal) {
        cout << vertex << " ";
    }
    cout << endl;

    // Remove an edge and display the graph again
    directedGraph.removeEdge(0, 1);

    cout << "\nDirected Weighted Graph after removing edge (0, 1):" << endl;
    directedGraph.display();

    return 0;
}
