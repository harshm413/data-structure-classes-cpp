#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <stack>
#include <set>

using namespace std;

class WeightedGraphMatrix {
private:
    vector<vector<int>> adjacencyMatrix;
    int numVertices;
    bool isDirected;

public:
    // Constructor
    WeightedGraphMatrix(int n, bool directed) : numVertices(n), isDirected(directed) {
        // Initialize adjacency matrix with INT_MAX (representing no edge initially)
        adjacencyMatrix.resize(n, vector<int>(n, INT_MAX));
        for (int i = 0; i < n; ++i) {
            adjacencyMatrix[i][i] = 0;  // Distance to self is 0
        }
    }

    // Add edge to the graph
    void addEdge(int u, int v, int weight) {
        adjacencyMatrix[u][v] = weight;
        if (!isDirected && u != v) {
            adjacencyMatrix[v][u] = weight;
        }
    }

    // Remove edge from the graph
    void removeEdge(int u, int v) {
        adjacencyMatrix[u][v] = INT_MAX;
        if (!isDirected) {
            adjacencyMatrix[v][u] = INT_MAX;
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

            for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
                if (adjacencyMatrix[vertex][neighbor] != INT_MAX && visited.find(neighbor) == visited.end()) {
                    q.push(neighbor);
                    visited.insert(neighbor);
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

            for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
                if (adjacencyMatrix[vertex][neighbor] != INT_MAX && visited.find(neighbor) == visited.end()) {
                    s.push(neighbor);
                    visited.insert(neighbor);
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
                if (adjacencyMatrix[i][j] != INT_MAX && adjacencyMatrix[i][j] != 0) {
                    cout << "(" << j << "," << adjacencyMatrix[i][j] << ") ";
                }
            }
            cout << endl;
        }
    }

    // Display the adjacency matrix
    void displayMatrix() {
        for (int i = 0; i < numVertices; ++i) {
            for (int j = 0; j < numVertices; ++j) {
                if (adjacencyMatrix[i][j] == INT_MAX) {
                    cout << "INF ";
                } else {
                    cout << adjacencyMatrix[i][j] << " ";
                }
            }
            cout << endl;
        }
    }

    // Check if the graph is directed
    bool isGraphDirected() const {
        return isDirected;
    }

    // Floyd-Warshall Algorithm
    vector<vector<int>> floydWarshall() {
        vector<vector<int>> dist = adjacencyMatrix;

        // Ensure the diagonal elements are 0
        // for (int i = 0; i < numVertices; ++i) {
        //     dist[i][i] = 0;
        // }

        for (int intermediate = 0; intermediate < numVertices; ++intermediate) {
            for (int start = 0; start < numVertices; ++start) {
                for (int end = 0; end < numVertices; ++end) {
                    if (dist[start][intermediate] != INT_MAX && dist[intermediate][end] != INT_MAX && dist[start][intermediate] + dist[intermediate][end] < dist[start][end]) {
                        dist[start][end] = dist[start][intermediate] + dist[intermediate][end];
                    }
                }
            }
        }

        return dist;
    }

    // Check if the graph contains a cycle
    bool hasCycle() {
        vector<bool> visited(numVertices, false);
        vector<bool> recStack(numVertices, false); // To track nodes in the current recursion stack

        for (int i = 0; i < numVertices; ++i) {
            if (!visited[i]) {
                if (dfsCycleDetection(i, visited, recStack, -1)) {
                    return true;
                }
            }
        }
        return false;
    }

private:
    // Utility function for recursive DFS
    void DFSRecHelper(int vertex, set<int>& visited, vector<int>& traversal) {
        visited.insert(vertex);
        traversal.push_back(vertex);

        for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
            if (adjacencyMatrix[vertex][neighbor] != INT_MAX && visited.find(neighbor) == visited.end()) {
                DFSRecHelper(neighbor, visited, traversal);
            }
        }
    }

    // Utility function for detecting cycles using DFS
    bool dfsCycleDetection(int vertex, vector<bool>& visited, vector<bool>& recStack, int parent) {
        visited[vertex] = true;
        recStack[vertex] = true;

        for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
            if (vertex != neighbor && adjacencyMatrix[vertex][neighbor] != INT_MAX) {
                if (!visited[neighbor]) {
                    if (dfsCycleDetection(neighbor, visited, recStack, vertex)) {
                        return true;
                    }
                } else if (recStack[neighbor] && (isDirected || neighbor != parent)) {
                    return true;
                }
            }
        }

        recStack[vertex] = false;
        return false;
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

    WeightedGraphMatrix directedGraph2(4, true);
    directedGraph2.addEdge(0, 1, 8);
    directedGraph2.addEdge(0, 3, 1);
    directedGraph2.addEdge(1, 2, 1);
    directedGraph2.addEdge(3, 1, 2);
    directedGraph2.addEdge(3, 2, 9);
    directedGraph2.addEdge(2, 0, 4);

    cout << "\nDirected Weighted Graph:" << endl;
    directedGraph2.display();

    cout << endl;

    cout << "Directed Weighted Matrix:" << endl;
    directedGraph2.displayMatrix();

    cout << endl;

    // Floyd-Warshall Algorithm
    cout << "Floyd-Warshall Algorithm result:" << endl;
    vector<vector<int>> fwResult = directedGraph2.floydWarshall();

    for (int i = 0; i < fwResult.size(); ++i) {
        for (int j = 0; j < fwResult[i].size(); ++j) {
            if (fwResult[i][j] == INT_MAX) {
                cout << "INF ";
            } else {
                cout << fwResult[i][j] << " ";
            }
        }
        cout << endl;
    }

    // New undirected weighted graph with a cycle
    WeightedGraphMatrix undirectedWeightedGraphForCycle(4, false);
    undirectedWeightedGraphForCycle.addEdge(0, 1, 2);
    undirectedWeightedGraphForCycle.addEdge(1, 2, 3);
    undirectedWeightedGraphForCycle.addEdge(2, 3, 4);
    //undirectedWeightedGraphForCycle.addEdge(3, 0, 5);

    cout << "\nUndirected Weighted Graph For Cycle Detection:" << endl;
    undirectedWeightedGraphForCycle.display();

    cout << endl;

    cout << "Does the undirected weighted graph contain a cycle? " << (undirectedWeightedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;

    // New directed weighted graph with a cycle
    WeightedGraphMatrix directedWeightedGraphForCycle(4, true);
    directedWeightedGraphForCycle.addEdge(0, 1, 2);
    directedWeightedGraphForCycle.addEdge(1, 2, 3);
    directedWeightedGraphForCycle.addEdge(2, 3, 4);
    directedWeightedGraphForCycle.addEdge(3, 1, 5);

    cout << "\nDirected Weighted Graph For Cycle Detection:" << endl;
    directedWeightedGraphForCycle.display();

    cout << endl;

    cout << "Does the directed weighted graph contain a cycle? " << (directedWeightedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;

    return 0;
}
