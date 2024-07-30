#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <stack>

using namespace std;

class UnweightedGraphMatrix {
private:
    vector<vector<bool>> adjacencyMatrix;
    int numVertices;
    bool isDirected;

public:
    // Constructor
    UnweightedGraphMatrix(int n, bool directed) : numVertices(n), isDirected(directed) {
        // Initialize adjacency matrix with size n x n, initially all false (no edges)
        adjacencyMatrix.resize(n, vector<bool>(n, false));
    }

    // Add edge to the graph
    void addEdge(int u, int v) {
        adjacencyMatrix[u][v] = true;
        if (!isDirected) {
            adjacencyMatrix[v][u] = true;  // For undirected graphs
        }
    }

    // Remove edge from the graph
    void removeEdge(int u, int v) {
        adjacencyMatrix[u][v] = false;
        if (!isDirected) {
            adjacencyMatrix[v][u] = false;  // For undirected graphs
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
                if (adjacencyMatrix[vertex][neighbor] && visited.find(neighbor) == visited.end()) {
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
                if (adjacencyMatrix[vertex][neighbor] && visited.find(neighbor) == visited.end()) {
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
                if (adjacencyMatrix[i][j]) {
                    cout << j << " ";
                }
            }
            cout << endl;
        }
    }

    // Check if the graph is directed
    bool isGraphDirected() const {
        return isDirected;
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
            if (adjacencyMatrix[vertex][neighbor] && visited.find(neighbor) == visited.end()) {
                DFSRecHelper(neighbor, visited, traversal);
            }
        }
    }

    // Utility function for detecting cycles using DFS
    bool dfsCycleDetection(int vertex, vector<bool>& visited, vector<bool>& recStack, int parent) {
        visited[vertex] = true;
        recStack[vertex] = true;

        for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
            if (adjacencyMatrix[vertex][neighbor]) {
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
    UnweightedGraphMatrix undirectedGraph(7, false);
    undirectedGraph.addEdge(0, 1);
    undirectedGraph.addEdge(0, 2);
    undirectedGraph.addEdge(1, 3);
    undirectedGraph.addEdge(2, 3);
    undirectedGraph.addEdge(2, 4);
    undirectedGraph.addEdge(3, 5);
    undirectedGraph.addEdge(4, 5);
    undirectedGraph.addEdge(5, 6);

    cout << "Undirected Graph:" << endl;
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

    // Remove a vertex and an edge and display the graph again
    undirectedGraph.removeEdge(0, 1);

    cout << "\nUndirected Graph after removing edge (0, 1):" << endl;
    undirectedGraph.display();

    cout << endl;

    // Directed graph example
    UnweightedGraphMatrix directedGraph(6, true);
    directedGraph.addEdge(0, 1);
    directedGraph.addEdge(0, 2);
    directedGraph.addEdge(1, 3);
    directedGraph.addEdge(2, 3);
    directedGraph.addEdge(2, 4);
    directedGraph.addEdge(3, 5);
    directedGraph.addEdge(4, 5);

    cout << "Directed Graph:" << endl;
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

    // Remove a vertex and an edge and display the graph again
    directedGraph.removeEdge(0, 1);

    cout << "\nDirected Graph after removing edge (0, 1):" << endl;
    directedGraph.display();

    // New undirected graph with a cycle
    UnweightedGraphMatrix undirectedGraphForCycle(4, false);
    undirectedGraphForCycle.addEdge(0, 1);
    undirectedGraphForCycle.addEdge(1, 2);
    undirectedGraphForCycle.addEdge(2, 3);
    undirectedGraphForCycle.addEdge(2, 0);

    cout << "\nUndirected Graph For Cycle Detection:" << endl;
    undirectedGraphForCycle.display();

    cout << endl;

    cout << "Does the undirected graph contain a cycle? " << (undirectedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;

    // New directed graph with a cycle
    UnweightedGraphMatrix directedGraphForCycle(4, true);
    directedGraphForCycle.addEdge(0, 1);
    directedGraphForCycle.addEdge(1, 2);
    directedGraphForCycle.addEdge(2, 3);
    directedGraphForCycle.addEdge(3, 1);

    cout << "\nDirected Graph For Cycle Detection:" << endl;
    directedGraphForCycle.display();

    cout << endl;

    cout << "Does the directed graph contain a cycle? " << (directedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;

    return 0;
}
