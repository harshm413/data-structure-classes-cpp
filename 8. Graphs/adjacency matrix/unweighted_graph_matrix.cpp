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

    // Perform topological sort (only for directed graphs)
    vector<int> topologicalSort() {
        if (!isDirected) {
            cout << "Topological sort is not applicable to undirected graphs." << endl;
            return {};
        }

        if (hasCycle()) {
            cout << "Topological sort is not possible because the graph contains a cycle." << endl;
            return {};
        }

        vector<int> topoOrder;
        vector<bool> visited(numVertices, false);
        stack<int> st;

        // Call the recursive helper function for each vertex
        for (int i = 0; i < numVertices; ++i) {
            if (!visited[i]) {
                topologicalSortUtil(i, visited, st);
            }
        }

        // Collect the topological order from the stack
        while (!st.empty()) {
            topoOrder.push_back(st.top());
            st.pop();
        }

        return topoOrder;
    }

    // New method to check reachability and degree of farness
    int isReachable(int startVertex, int targetVertex) {
        if (startVertex == targetVertex) {
            return 0; // Same vertex
        }

        vector<int> distance(numVertices, -1); // Distance from startVertex to each vertex
        queue<int> q;
        
        q.push(startVertex);
        distance[startVertex] = 0;

        while (!q.empty()) {
            int vertex = q.front();
            q.pop();

            for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
                if (adjacencyMatrix[vertex][neighbor] && distance[neighbor] == -1) {
                    distance[neighbor] = distance[vertex] + 1;
                    q.push(neighbor);

                    // If we reach the targetVertex, return the distance
                    if (neighbor == targetVertex) {
                        return distance[neighbor];
                    }
                }
            }
        }

        // If targetVertex was not reached
        return -1;
    }

    // Count connected components in the graph using Union-Find
    int countConnectedComponents() {
        class UnionFind {
        private:
            vector<int> parent;
            vector<int> rank;

        public:
            UnionFind(int n) {
                parent.resize(n);
                rank.resize(n, 0);
                for (int i = 0; i < n; ++i) {
                    parent[i] = i;
                }
            }

            int find(int vertex) {
                if (parent[vertex] != vertex) {
                    parent[vertex] = find(parent[vertex]); // Path compression
                }
                return parent[vertex];
            }

            void unite(int u, int v) {
                int rootU = find(u);
                int rootV = find(v);

                if (rootU != rootV) {
                    // Union by rank
                    if (rank[rootU] > rank[rootV]) {
                        parent[rootV] = rootU;
                    } else if (rank[rootU] < rank[rootV]) {
                        parent[rootU] = rootV;
                    } else {
                        parent[rootV] = rootU;
                        rank[rootU]++;
                    }
                }
            }
        };

        UnionFind uf(numVertices);

        // Union all connected vertices according to the adjacency matrix
        for (int u = 0; u < numVertices; ++u) {
            for (int v = 0; v < numVertices; ++v) {
                if (adjacencyMatrix[u][v]) {
                    uf.unite(u, v);
                }
            }
        }

        // Count unique components
        set<int> components;
        for (int i = 0; i < numVertices; ++i) {
            components.insert(uf.find(i));
        }

        return components.size();
    }

    // Check if the graph contains a cycle using Union-Find
    bool hasCycleUF() {
        class UnionFind {
        private:
            vector<int> parent;
            vector<int> rank;

        public:
            UnionFind(int n) : parent(n), rank(n, 0) {
                for (int i = 0; i < n; ++i) {
                    parent[i] = i;
                }
            }

            int find(int vertex) {
                if (parent[vertex] != vertex) {
                    parent[vertex] = find(parent[vertex]); // Path compression
                }
                return parent[vertex];
            }

            void unite(int u, int v) {
                int rootU = find(u);
                int rootV = find(v);

                if (rootU != rootV) {
                    // Union by rank
                    if (rank[rootU] > rank[rootV]) {
                        parent[rootV] = rootU;
                    } else if (rank[rootU] < rank[rootV]) {
                        parent[rootU] = rootV;
                    } else {
                        parent[rootV] = rootU;
                        rank[rootU]++;
                    }
                }
            }
        };

        UnionFind uf(numVertices);

        // Iterate through the adjacency matrix and apply Union-Find
        for (int u = 0; u < numVertices; ++u) {
            for (int v = 0; v < numVertices; ++v) {
                if (adjacencyMatrix[u][v]) {
                    if (isDirected || u < v) {  // Only consider each edge once
                        if (uf.find(u) == uf.find(v)) {
                            return true; // Cycle detected
                        }
                        uf.unite(u, v);
                    }
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

    // Utility function for topological sort using DFS
    void topologicalSortUtil(int vertex, vector<bool>& visited, stack<int>& st) {
        visited[vertex] = true;

        // Recur for all vertices adjacent to this vertex
        for (int neighbor = 0; neighbor < numVertices; ++neighbor) {
            if (adjacencyMatrix[vertex][neighbor] && !visited[neighbor]) {
                topologicalSortUtil(neighbor, visited, st);
            }
        }

        // Push current vertex to stack which stores the result
        st.push(vertex);
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
    //undirectedGraphForCycle.addEdge(2, 0);

    cout << "\nUndirected Graph For Cycle Detection:" << endl;
    undirectedGraphForCycle.display();

    cout << endl;

    cout << "Does the undirected graph contain a cycle? " << (undirectedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;
    cout << "Does the undirected graph contain a cycle?(UnionFind Method) " << (undirectedGraphForCycle.hasCycleUF() ? "Yes" : "No") << endl;

    // New directed graph with a cycle
    UnweightedGraphMatrix directedGraphForCycle(4, true);
    directedGraphForCycle.addEdge(0, 1);
    directedGraphForCycle.addEdge(1, 2);
    directedGraphForCycle.addEdge(2, 3);
    //directedGraphForCycle.addEdge(3, 1);

    cout << "\nDirected Graph For Cycle Detection:" << endl;
    directedGraphForCycle.display();

    cout << endl;

    cout << "Does the directed graph contain a cycle? " << (directedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;
    cout << "Does the directed graph contain a cycle?(UnionFind Method) " << (directedGraphForCycle.hasCycleUF() ? "Yes" : "No") << endl;

    // Topological Sort on DAG
    UnweightedGraphMatrix dag(6, true);
    dag.addEdge(5, 0);
    dag.addEdge(5, 2);
    dag.addEdge(4, 0);
    dag.addEdge(4, 1);
    dag.addEdge(2, 3);
    dag.addEdge(3, 1);

    cout << "\nDirected Acyclic Graph (DAG):" << endl;
    dag.display();

    cout << endl;

    cout << "Topological Sort: ";
    vector<int> topoOrder = dag.topologicalSort();
    if (!topoOrder.empty()) {
        for (int vertex : topoOrder) {
            cout << vertex << " ";
        }
        cout << endl;
    }

    UnweightedGraphMatrix graph(4, false);
    graph.addEdge(0, 1);
    graph.addEdge(1, 2);
    graph.addEdge(2, 3);
    graph.addEdge(0, 3);

    cout << "Degree of farness from 0 to 3: " << graph.isReachable(0, 3) << endl;
    cout << "Degree of farness from 1 to 3: " << graph.isReachable(1, 3) << endl;
    cout << "Degree of farness from 3 to 1: " << graph.isReachable(3, 1) << endl;

    UnweightedGraphMatrix graph2(4, true);
    graph2.addEdge(0, 1);
    graph2.addEdge(1, 2);
    graph2.addEdge(2, 3);
    graph2.addEdge(0, 3);

    cout << "Degree of farness from 3 to 0: " << graph2.isReachable(3, 0) << endl;
    cout << "Degree of farness from 1 to 3: " << graph2.isReachable(1, 3) << endl;
    cout << "Degree of farness from 3 to 1: " << graph2.isReachable(3, 1) << endl;
    
    UnweightedGraphMatrix graphC1(8, false); // Undirected graph

    graphC1.addEdge(0, 1);
    graphC1.addEdge(1, 2);
    graphC1.addEdge(2, 3);
    graphC1.addEdge(0, 3);
    graphC1.addEdge(4, 5);
    graphC1.addEdge(6, 7);

    cout << "Number of connected components: " << graphC1.countConnectedComponents() << endl;

    UnweightedGraphMatrix graphC2(7, true); // Directed graph

    graphC2.addEdge(0, 1);
    graphC2.addEdge(1, 2);
    graphC2.addEdge(2, 3);
    graphC2.addEdge(0, 3);
    graphC2.addEdge(4, 5);
    graphC2.addEdge(5, 6);

    cout << "Number of connected components in directed graph: " << graphC2.countConnectedComponents() << endl;

    return 0;
}
