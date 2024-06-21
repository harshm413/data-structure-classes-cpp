#include <iostream>
#include <map>
#include <list>
#include <queue>
#include <stack>
#include <set>
#include <vector>
#include <algorithm>
#include <utility>  // For pair
#include <unordered_map>
#include <functional>

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

    // New constructor to initialize with an existing adjacency list
    WeightedGraph(const map<int, list<Edge>>& adj, bool directed) : adjList(adj), isDirected(directed) {}

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

    // Dijkstra's algorithm
    map<int, int> dijkstra(int startVertex) {
        // Map to store the shortest distance from startVertex to each vertex
        map<int, int> distances;
        for (const auto& pair : adjList) {
            distances[pair.first] = INT_MAX;
        }
        distances[startVertex] = 0;

        // Priority queue to select the vertex with the smallest distance
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        pq.push({0, startVertex});

        while (!pq.empty()) {
            int currentDistance = pq.top().first;
            int currentVertex = pq.top().second;
            pq.pop();

            // Visit each neighbor of the current vertex
            for (const auto& edge : adjList[currentVertex]) {
                int neighbor = edge.destination;
                int weight = edge.weight;
                int distance = currentDistance + weight;

                // If a shorter path to the neighbor is found
                if (distance < distances[neighbor]) {
                    distances[neighbor] = distance;
                    pq.push({distance, neighbor});
                }
            }
        }

        return distances;
    }

    // Prim's algorithm to find Minimum Spanning Tree (MST)
    map<int, list<Edge>> prim(int startVertex) {
        map<int, list<Edge>> mst;  // To store the MST
        priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> pq;
        map<int, bool> visited;  // To track vertices included in MST

        // Initialize all vertices as not in MST
        for (const auto& pair : adjList) {
            visited[pair.first] = false;
        }

        // Start with the given startVertex
        int startWeight = 0;
        pq.push({startWeight, {startVertex, startVertex}});  // {weight, {fromVertex, currentVertex}}

        while (!pq.empty()) {
            int weight = pq.top().first;
            int fromVertex = pq.top().second.first;
            int currentVertex = pq.top().second.second;
            pq.pop();

            // If currentVertex is already in MST, skip
            if (visited[currentVertex]) continue;

            // Include currentVertex in MST
            visited[currentVertex] = true;

            // Add edge to MST (skip if it's a self-loop)
            if (fromVertex != currentVertex) {
                mst[fromVertex].push_back(Edge(currentVertex, weight));
                mst[currentVertex].push_back(Edge(fromVertex, weight));
            }

            // Add all edges from currentVertex to the priority queue
            for (const auto& edge : adjList[currentVertex]) {
                int neighbor = edge.destination;
                int edgeWeight = edge.weight;
                if (!visited[neighbor]) {
                    pq.push({edgeWeight, {currentVertex, neighbor}});
                }
            }
        }

        return mst;
    }

    // Kruskal's algorithm to find Minimum Spanning Tree (MST)
    map<int, list<Edge>> kruskal() {
        vector<pair<int, Edge>> edges; // To store all edges with source vertex
        map<int, list<Edge>> mst;      // To store the MST

        // Collect all edges from the graph
        for (const auto& pair : adjList) {
            int u = pair.first;
            for (const auto& edge : pair.second) {
                edges.push_back({u, edge});
            }
        }

        // Sort edges based on weight (ascending order)
        sort(edges.begin(), edges.end(), [](const pair<int, Edge>& a, const pair<int, Edge>& b) {
            return a.second.weight < b.second.weight;
        });

        // Union-Find data structure to detect cycles
        unordered_map<int, int> parent; // To store parent of each vertex
        unordered_map<int, int> rank;   // To store rank of each vertex (for union by rank)

        // Initialize parent and rank
        for (const auto& pair : adjList) {
            int vertex = pair.first;
            parent[vertex] = vertex;
            rank[vertex] = 0;
        }

        // Function to find root of a vertex (with path compression)
        function<int(int)> findParent = [&](int vertex) {
            if (parent[vertex] != vertex) {
                parent[vertex] = findParent(parent[vertex]); // Path compression
            }
            return parent[vertex];
        };

        // Function to perform union of two sets (with union by rank)
        auto unite = [&](int u, int v) {
            int rootU = findParent(u);
            int rootV = findParent(v);

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
                return true; // Successfully united
            }
            return false; // Already in the same set
        };

        // Apply Kruskal's algorithm
        for (const auto& item : edges) {
            int u = item.first;
            int v = item.second.destination;
            int weight = item.second.weight;

            // Check if including this edge forms a cycle
            if (findParent(u) != findParent(v)) {
                // Include this edge in the MST
                mst[u].push_back(Edge(v, weight));
                if (!isDirected) {
                    mst[v].push_back(Edge(u, weight));
                }
                unite(u, v);
            }
        }

        return mst;
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

    cout << endl;

    // Create an undirected weighted graph
    WeightedGraph undirectedGraph2(false);
    undirectedGraph2.addVertex(1);
    undirectedGraph2.addVertex(2);
    undirectedGraph2.addVertex(3);
    undirectedGraph2.addVertex(4);
    undirectedGraph2.addVertex(5);
    undirectedGraph2.addVertex(6);

    undirectedGraph2.addEdge(1, 2, 7);
    undirectedGraph2.addEdge(1, 3, 8);
    undirectedGraph2.addEdge(2, 3, 3);
    undirectedGraph2.addEdge(2, 4, 6);
    undirectedGraph2.addEdge(3, 4, 4);
    undirectedGraph2.addEdge(3, 5, 3);
    undirectedGraph2.addEdge(4, 5, 2);
    undirectedGraph2.addEdge(4, 6, 5);
    undirectedGraph2.addEdge(5, 6, 2);

    cout << "Undirected Weighted Graph:" << endl;
    undirectedGraph2.display();

    // Compute Minimum Spanning Tree using Prim's algorithm
    int startVertex = 1; // Starting from vertex 1
    WeightedGraph mstPrim(undirectedGraph2.prim(startVertex), false);

    // Display MST computed by Prim's algorithm
    cout << "\nMinimum Spanning Tree (Prim's) from vertex " << startVertex << ":" << endl;
    mstPrim.display();

    // Compute Minimum Spanning Tree using Kruskal's algorithm
    WeightedGraph mstKruskal(undirectedGraph2.kruskal(), false);

    // Display MST computed by Kruskal's algorithm
    cout << "\nMinimum Spanning Tree (Kruskal's):" << endl;
    mstKruskal.display();

    cout << endl;
    
    // Create an undirected weighted graph
    WeightedGraph undirectedGraph3(false);
    undirectedGraph3.addVertex(0);
    undirectedGraph3.addVertex(1);
    undirectedGraph3.addVertex(2);
    undirectedGraph3.addVertex(3);
    undirectedGraph3.addVertex(4);
    undirectedGraph3.addVertex(5);
    undirectedGraph3.addVertex(6);
    undirectedGraph3.addVertex(7);
    undirectedGraph3.addVertex(8);

    undirectedGraph3.addEdge(0, 1, 4);
    undirectedGraph3.addEdge(0, 4, 8);
    undirectedGraph3.addEdge(1, 4, 11);
    undirectedGraph3.addEdge(1, 2, 8);
    undirectedGraph3.addEdge(4, 8, 7);
    undirectedGraph3.addEdge(4, 5, 1);
    undirectedGraph3.addEdge(2, 8, 2);
    undirectedGraph3.addEdge(8, 5, 6);
    undirectedGraph3.addEdge(2, 6, 4);
    undirectedGraph3.addEdge(2, 3, 7);
    undirectedGraph3.addEdge(5, 6, 2);
    undirectedGraph3.addEdge(3, 6, 14);
    undirectedGraph3.addEdge(3, 7, 9);
    undirectedGraph3.addEdge(6, 7, 10);

    cout << "Undirected Weighted Graph:" << endl;
    undirectedGraph3.display();

    // Compute shortest paths from vertex 1 using Dijkstra's algorithm
    startVertex = 0;
    map<int, int> shortestPaths = undirectedGraph3.dijkstra(startVertex);

    // Display shortest paths
    cout << "\nShortest paths from vertex " << startVertex << ":" << endl;
    for (const auto& pair : shortestPaths) {
        cout << "Vertex " << pair.first << " is at distance " << pair.second << endl;
    }

    return 0;
}

// OUTPUT:
// Undirected Weighted Graph:
// 1: (2,2) (3,4)
// 2: (1,2) (4,1)
// 3: (1,4) (4,3) (5,7)
// 4: (2,1) (3,3) (6,5)
// 5: (3,7) (6,2)
// 6: (4,5) (5,2)

// BFS from vertex 1: 1 2 3 4 5 6
// DFS (iterative) from vertex 1: 1 3 5 6 4 2 
// DFS (recursive) from vertex 1: 1 2 4 3 5 6

// Undirected Weighted Graph after removing edge (1, 2) and vertex 4:
// 1: (3,4)
// 2:
// 3: (1,4) (5,7)
// 5: (3,7) (6,2)
// 6: (5,2)

// Directed Weighted Graph:
// 1: (2,2) (3,4)
// 2: (4,1)
// 3: (4,3) (5,7)
// 4: (6,5)
// 5: (6,2)
// 6:

// BFS from vertex 1: 1 2 3 4 5 6
// DFS (iterative) from vertex 1: 1 3 5 6 4 2
// DFS (recursive) from vertex 1: 1 2 4 6 3 5

// Directed Weighted Graph after removing edge (1, 2) and vertex 4:
// 1: (3,4) 
// 2:
// 3: (5,7)
// 5: (6,2)
// 6:

// Undirected Weighted Graph:
// 1: (2,7) (3,8)
// 2: (1,7) (3,3) (4,6)
// 3: (1,8) (2,3) (4,4) (5,3)
// 4: (2,6) (3,4) (5,2) (6,5)
// 5: (3,3) (4,2) (6,2)
// 6: (4,5) (5,2)

// Minimum Spanning Tree (Prim's) from vertex 1:
// 1: (2,7)
// 2: (1,7) (3,3)
// 3: (2,3) (5,3)
// 4: (5,2)
// 5: (3,3) (4,2) (6,2) 
// 6: (5,2)

// Minimum Spanning Tree (Kruskal's):
// 1: (2,7)
// 2: (3,3) (1,7)
// 3: (2,3) (5,3)
// 4: (5,2)
// 5: (6,2) (4,2) (3,3)
// 6: (5,2)

// Undirected Weighted Graph:
// 0: (1,4) (4,8)
// 1: (0,4) (4,11) (2,8)
// 2: (1,8) (8,2) (6,4) (3,7)
// 3: (2,7) (6,14) (7,9)
// 4: (0,8) (1,11) (8,7) (5,1)
// 5: (4,1) (8,6) (6,2)
// 6: (2,4) (5,2) (3,14) (7,10)
// 7: (3,9) (6,10)
// 8: (4,7) (2,2) (5,6)

// Shortest paths from vertex 0:
// Vertex 0 is at distance 0
// Vertex 1 is at distance 4
// Vertex 2 is at distance 12
// Vertex 3 is at distance 19
// Vertex 4 is at distance 8
// Vertex 5 is at distance 9
// Vertex 6 is at distance 11
// Vertex 7 is at distance 21
// Vertex 8 is at distance 14
