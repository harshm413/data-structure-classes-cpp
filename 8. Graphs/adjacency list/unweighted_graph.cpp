#include <iostream>
#include <map>
#include <list>
#include <queue>
#include <stack>
#include <set>
#include <vector>

using namespace std;

class UnweightedGraph {
private:
    map<int, list<int>> adjList;
    bool isDirected;

public:
    // Constructor
    UnweightedGraph(bool directed) : isDirected(directed) {}

    // Add vertex to the graph
    void addVertex(int vertex) {
        if (adjList.find(vertex) == adjList.end()) {
            adjList[vertex] = list<int>();
        }
    }

    // Remove vertex from the graph
    void removeVertex(int vertex) {
        adjList.erase(vertex);
        for (auto& pair : adjList) {
            pair.second.remove(vertex);
        }
    }

    // Add edge to the graph
    void addEdge(int u, int v) {
        addVertex(u);
        addVertex(v);
        adjList[u].push_back(v);
        if (!isDirected && u != v) {
            adjList[v].push_back(u);  // Avoid self-loop for undirected graphs
        }
    }

    // Remove edge from the graph
    void removeEdge(int u, int v) {
        if (adjList.find(u) != adjList.end()) {
            adjList[u].remove(v);
        }
        if (!isDirected && adjList.find(v) != adjList.end()) {
            adjList[v].remove(u);
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

            for (int neighbor : adjList[vertex]) {
                if (visited.find(neighbor) == visited.end()) {
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

            for (int neighbor : adjList[vertex]) {
                if (visited.find(neighbor) == visited.end()) {
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
        for (auto& pair : adjList) {
            int vertex = pair.first;
            list<int> neighbors = pair.second;
            cout << vertex << ": ";
            for (int neighbor : neighbors) {
                cout << neighbor << " ";
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
        set<int> visited;
        set<int> recStack;

        for (const auto& pair : adjList) {
            int vertex = pair.first;
            if (visited.find(vertex) == visited.end()) {
                if (dfsCycleDetection(vertex, visited, recStack, -1)) {
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
        stack<int> st;
        set<int> visited;

        // Call the recursive helper function for each vertex
        for (const auto& pair : adjList) {
            int vertex = pair.first;
            if (visited.find(vertex) == visited.end()) {
                topologicalSortUtil(vertex, visited, st);
            }
        }

        // Collect the topological order from the stack
        while (!st.empty()) {
            topoOrder.push_back(st.top());
            st.pop();
        }

        return topoOrder;
    }

private:
    // Utility function for recursive DFS
    void DFSRecHelper(int vertex, set<int>& visited, vector<int>& traversal) {
        visited.insert(vertex);
        traversal.push_back(vertex);

        for (int neighbor : adjList[vertex]) {
            if (visited.find(neighbor) == visited.end()) {
                DFSRecHelper(neighbor, visited, traversal);
            }
        }
    }

    // Utility function for detecting cycles using DFS
    bool dfsCycleDetection(int vertex, set<int>& visited, set<int>& recStack, int parent) {
        visited.insert(vertex);
        recStack.insert(vertex);

        for (int neighbor : adjList[vertex]) {
            if (visited.find(neighbor) == visited.end()) {
                if (dfsCycleDetection(neighbor, visited, recStack, vertex)) {
                    return true;
                }
            } else if (recStack.find(neighbor) != recStack.end() && (isDirected || neighbor != parent)) {
                return true;
            }
            
        }

        recStack.erase(vertex);
        return false;
    }

    // Utility function for topological sort using DFS
    void topologicalSortUtil(int vertex, set<int>& visited, stack<int>& st) {
        visited.insert(vertex);

        // Recur for all vertices adjacent to this vertex
        for (int neighbor : adjList[vertex]) {
            if (visited.find(neighbor) == visited.end()) {
                topologicalSortUtil(neighbor, visited, st);
            }
        }

        // Push current vertex to stack which stores the result
        st.push(vertex);
    }
};

int main() {
    // Create an undirected graph
    UnweightedGraph undirectedGraph(false);
    undirectedGraph.addVertex(1);
    undirectedGraph.addVertex(2);
    undirectedGraph.addVertex(3);
    undirectedGraph.addVertex(4);
    undirectedGraph.addVertex(5);
    undirectedGraph.addVertex(6);

    undirectedGraph.addEdge(1, 2);
    undirectedGraph.addEdge(1, 3);
    undirectedGraph.addEdge(2, 4);
    undirectedGraph.addEdge(3, 4);
    undirectedGraph.addEdge(3, 5);
    undirectedGraph.addEdge(4, 6);
    undirectedGraph.addEdge(5, 6);

    cout << "Undirected Graph:" << endl;
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

    cout << "\nUndirected Graph after removing edge (1, 2) and vertex 4:" << endl;
    undirectedGraph.display();

    cout << endl;

    // Create a directed graph
    UnweightedGraph directedGraph(true);
    directedGraph.addVertex(1);
    directedGraph.addVertex(2);
    directedGraph.addVertex(3);
    directedGraph.addVertex(4);
    directedGraph.addVertex(5);
    directedGraph.addVertex(6);

    directedGraph.addEdge(1, 2);
    directedGraph.addEdge(1, 3);
    directedGraph.addEdge(2, 4);
    directedGraph.addEdge(3, 4);
    directedGraph.addEdge(3, 5);
    directedGraph.addEdge(4, 6);
    directedGraph.addEdge(5, 6);

    cout << "Directed Graph:" << endl;
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

    cout << "\nDirected Graph after removing edge (1, 2) and vertex 4:" << endl;
    directedGraph.display();

    // New undirected graph with a cycle
    UnweightedGraph undirectedGraphForCycle(false);
    undirectedGraphForCycle.addEdge(0, 1);
    undirectedGraphForCycle.addEdge(1, 2);
    undirectedGraphForCycle.addEdge(2, 3);
    //undirectedGraphForCycle.addEdge(2, 0);

    cout << "\nUndirected Graph For Cycle Detection:" << endl;
    undirectedGraphForCycle.display();

    cout << endl;

    cout << "Does the undirected graph contain a cycle? " << (undirectedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;

    // New directed graph with a cycle
    UnweightedGraph directedGraphForCycle(true);
    directedGraphForCycle.addEdge(0, 1);
    directedGraphForCycle.addEdge(1, 2);
    directedGraphForCycle.addEdge(2, 3);
    //directedGraphForCycle.addEdge(3, 1);

    cout << "\nDirected Graph For Cycle Detection:" << endl;
    directedGraphForCycle.display();

    cout << endl;

    cout << "Does the directed graph contain a cycle? " << (directedGraphForCycle.hasCycle() ? "Yes" : "No") << endl;

    // Create a directed acyclic graph (DAG) for topological sorting
    UnweightedGraph dag(true);
    dag.addEdge(5, 0);
    dag.addEdge(5, 2);
    dag.addEdge(4, 0);
    dag.addEdge(4, 1);
    dag.addEdge(2, 3);
    dag.addEdge(3, 1);

    cout << "Directed Acyclic Graph (DAG):" << endl;
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

    return 0;
}
