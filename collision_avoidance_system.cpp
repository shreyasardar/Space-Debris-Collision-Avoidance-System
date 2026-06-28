#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <array>
#include <cmath>
#include <queue> // Required for A* priority_queue
#include <limits> // Required for numeric_limits<double>::infinity()
#include <algorithm> // Required for reverse()
#include <sstream> // Required for input processing
#include <iomanip> // Required for setprecision in final output

using namespace std;

// --- Utility Functions ---

/**
 * @brief Clears the input buffer to handle potential errors from invalid input types.
 */
void clear_input() {
    if (cin.fail()) {
        cin.clear();
    }
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

// --- 1. Graph and Node Setup ---

/**
 * @brief Represents a single orbital waypoint.
 */
struct Node {
    string id;
    array<double, 3> pos; // 3D coordinates (X, Y, Z) in orbital space
    bool is_hazard = false;
    string hazard_reason;

    Node() = default;
    Node(const string& id1, double x, double y, double z)
        : id(id1), pos{x, y, z} {}
};

/**
 * @brief Represents a connection (path) between two nodes.
 */
struct Edge {
    string to; // Target node ID
    double weight; // Fuel cost (weight) required to traverse this path
    Edge(const string& t, double w) : to(t), weight(w) {}
};

/**
 * @brief Manages the entire orbital graph structure (nodes and connections).
 */
class OrbitalGraph {
public:
    /**
     * @brief Adds a new waypoint (Node) to the graph.
     */
    void add_node(const Node& node) {
        nodes[node.id] = node;
        // Initialize an empty vector for the new node's adjacency list
        adjacency[node.id] = vector<Edge>{};
    }

    /**
     * @brief Adds a bidirectional edge (path) between two existing nodes.
     * @param a ID of the first node.
     * @param b ID of the second node.
     * @param w Fuel cost (weight) of the path.
     */
    void add_edge(const string& a, const string& b, double w) {
        if (!has_node(a) || !has_node(b)) {
            cerr << "Warning: Could not create edge. One or both nodes do not exist." << endl;
            return;
        }
        // Add edge in both directions (bidirectional path)
        adjacency[a].emplace_back(b, w);
        adjacency[b].emplace_back(a, w);
    }

    /**
     * @brief Marks a node as a collision hazard, requiring avoidance.
     * @param id The ID of the node to mark.
     * @param reason The reason for the hazard (e.g., debris, failed satellite).
     */
    void mark_hazard(const string& id, const string& reason = "Predicted Debris Intersection") {
        if (nodes.count(id) == 0) {
            cerr << "Error: Cannot mark hazard. Node " << id << " does not exist." << endl;
            return;
        }
        nodes[id].is_hazard = true;
        nodes[id].hazard_reason = reason;
        cout << "\n--- COLLISION PREDICTED: Node " << id
             << " marked as HAZARD (Reason: " << reason << ") ---\n";
    }

    // --- Accessor Methods Required for A* ---

    bool has_node(const string& id) const {
        return nodes.count(id);
    }

    const Node& get_node(const string& id) const {
        // We assume caller has already checked existence (like in A*) or handles exception
        return nodes.at(id);
    }

    const map<string, Node>& get_all_nodes() const {
        return nodes;
    }

    const vector<Edge>& get_neighbors(const string& id) const {
        if (adjacency.count(id)) {
            return adjacency.at(id);
        }
        // Return an empty vector safely if the node has no neighbors (or doesn't exist)
        static const vector<Edge> empty_edges;
        return empty_edges;
    }

private:
    map<string, Node> nodes;
    map<string, vector<Edge>> adjacency;
};

// --- 2. A* Search Algorithm Components ---

/**
 * @brief Heuristic function h(n): Calculates 3D Euclidean distance (straight-line distance)
 * between the current node and the goal node. This is an admissible heuristic.
 */
double heuristic(const Node& current, const Node& goal) {
    double dx = goal.pos[0] - current.pos[0];
    double dy = goal.pos[1] - current.pos[1];
    double dz = goal.pos[2] - current.pos[2];

    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Custom structure for the priority queue: (f_score, node_id).
// The priority queue will use 'greater' to create a Min-Heap based on f_score.
using AStarEntry = pair<double, string>;

/**
 * @brief Finds the minimum fuel cost path from start to goal, avoiding hazards.
 * @return A pair containing the path (vector of node IDs) and the total fuel cost.
 */
pair<vector<string>, double> find_safe_path_astar(const OrbitalGraph& graph,
                                                     const string& start_id,
                                                     const string& goal_id) {
    
    // Initial validation check
    if (!graph.has_node(start_id) || !graph.has_node(goal_id)) {
        return {{"Error: Start or Goal node not found."}, 0.0};
    }

    const Node& goal_node = graph.get_node(goal_id);

    // g_score: actual minimum cost found so far from start to a node
    map<string, double> g_score;
    // f_score: estimated total cost (g_score + heuristic)
    map<string, double> f_score;
    // came_from: Tracks the optimal previous node for path reconstruction
    map<string, string> came_from;

    // Initialize all scores to an impossible value (infinity)
    for (const auto& pair : graph.get_all_nodes()) {
        g_score[pair.first] = numeric_limits<double>::infinity();
        f_score[pair.first] = numeric_limits<double>::infinity();
    }

    // Initialize the start node
    g_score[start_id] = 0;
    f_score[start_id] = heuristic(graph.get_node(start_id), goal_node);

    // Min-Heap priority queue to efficiently retrieve the node with the lowest f_score
    priority_queue<AStarEntry, vector<AStarEntry>, greater<AStarEntry>> open_set;
    open_set.push({f_score[start_id], start_id});

    while (!open_set.empty()) {
        // Get the node with the lowest f_score from the open set
        string current_id = open_set.top().second;
        open_set.pop();

        // 1. Goal Check: If we reached the destination, reconstruct the path
        if (current_id == goal_id) {
            vector<string> path;
            double total_cost = g_score[current_id];
            string current = current_id;

            // Backtrack from goal to start using the came_from map
            while (current != start_id) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start_id);
            // The path is built in reverse, so we need to flip it
            reverse(path.begin(), path.end());
            return {path, total_cost};
        }

        // 2. Explore Neighbors
        const vector<Edge>& neighbors = graph.get_neighbors(current_id);
        for (const auto& edge : neighbors) {
            string neighbor_id = edge.to;
            double edge_cost = edge.weight;

            const Node& neighbor_node = graph.get_node(neighbor_id);

            // CRITICAL COLLISION AVOIDANCE LOGIC: Immediately skip and ignore hazards
            if (neighbor_node.is_hazard) {
                continue;
            }

            // Calculate the cost to reach the neighbor through the current node
            double tentative_g_score = g_score[current_id] + edge_cost;

            // If this new path is better (lower g_score) than any previously found path
            if (tentative_g_score < g_score[neighbor_id]) {
                // Update the path information
                came_from[neighbor_id] = current_id;
                g_score[neighbor_id] = tentative_g_score;
                // Update the estimated total cost
                f_score[neighbor_id] = tentative_g_score + heuristic(neighbor_node, goal_node);

                // Add the neighbor to the priority queue to be explored
                // (Note: This may create a duplicate entry if the node was already in the queue,
                // but the old, worse entry will be ignored later when popped).
                open_set.push({f_score[neighbor_id], neighbor_id});
            }
        }
    }

    // If the loop finishes without finding the goal
    return {{"No safe path found."}, 0.0};
}


// --- 3. User Input and Demo Functions ---

/**
 * @brief Prints all currently defined waypoints for user reference.
 */
void display_current_nodes(const OrbitalGraph& graph) {
    cout << "\nExisting Nodes (Waypoints): ";
    bool first = true;
    for (const auto& pair : graph.get_all_nodes()) {
        if (!first) cout << ", ";
        cout << pair.first << (pair.second.is_hazard ? " (HAZARD)" : "");
        first = false;
    }
    cout << endl;
}

/**
 * @brief Handles interactive input for defining orbital waypoints (nodes).
 */
void read_nodes(OrbitalGraph& graph) {
    int num_nodes = 0;
    cout << "\n--- Graph Construction: Waypoints (Nodes) ---" << endl;
    cout << "Enter the total number of waypoints (nodes) in the orbital grid: ";
    if (!(cin >> num_nodes) || num_nodes < 1) {
        cout << "Invalid number or zero nodes entered. Exiting node entry." << endl;
        clear_input();
        return;
    }

    for (int i = 0; i < num_nodes; ++i) {
        string id;
        double x, y, z;
        cout << "Node " << i + 1 << " of " << num_nodes << ":" << endl;
        cout << "  Enter Node ID (e.g., A, WP1): ";
        cin >> id;
        cout << "  Enter X, Y, Z coordinates (space separated, e.g., 100.5 20.0 5.1): ";
        if (!(cin >> x >> y >> z)) {
            cerr << "Invalid coordinate input. Node " << id << " skipped." << endl;
            clear_input(); // Use the cleaner utility function
            continue;
        }
        graph.add_node(Node(id, x, y, z));
    }
    display_current_nodes(graph);
}

/**
 * @brief Handles interactive input for defining paths (edges) and their fuel costs.
 */
void read_edges(OrbitalGraph& graph) {
    int num_edges = 0;
    cout << "\n--- Graph Construction: Connections (Edges/Paths) ---" << endl;
    cout << "Edges represent potential paths between waypoints. Cost is fuel consumption." << endl;
    cout << "Enter the total number of connections (edges) to define: ";
    if (!(cin >> num_edges) || num_edges < 0) {
        cout << "Invalid number or zero edges entered. Exiting edge entry." << endl;
        clear_input(); // Use the cleaner utility function
        return;
    }

    for (int i = 0; i < num_edges; ++i) {
        string id1, id2;
        double cost;
        cout << "Edge " << i + 1 << " of " << num_edges << ":" << endl;
        cout << "  Enter ID of Node 1, ID of Node 2, and Fuel Cost (e.g., A B 1.5): ";
        if (!(cin >> id1 >> id2 >> cost)) {
            cerr << "Invalid edge input. Edge " << i + 1 << " skipped." << endl;
            clear_input(); // Use the cleaner utility function
            continue;
        }
        graph.add_edge(id1, id2, cost);
    }
}

/**
 * @brief Gathers the required simulation parameters: Start, Goal, and Hazard nodes.
 */
void read_scenario_parameters(OrbitalGraph& graph, string& start_id, string& goal_id) {
    string hazard_id;
    bool valid = false;

    cout << "\n--- Scenario Setup: Satellite Route ---" << endl;

    // Get Start Node with validation
    while (!valid) {
        cout << "Enter Satellite's START Node ID: ";
        cin >> start_id;
        if (graph.has_node(start_id)) {
            valid = true;
        } else {
            cout << "Invalid node ID. Please enter one of the existing nodes." << endl;
            display_current_nodes(graph);
        }
    }

    // Get Goal Node with validation
    valid = false;
    while (!valid) {
        cout << "Enter Satellite's GOAL (Destination) Node ID: ";
        cin >> goal_id;
        if (graph.has_node(goal_id)) {
            valid = true;
        } else {
            cout << "Invalid node ID. Please enter one of the existing nodes." << endl;
        }
    }

    // Get Hazard Node
    cout << "\n--- Collision Prediction ---" << endl;
    cout << "Enter the Node ID where space debris is predicted (HAZARD NODE ID): ";
    cin >> hazard_id;

    if (graph.has_node(hazard_id)) {
        graph.mark_hazard(hazard_id);
    } else {
        cout << "Warning: Hazard node " << hazard_id << " not found in graph. Proceeding without marking hazard." << endl;
    }
}

/**
 * @brief Runs the complete interactive demonstration.
 */
void run_interactive_demo() {
    cout << "--- Graph-Based Space Debris Collision Avoidance System (A* Rerouting) ---" << endl;
    cout << "Let's build your orbital grid and simulate collision avoidance with A* Search." << endl;

    OrbitalGraph orbit_graph;
    string start_id, goal_id;

    // 1. Read Nodes
    read_nodes(orbit_graph);

    // 2. Read Edges
    read_edges(orbit_graph);

    // 3. Read Scenario Parameters (Start, Goal, Hazard)
    read_scenario_parameters(orbit_graph, start_id, goal_id);

    cout << "\n" << string(60, '=') << endl;
    cout << "SIMULATION RUNNING: Finding optimal path from " << start_id << " to " << goal_id << endl;
    cout << string(60, '=') << endl;

    // 4. Collision Avoidance (Rerouting)
    pair<vector<string>, double> result = find_safe_path_astar(orbit_graph, start_id, goal_id);
    vector<string> safe_path = result.first;
    double safe_cost = result.second;

    // --- Output Results (Final Report) ---
    cout << "\n" << string(50, '=') << endl;
    cout << "A* REROUTING RESULTS (MINIMUM FUEL COST)" << endl;
    cout << string(50, '=') << endl;

    cout << "Start Point: " << start_id << endl;
    cout << "Destination: " << goal_id << endl;
    cout << "Status: " << (safe_path.size() > 1 ? "Path Found" : "No Path Found") << endl;

    cout << "==================================================" << endl;

    if (safe_path.size() > 1) {
        cout << "Optimal Path: ";
        for (size_t i = 0; i < safe_path.size(); ++i) {
            cout << safe_path[i] << (i == safe_path.size() - 1 ? "" : " -> ");
        }
        cout << endl;
        // Use setprecision to display the cost nicely
        cout << "Total Fuel Cost (g(n)): " << fixed << setprecision(3) << safe_cost << " units" << endl;
        cout << "\n**Conclusion: The A* algorithm successfully calculated the optimal, collision-free route with minimal fuel consumption.**" << endl;
    } else {
        // Output error message if only one element (the error string) is returned
        cout << "Error: " << safe_path[0] << endl;
        cout << "\n**Conclusion: Rerouting failed. All viable routes may be blocked by debris or the graph is disconnected.**" << endl;
    }
    cout << string(50, '=') << endl;
}

int main() {
    // Note: The main function simply calls the interactive demo.
    run_interactive_demo();
    return 0;
}
