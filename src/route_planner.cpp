#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Call FindNeighbors() on the current_node:
    current_node->FindNeighbors();

    // Iterate over the neighbors and set the parent, the g_value and h_value:
    for(auto neighbor : current_node->neighbors ){
        // set the parent of the neighbor to the current:
        neighbor->parent = current_node;

        // set the g_value to be the current g_value + distance() to current_node:
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);

        // set the h_value using CalculateHValue():
        neighbor->h_value = CalculateHValue(neighbor);

        // add neighbor to the open_list:
        open_list.push_back(neighbor);

        // mark neighbor as visited:
        neighbor->visited = true;
    }

    // for(std::size_t i = 0; i < current_node->neighbors.size(); i++) {
        
    //     // std::cout << "current node[" << i << "].x: " << current_node[i].x << std::endl;

    //     // Set the parent, the h_value, the g_value:
    //     current_node->neighbors[i]->parent = current_node;
    //     current_node->neighbors[i]->g_value = current_node->g_value + current_node->distance(*current_node->neighbors[i]);

    //     // Use CalculateHValue below to implement the h-Value calculation:
    //     current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);

    //     // Add the neighbor to open_list and set the node's visited attribute to true:
    //     open_list.push_back(current_node->neighbors[i]);
    //     current_node->neighbors[i]->visited = true;
    // }
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(), CompareValues);

    // note: en.cppreference.co/algorithm/sort for lambda expression
    // useful to force an error in the auto type, so the compiler can throw the error and explicitly show the type of auto
    // I used this example to explain an answear on the Knowlegde forum.
    // std::sort(open_list.begin(), open_list.end(), [](const auto& a, const auto& b){
    //     int c = a;
    //     return true;
    // });

    // create the pointer with the lowest sum node and remove the node from the open_list:
    RouteModel::Node* lowestSumNode = open_list.back();
    open_list.pop_back();
    // RouteModel::Node* lowestSumNode = open_list.front();
    // open_list.erase(open_list.begin());

    // std::cout << "lowestSumNode: " << lowestSumNode->x << ", " << lowestSumNode->y << std::endl;

    return lowestSumNode;
}

// Helper function to sort the open_list according to the sum of the h value and g value:
bool RoutePlanner::CompareValues(const RouteModel::Node* v1, const RouteModel::Node* v2) {
    return( v1->g_value + v1->h_value > v2->g_value + v2->h_value ); 
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // The parent of the start_node is a nullptr (that's our stopping condition)
    while(current_node->parent != nullptr){
        
        // Add the current_node to the path_found
        path_found.push_back(*current_node);
        
        // Add the distance from the node to its parent to the distance variable:
        if(current_node->parent) {
            distance += current_node->distance(*current_node->parent);
        }
        
        // // Save the current note
        // // And update nodes so the parent node could be the current:
        // // path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    // don't forget to add the first node (last from the while loop)
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());                 // needed to reverse the vector (TDD was awesome for the insight!)

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    // Initialize the open_list:
    start_node->visited = true;
    open_list.push_back(start_node);
    // std::cout << open_list.size() << std::endl;
    
    while(open_list.size() > 0){
        current_node = NextNode();
    
        if(current_node->distance(*end_node) == 0){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else{
            AddNeighbors(current_node);
        }
    }
}