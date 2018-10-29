/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <iomanip>
#include <cstdio>

#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "LatLon.h"
#include "Feature.h"
#include "OSMID.h"
#include <vector>
#include <string>
#include <set>
#include <utility>
#include <queue>

const unsigned ID_NULL = 99999999;
double heuristic_factor = 0.9;
double maximum_speed = 25.0; // = 90km/s

struct visited_node{
    bool visited;
    unsigned parent;
};

// New m1 data structure functions
unsigned find_street_segment_from_intersection_IDs(unsigned intersection_id1, unsigned intersection_id2);
// New m2 function for finding intersection for a given POI name 
std::vector<unsigned> intersection_to_go_for_POI(std::string POI_name);


//compare two elements as arguments and return a boolean value, used in priority queue
class Comparator {
public:
    bool operator ()(const std::pair<unsigned, double> &compare_point_1 ,const std::pair<unsigned, double> &compare_point_2) const {
        return compare_point_1.second > compare_point_2.second;
    }
};


// Returns the time required to travel along the path specified, in seconds. 
// The path is given as a vector of street segment ids, and this function 
// can assume the vector either forms a legal path or has size == 0.
// The travel time is the sum of the length/speed-limit of each street 
// segment, plus the given turn_penalty (in seconds) per turn implied by the path. 
// A turn occurs when two consecutive street segments have different street IDs.
double compute_path_travel_time(const std::vector<unsigned>& path, 
                                const double turn_penalty){
    double travel_time = 0.0;
    //traverse the current segment to get the street segment travel time 
    for (unsigned current_seg = 0; current_seg < path.size(); current_seg++){
        travel_time = travel_time + find_street_segment_travel_time(path[current_seg]);
        if ( current_seg != 0){
            unsigned current_street = getStreetSegmentInfo(path[current_seg]).streetID;
            unsigned previous_street = getStreetSegmentInfo(path[current_seg-1]).streetID;
            //consider turn_panelty in this case
            if (current_street != previous_street){
                travel_time = travel_time + turn_penalty;
            } 
        }  
    }
    return travel_time;     
    
}


// Returns a path (route) between the start intersection and the end 
// intersection, if one exists. This routine should return the shortest path
// between the given intersections when the time penalty to turn (change
// street IDs) is given by turn_penalty (in seconds).
// If no path exists, this routine returns an empty (size == 0) vector. 
// If more than one path exists, the path with the shortest travel time is 
// returned. The path is returned as a vector of street segment ids; traversing 
// these street segments, in the returned order, would take one from the start 
// to the end intersection.

//this function is using the algorithm that is the same as dijkstra's theorem that is called uniform cost search
//Dijkstra's Algorithm finds the shortest path from root node to every other node. uniform cost searches for 
//shortest paths in terms of cost from root node to a goal node
//we divide all the nodes into two groups one is frontier one is explored

//this function also uses A* algorithm for the performance purpose
//aka make the finding process more effective
std::vector<unsigned> find_path_between_intersections(const unsigned intersect_id_start, 
                                                      const unsigned intersect_id_end,
                                                      const double turn_penalty){   
   heuristic_factor = 0.9;
   maximum_speed = 25.0; // = 90km/s

    std::vector<unsigned> path;
    std::vector<unsigned> result;
    std::vector<unsigned> intersection_path;
     // if the start intersection id is the same as the end intersection id then then we already find the path
    if (intersect_id_start == intersect_id_end){
        return path;
    }
    //using priority queue to store all the frontier
    //priority queue is ordered by the cost of node
    std::priority_queue<std::pair<unsigned, double>, std::vector <std::pair <unsigned, double>>, Comparator> frontier;
    std::vector<visited_node> explored;
    explored.resize(getNumberOfIntersections()+1);
    
    //nodes in frontier should store 1.if exist 2. ID 3. cost 4. heuristic cost 5. parent
    //a vector of booleans to store exist in frontier
    std::vector<bool> frontier_exist;
    frontier_exist.resize(getNumberOfIntersections()+1);
    //frontier IDs
    std::vector<unsigned> frontier_ID;
    frontier_ID.resize(getNumberOfIntersections()+1);
    //frontier costs
    std::vector <double> frontier_cost;
    frontier_cost.resize(getNumberOfIntersections()+1);
    
    std::vector <double> frontier_Hcost;
    frontier_Hcost.resize(getNumberOfIntersections()+1);
    
    std::vector<unsigned> frontier_parent;
    frontier_parent.resize(getNumberOfIntersections()+1);
    
    //initialize the values in frontier 
    frontier_exist [intersect_id_start] = true;
    frontier_cost[intersect_id_start] = 0.0;
    //calculate the heuristic cost of the node 
    frontier_Hcost[intersect_id_start] = 0.0 + (1 / maximum_speed)*(heuristic_factor)*find_distance_between_two_points(
            getIntersectionPosition(intersect_id_start),getIntersectionPosition(intersect_id_end));
    //set the parent value into an infinite number which means is hasnt been visited
    frontier_parent[intersect_id_start] = ID_NULL;
    //push the node in 
    frontier.push( std::make_pair(intersect_id_start,frontier_Hcost[intersect_id_start]) );
    
    
    do{
        //if frontier is empty then return failure
        if (frontier.empty() == true){
            break;          
        }
        //when the first element of frontier is the same as the end intersection that we are looking for  
        if (frontier.top().first == intersect_id_end){
            //set the parent id to the frontier parent id
            explored[frontier.top().first].parent = frontier_parent[frontier.top().first];
            explored[frontier.top().first].visited = true;
            
            unsigned current_node = intersect_id_end;
            //back track
            while ( explored[current_node].parent != ID_NULL){
                intersection_path.push_back(current_node);
                current_node = explored[current_node].parent;  
            }
            intersection_path.push_back(intersect_id_start); 
                 
            std::reverse(intersection_path.begin(), intersection_path.end());
            //this is for debug purpose
           //return intersection_path;
           for (unsigned current_intersection = 0; current_intersection < intersection_path.size()-1; 
                    current_intersection++){
                unsigned street_seg_id = find_street_segment_from_intersection_IDs(intersection_path[current_intersection], 
                        intersection_path[current_intersection+1]);
                path.push_back(street_seg_id);
            }
              
            return path;
        }
        //this is the node that we find should be expanded
        unsigned node_to_expand_id;
              
        node_to_expand_id =  frontier.top().first;
        frontier.pop();
       
        if (explored[node_to_expand_id].visited == true){ 
            continue;
        }
                      
        std::vector<unsigned> child = find_adjacent_intersections(node_to_expand_id);
        explored[node_to_expand_id].parent = frontier_parent[node_to_expand_id];
        explored[node_to_expand_id].visited = true;
        //for each child we need 
        for (unsigned current_child = 0; current_child < child.size(); current_child ++ ){
            //if the child we are looking for is in the explored
            if ( explored[child[current_child]].visited == true  ){
                
            }
            else if ( frontier_exist[child[current_child]] == true ){
                
                double current_cost = frontier_cost[node_to_expand_id];
                unsigned current_segment = find_street_segment_from_intersection_IDs(node_to_expand_id, child[current_child]);
                    current_cost = current_cost + find_street_segment_travel_time(current_segment);
                    unsigned current_street = getStreetSegmentInfo(current_segment).streetID;

                    if (frontier_parent[node_to_expand_id] != ID_NULL){
                        unsigned previous_segment = find_street_segment_from_intersection_IDs(frontier_parent[node_to_expand_id],node_to_expand_id); 
                        unsigned previous_street = getStreetSegmentInfo(previous_segment).streetID;
                        if (current_street!=previous_street){
                            current_cost = current_cost + turn_penalty;
                        }
                    }
                 double H_cost = current_cost + (1/maximum_speed)*(heuristic_factor)*find_distance_between_two_points(
                                getIntersectionPosition(child[current_child]),getIntersectionPosition(intersect_id_end));
                 //check the cost of the heuristic cost and the frontier cost 
                 //if the cost is smaller than replace
                if (H_cost < frontier_Hcost[ child[current_child] ] ){
                                        
                    frontier_cost[ child[current_child] ] = current_cost;
                    frontier_parent[ child[current_child] ] = node_to_expand_id;  
                    frontier_Hcost[ child[current_child] ] = H_cost;
                                       
                    frontier.push(std::make_pair(child[current_child], H_cost));                  
                }                                     
            }
            //if the child is not in the explored or frontier
            else{
                double current_cost = frontier_cost[node_to_expand_id];
                unsigned current_segment = find_street_segment_from_intersection_IDs(node_to_expand_id, 
                                child[current_child]);
                        current_cost = current_cost + find_street_segment_travel_time(current_segment);
                unsigned current_street = getStreetSegmentInfo(current_segment).streetID;
                
                        if (frontier_parent[node_to_expand_id] != ID_NULL){
                            unsigned previous_segment = find_street_segment_from_intersection_IDs(frontier_parent[node_to_expand_id],node_to_expand_id); 
                            unsigned previous_street = getStreetSegmentInfo(previous_segment).streetID;
                            if (current_street!=previous_street){
                                current_cost = current_cost + turn_penalty;
                            }
                        }
                
                 double H_cost = current_cost + (1/maximum_speed)*(heuristic_factor)*find_distance_between_two_points(
                                getIntersectionPosition(child[current_child]),getIntersectionPosition(intersect_id_end));
                
                frontier.push(std::make_pair(child[current_child], H_cost));                                          
                frontier_exist [ child[current_child] ] = true;       
                frontier_cost[ child[current_child] ] = current_cost;
                frontier_parent[ child[current_child] ] = node_to_expand_id;
                frontier_Hcost[ child[current_child] ] = H_cost;                            
            }                             
        }   
    }while(true);
    
    return path;
}


// Returns the shortest travel time path (vector of street segments) from 
// the start intersection to a point of interest with the specified name.
// The path will begin at the specified intersection, and end on the 
// intersection that is closest (in Euclidean distance) to the point of 
// interest.
// If no such path exists, returns an empty (size == 0) vector.

//this function is using the algorithm that is the same as dijkstra's theorem that is called uniform cost search
//Dijkstra's Algorithm finds the shortest path from root node to every other node. uniform cost searches for 
//shortest paths in terms of cost from root node to a goal node
//we divide all the nodes into two groups one is frontier one is explored
std::vector<unsigned> find_path_to_point_of_interest(const unsigned intersect_id_start, 
                                               const std::string point_of_interest_name,
                                               const double turn_penalty){
    //
    std::vector<unsigned> valid_intersection;        
    valid_intersection = intersection_to_go_for_POI( point_of_interest_name);
        
    std::vector<bool> intersection_bool_POI;
    intersection_bool_POI.resize(getNumberOfIntersections()+1);
      
    for (unsigned current_intersection = 0; current_intersection < valid_intersection.size(); 
            current_intersection++){        
        intersection_bool_POI[valid_intersection[current_intersection]] = true; 
    }

    //**************************************************************************************************************************************
    //using the exact function above (find_path_to_point_of_interest) since we already convert the point of interest to intersection 
    //**************************************************************************************************************************************
    std::vector<unsigned> path;
     std::vector<unsigned> result;
    std::vector<unsigned> intersection_path;
          
    if (intersection_bool_POI[intersect_id_start] == true){
        return path;
    }
               
    std::priority_queue<std::pair<unsigned, double>, std::vector <std::pair <unsigned, double>>, Comparator> frontier;
    std::vector<visited_node> explored;
    explored.resize(getNumberOfIntersections()+1);
    
    std::vector<bool> frontier_exist;
    frontier_exist.resize(getNumberOfIntersections()+1);
    
    std::vector<unsigned> frontier_ID;
    frontier_ID.resize(getNumberOfIntersections()+1);
    
    std::vector <double> frontier_cost;
    frontier_cost.resize(getNumberOfIntersections()+1);
    
    std::vector<unsigned> frontier_parent;
    frontier_parent.resize(getNumberOfIntersections()+1);
    
    frontier.push( std::make_pair(intersect_id_start,0.0) );
    
    frontier_exist [intersect_id_start] = true;
    frontier_cost[intersect_id_start] = 0.0;
    frontier_parent[intersect_id_start] = ID_NULL;
    
 
    do{
        if (frontier.empty() == true){
            break;          
        }
        
        if ( intersection_bool_POI[frontier.top().first] == true ){
            
            explored[frontier.top().first].parent = frontier_parent[frontier.top().first];
            explored[frontier.top().first].visited = true;
            
            unsigned current_node = frontier.top().first;
            
            while ( explored[current_node].parent != ID_NULL){
                intersection_path.push_back(current_node);
                current_node = explored[current_node].parent;  
            }
            intersection_path.push_back(intersect_id_start); 
                 
            std::reverse(intersection_path.begin(), intersection_path.end());
            
           // return intersection_path;
            
           for (unsigned current_intersection = 0; current_intersection < intersection_path.size()-1; 
                    current_intersection++){
                unsigned street_seg_id = find_street_segment_from_intersection_IDs(intersection_path[current_intersection], 
                        intersection_path[current_intersection+1]);
                path.push_back(street_seg_id);
            }
              
            return path;
        }
        
        unsigned node_to_expand_id =  frontier.top().first;
        frontier.pop();
        
        if (explored[node_to_expand_id].visited == true){ 
            continue;
        }
        
        std::vector<unsigned> child = find_adjacent_intersections(node_to_expand_id);
        explored[node_to_expand_id].parent = frontier_parent[node_to_expand_id];
        explored[node_to_expand_id].visited = true;
        
        for (unsigned current_child = 0; current_child < child.size(); current_child ++ ){
            
            if ( explored[child[current_child]].visited == true  ){
                
            }
            else if ( frontier_exist[child[current_child]] == true ){
                
                // check the cost
                double current_cost = frontier_cost[node_to_expand_id];
                unsigned current_segment = find_street_segment_from_intersection_IDs(node_to_expand_id, 
                                child[current_child]);
                        current_cost = current_cost + find_street_segment_travel_time(current_segment);
                        unsigned current_street = getStreetSegmentInfo(current_segment).streetID;
                
                        if (frontier_parent[node_to_expand_id] != ID_NULL){
                            unsigned previous_segment = find_street_segment_from_intersection_IDs(frontier_parent[node_to_expand_id],node_to_expand_id); 
                            unsigned previous_street = getStreetSegmentInfo(previous_segment).streetID;
                            if (current_street!=previous_street){
                                current_cost = current_cost + turn_penalty;
                            }
                        }
                
                if (current_cost < frontier_cost[ child[current_child] ] ){
                    frontier.push(std::make_pair(child[current_child], current_cost));
                    
                    frontier_cost[ child[current_child] ] = current_cost;
                    frontier_parent[ child[current_child] ] = node_to_expand_id;                  
                }                                     
            }
            
            else{
                double current_cost = frontier_cost[node_to_expand_id];
                unsigned current_segment = find_street_segment_from_intersection_IDs(node_to_expand_id, 
                                child[current_child]);
                        current_cost = current_cost + find_street_segment_travel_time(current_segment);
                unsigned current_street = getStreetSegmentInfo(current_segment).streetID;
                
                        if (frontier_parent[node_to_expand_id] != ID_NULL){
                            unsigned previous_segment = find_street_segment_from_intersection_IDs(frontier_parent[node_to_expand_id],node_to_expand_id); 
                            unsigned previous_street = getStreetSegmentInfo(previous_segment).streetID;
                            if (current_street!=previous_street){
                                current_cost = current_cost + turn_penalty;
                            }
                        }
                
                frontier.push(std::make_pair(child[current_child], current_cost));                                          
                frontier_exist [ child[current_child] ] = true;       
                frontier_cost[ child[current_child] ] = current_cost;
                frontier_parent[ child[current_child] ] = node_to_expand_id;
                                            
            }                             
        }   
    }while(true);
    
    return path;     
}



double find_time_between_intersections_fast(const unsigned intersect_id_start, 
                                                      const unsigned intersect_id_end,
                                                      const double turn_penalty){   
   heuristic_factor = 2;
   maximum_speed = 25.0; // = 90km/s

    std::vector<unsigned> path;
    std::vector<unsigned> result;
    std::vector<unsigned> intersection_path;
     // if the start intersection id is the same as the end intersection id then then we already find the path
    if (intersect_id_start == intersect_id_end){
        return 0.0;
    }
    
    //using priority queue to store all the frontier
    //priority queue is ordered by the cost of node
    std::priority_queue<std::pair<unsigned, double>, std::vector <std::pair <unsigned, double>>, Comparator> frontier;
    std::vector<visited_node> explored;
    explored.resize(getNumberOfIntersections()+1);
    
    //nodes in frontier should store 1.if exist 2. ID 3. cost 4. heuristic cost 5. parent
    //a vector of booleans to store exist in frontier
    std::vector<bool> frontier_exist;
    frontier_exist.resize(getNumberOfIntersections()+1);
    //frontier IDs
    std::vector<unsigned> frontier_ID;
    frontier_ID.resize(getNumberOfIntersections()+1);
    //frontier costs
    std::vector <double> frontier_cost;
    frontier_cost.resize(getNumberOfIntersections()+1);
    
    std::vector <double> frontier_Hcost;
    frontier_Hcost.resize(getNumberOfIntersections()+1);
    
    std::vector<unsigned> frontier_parent;
    frontier_parent.resize(getNumberOfIntersections()+1);
    
    //initialize the values in frontier 
    frontier_exist [intersect_id_start] = true;
    frontier_cost[intersect_id_start] = 0.0;
    //calculate the heuristic cost of the node 
    frontier_Hcost[intersect_id_start] = 0.0 + (1 / maximum_speed)*(heuristic_factor)*find_distance_between_two_points(
            getIntersectionPosition(intersect_id_start),getIntersectionPosition(intersect_id_end));
    //set the parent value into an infinite number which means is hasnt been visited
    frontier_parent[intersect_id_start] = ID_NULL;
    //push the node in 
    frontier.push( std::make_pair(intersect_id_start,frontier_Hcost[intersect_id_start]) );
    
    
    do{
        //if frontier is empty then return failure
        if (frontier.empty() == true){
            break;          
        }
        
        //when the first element of frontier is the same as the end intersection that we are looking for  
        if (frontier.top().first == intersect_id_end){            
            return frontier.top().second;
        }
        
        //this is the node that we find should be expanded
        unsigned node_to_expand_id;
              
        node_to_expand_id =  frontier.top().first;
        frontier.pop();
       
        if (explored[node_to_expand_id].visited == true){ 
            continue;
        }
                      
        std::vector<unsigned> child = find_adjacent_intersections(node_to_expand_id);
        explored[node_to_expand_id].parent = frontier_parent[node_to_expand_id];
        explored[node_to_expand_id].visited = true;
        //for each child we need 
        for (unsigned current_child = 0; current_child < child.size(); current_child ++ ){
            //if the child we are looking for is in the explored
            if ( explored[child[current_child]].visited == true  ){
                
            }
            else if ( frontier_exist[child[current_child]] == true ){
                
                double current_cost = frontier_cost[node_to_expand_id];
                unsigned current_segment = find_street_segment_from_intersection_IDs(node_to_expand_id, child[current_child]);
                    current_cost = current_cost + find_street_segment_travel_time(current_segment);
                    unsigned current_street = getStreetSegmentInfo(current_segment).streetID;

                    if (frontier_parent[node_to_expand_id] != ID_NULL){
                        unsigned previous_segment = find_street_segment_from_intersection_IDs(frontier_parent[node_to_expand_id],node_to_expand_id); 
                        unsigned previous_street = getStreetSegmentInfo(previous_segment).streetID;
                        if (current_street!=previous_street){
                            current_cost = current_cost + turn_penalty;
                        }
                    }
                 double H_cost = current_cost + (1/maximum_speed)*(heuristic_factor)*find_distance_between_two_points(
                                getIntersectionPosition(child[current_child]),getIntersectionPosition(intersect_id_end));
                 //check the cost of the heuristic cost and the frontier cost 
                 //if the cost is smaller than replace
                if (H_cost < frontier_Hcost[ child[current_child] ] ){
                                        
                    frontier_cost[ child[current_child] ] = current_cost;
                    frontier_parent[ child[current_child] ] = node_to_expand_id;  
                    frontier_Hcost[ child[current_child] ] = H_cost;
                                       
                    frontier.push(std::make_pair(child[current_child], H_cost));                  
                }                                     
            }
            //if the child is not in the explored or frontier
            else{
                double current_cost = frontier_cost[node_to_expand_id];
                unsigned current_segment = find_street_segment_from_intersection_IDs(node_to_expand_id, 
                                child[current_child]);
                        current_cost = current_cost + find_street_segment_travel_time(current_segment);
                unsigned current_street = getStreetSegmentInfo(current_segment).streetID;
                
                        if (frontier_parent[node_to_expand_id] != ID_NULL){
                            unsigned previous_segment = find_street_segment_from_intersection_IDs(frontier_parent[node_to_expand_id],node_to_expand_id); 
                            unsigned previous_street = getStreetSegmentInfo(previous_segment).streetID;
                            if (current_street!=previous_street){
                                current_cost = current_cost + turn_penalty;
                            }
                        }
                
                 double H_cost = current_cost + (1/maximum_speed)*(heuristic_factor)*find_distance_between_two_points(
                                getIntersectionPosition(child[current_child]),getIntersectionPosition(intersect_id_end));
                
                frontier.push(std::make_pair(child[current_child], H_cost));                                          
                frontier_exist [ child[current_child] ] = true;       
                frontier_cost[ child[current_child] ] = current_cost;
                frontier_parent[ child[current_child] ] = node_to_expand_id;
                frontier_Hcost[ child[current_child] ] = H_cost;                            
            }                             
        }   
    }while(true);
    
    return 0.0;
}

