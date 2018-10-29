/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <iostream>
#include <string>
#include <chrono>
#include <vector>
#include <list>
#include <unordered_map>
#include <time.h> 
#include <chrono>
#include <cstdlib> 
#include <ctime> 


#include "m1.h"
#include "m3.h"
#include "m4.h"
#include "StreetsDatabaseAPI.h"

// m3 new function
double find_time_between_intersections_fast(const unsigned intersect_id_start, 
                                                      const unsigned intersect_id_end,
                                                      const double turn_penalty);


std::vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries,
    const std::vector<unsigned>& depots, const float turn_penalty){
      
    auto start = std::chrono::high_resolution_clock::now();
    
    std::vector <unsigned> path;
    std::vector <unsigned> delivery_path; 
    std::vector <unsigned> valid_depots;
    // a vector of intersections in the order of visiting
    
    
    // get the valid depots, ignore the depot that can not be connected to any delivery
    // point
    for (unsigned  depot_index = 0; depot_index < depots.size(); depot_index ++){
        if (find_path_between_intersections(depots[depot_index],deliveries[0].pickUp,turn_penalty).size() == 0){
            
        }
        else {
            valid_depots.push_back(depots[depot_index]);
        } 
    }
    
    
    // valid points to visit
    std::list<unsigned> packages_to_deliver; 
    
    for (unsigned delivery_index = 0; delivery_index < deliveries.size(); delivery_index++ ){
        packages_to_deliver.push_back(deliveries[delivery_index].pickUp);
    }
    
    unsigned current_location = valid_depots[0];
    delivery_path.push_back(valid_depots[0]);
    
    
    while (packages_to_deliver.empty() == false){
        double shortest_distance = 9999999.99;
        unsigned intersection_to_visit;
        std::list<unsigned> :: iterator intersection_to_visit_it;
        
        for (auto it = packages_to_deliver.begin(); it != packages_to_deliver.end(); it ++){
            double distance = find_distance_between_two_points(getIntersectionPosition(current_location),getIntersectionPosition(*it));
            
            if ( distance < shortest_distance ){
                shortest_distance = distance;
                intersection_to_visit = *it;
                intersection_to_visit_it = it;
            } 
            
        }
              
        for (unsigned deliveries_index = 0; deliveries_index < deliveries.size(); deliveries_index++){
            if (intersection_to_visit == deliveries[deliveries_index].pickUp){
                packages_to_deliver.push_back( deliveries[deliveries_index].dropOff);
                // break;
            }  
        }
        
        current_location = intersection_to_visit;
        packages_to_deliver.erase(intersection_to_visit_it);
        delivery_path.push_back(intersection_to_visit); 
        
    }
    
    unsigned depot_to_return;
    double shortest_distance = 9999999.99;
    
    for (unsigned  depot_index = 0; depot_index < valid_depots.size(); depot_index ++){
        double distance = find_distance_between_two_points( 
            getIntersectionPosition(current_location),getIntersectionPosition( valid_depots[depot_index]) );
            if ( distance < shortest_distance ){
                shortest_distance = distance;
                depot_to_return = valid_depots[depot_index];
            } 
    }
    
    delivery_path.push_back(depot_to_return); 
    
   
    // getting the delivery order from greedy algorithm
   
    
    
    //******************************************************************************************
    // start doing two_opt
    std::unordered_map <unsigned,unsigned> intersection_id_to_index;
    
    for (unsigned delivery_index = 0; delivery_index < deliveries.size(); delivery_index++){       
        intersection_id_to_index.insert( {deliveries[delivery_index].pickUp, 2*delivery_index } );
        intersection_id_to_index.insert( {deliveries[delivery_index].dropOff, 2*delivery_index + 1 } );   
    }
    for (unsigned depot_index = 0; depot_index < valid_depots.size(); depot_index++) {
        intersection_id_to_index.insert( {valid_depots[depot_index], deliveries.size() * 2 + depot_index} );
    }
    
    unsigned num_points = deliveries.size() * 2 + depots.size();
    
    std::vector<std::vector<double> > traveling_time_vector; 
    traveling_time_vector.resize(num_points);
    
    for (unsigned start = 0; start < num_points; start++){
        for (unsigned end = 0; end < num_points; end++){
            traveling_time_vector[start].resize(num_points);
            traveling_time_vector[start][end] = -1;
        }
    }
    
    double best_travel_time = 0.0;
    for (unsigned path_index = 0; path_index < delivery_path.size() - 1; path_index++){
        double travel_time = 
        find_time_between_intersections_fast(delivery_path[path_index],delivery_path[path_index+1],turn_penalty);
        
        traveling_time_vector[intersection_id_to_index[delivery_path[path_index]]]
                [intersection_id_to_index[delivery_path[path_index+1]]] = travel_time;
        best_travel_time = best_travel_time + travel_time;            
    }
    
    
    auto end = std::chrono::high_resolution_clock::now();
    auto delta_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start);
      
    int total_attempt = 0;
    int valid_swap_count = 0;
    int successful_swap = 0;
    
    
    
      
    std::cout << "num_points: " << num_points << std::endl;
    
    // std::cout << traveling_time_vector[intersection_id_to_index[20499]][intersection_id_to_index[261649]] << std::endl;
    
    
    
    
    
    while (delta_time.count() < 26000000000){
        total_attempt ++;
             
        srand(delta_time.count()); 
        unsigned swap_1 = rand()% (deliveries.size()*2)+1;
        unsigned swap_2 = rand()% (deliveries.size()*2)+1;
       
        if ( swap_1 > swap_2 ){
            unsigned temp = swap_1;
            swap_1 = swap_2;
            swap_2 = temp; 
        }
        
        // cheak if the swap if valid
        bool valid_swap = true;
        
        bool swap_1_is_pickUp = false;
        bool swap_2_is_dropOff = false;
        
        unsigned swap_1_dropOff;
        unsigned swap_2_pickup;
        
        if ( intersection_id_to_index[delivery_path[swap_1]] % 2 == 0){
            swap_1_is_pickUp = true;
            swap_1_dropOff = deliveries[ intersection_id_to_index[delivery_path[swap_1]]/2 ].dropOff;
        }
                     
        if (  intersection_id_to_index[delivery_path[swap_2]] % 2 == 1){
            swap_2_is_dropOff = true;
            swap_2_pickup = deliveries[ (intersection_id_to_index[delivery_path[swap_2]]-1) /2 ].pickUp;
        }
        
        if (swap_1_is_pickUp || swap_2_is_dropOff){
            for (unsigned delivery_index = swap_1; delivery_index < swap_2; delivery_index++ ){
                if (  (delivery_path[delivery_index] == swap_1_dropOff) || (delivery_path[delivery_index] == swap_2_pickup) ){
                    valid_swap = false;
                }                 
            } 
        }
        
        if (valid_swap == true){
            valid_swap_count ++ ;
            
            unsigned temp1 = delivery_path[swap_1]; 
            unsigned temp2 = delivery_path[swap_2];
            delivery_path[swap_1] = temp2;
            delivery_path[swap_2] = temp1;
            
            double total = 0.0;
            
           for (unsigned path_index = 0; path_index < delivery_path.size() - 1; path_index++){
               
               if (traveling_time_vector[intersection_id_to_index[delivery_path[path_index]]]
                        [intersection_id_to_index[delivery_path[path_index+1]]] < 0){                        
                    double travel_time = 
                        find_time_between_intersections_fast(delivery_path[path_index],delivery_path[path_index+1],turn_penalty);      
                    total = total + travel_time; 
                    traveling_time_vector[intersection_id_to_index[delivery_path[path_index]]]
                        [intersection_id_to_index[delivery_path[path_index+1]]] = travel_time; 
               }
               else{
                    double travel_time = traveling_time_vector[intersection_id_to_index[delivery_path[path_index]]]
                        [intersection_id_to_index[delivery_path[path_index+1]]] ; 
                    total = total + travel_time;
               }
                                     
            }
            
            if (total >= best_travel_time){
                unsigned temp1 = delivery_path[swap_1]; 
                unsigned temp2 = delivery_path[swap_2];
                delivery_path[swap_1] = temp2;
                delivery_path[swap_2] = temp1;                             
            }
            else{
                std::cout << total - best_travel_time <<" improve"<< std::endl;
                best_travel_time = total;
                successful_swap ++ ; 
            }                                  
        }   
      
       end = std::chrono::high_resolution_clock::now();
       delta_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start);        
    }
            
    // ***************************************************************************************

    std::cout << "total attempt: "<< total_attempt << std::endl;
    std::cout << "valid swap: "<< valid_swap_count <<std::endl;
    std::cout << "successful: " << successful_swap << std::endl;
    
    
    // converting the intersection vector(delivery path) to the street_segment path
    for (unsigned path_index = 0; path_index < delivery_path.size() - 1; path_index++){
        std::vector <unsigned> current_path = 
            find_path_between_intersections(delivery_path[path_index],delivery_path[path_index+1],turn_penalty);

        path.insert( path.end(), current_path.begin(), current_path.end() );             
    }
      
    
    return path;
}


