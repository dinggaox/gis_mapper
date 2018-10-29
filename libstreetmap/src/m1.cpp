/* 
 * Copyright 2018 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "m1.h"
#include <string>
#include "StreetsDatabaseAPI.h"
#include <math.h>
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <limits>
#include <unordered_map>


// Useful variable that may be used in functions
//unsigned numberOfStreets = getNumberOfStreets();
//unsigned numberOfStreetSegments = getNumberOfStreetSegments();
//unsigned numberOfIntersections = getNumberOfIntersections();
//unsigned numberOfPOI = getNumberOfPointsOfInterest();
//unsigned numberOfFeatures = getNumberOfFeatures();

std::vector <std::vector<unsigned>> all_intersections_and_their_connected_segments;
double* all_street_segments_and_their_lengths;
double* speed_limit_of_street_segments;
std::vector <std:: vector<unsigned> > street_Street_Segment_List;
std::vector <std:: vector<unsigned> > street_Street_Intersection_List;
std::unordered_map < std::string, std::vector<unsigned> > street_map;

std::unordered_map <std::string,std::vector<unsigned> > POI_map;
std::vector <unsigned> map_POI_to_nearest_intersections;
std::vector <std::vector<unsigned> > map_street_segment_to_intersections; 


namespace BooGeo = boost::geometry;
namespace BooGeoIdx = boost::geometry::index;

struct MyLatLon{

    LatLon thisPoint;
    MyLatLon() {}
    MyLatLon(float lat_, float lon_) : thisPoint(lat_, lon_){}

    float get_lat() const { return thisPoint.lat(); }
    float get_lon() const { return thisPoint.lon(); }
    void set_lat(float v) { thisPoint = LatLon(v, thisPoint.lon()); }
    void set_lon(float v) { thisPoint = LatLon(thisPoint.lat(), v); }


};

//Allows boost::geometry to correctly recognize the structure MyLatLon
//as a valid data type (pair) for it to use
BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(MyLatLon, float,
        BooGeo::cs::spherical_equatorial<BooGeo::degree>,
        get_lon, get_lat, set_lon, set_lat)


typedef std::pair <MyLatLon, unsigned> node;
BooGeoIdx::rtree <node, BooGeoIdx::quadratic<16>> intersectionRtree;
BooGeoIdx::rtree <node, BooGeoIdx::quadratic<16>> pointOfInterestRtree;

bool load_map(std::string map_path) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully
    load_successful = loadStreetsDatabaseBIN(map_path);
    
    
    
    
    // New POI data structure for m3
  
    if (load_successful == true){
        unsigned numberOfPOI = getNumberOfPointsOfInterest();
        
        for (unsigned i = 0; i < numberOfPOI; i++){     
            if (POI_map.find(getPointOfInterestName(i)) == POI_map.end() ){
                std::vector<unsigned> POI_ID;
                POI_ID.push_back(i);
                POI_map[ getPointOfInterestName(i) ] = POI_ID; 
            }
            else{
                auto POI_vector = POI_map[getPointOfInterestName(i)];
                POI_vector.push_back(i);
                POI_map[ getPointOfInterestName(i) ] = POI_vector;
            }            
        }
    }
       
    
    //STRUCTURE 1
    if (load_successful == true){
        // A vector<unsigned> array of street list, then put the street segment belongs
        // to it in the vector
        unsigned numberOfStreets = getNumberOfStreets();
        unsigned numberOfStreetSegments = getNumberOfStreetSegments();

        street_Street_Segment_List.resize(numberOfStreets);
        
        for (unsigned i = 0; i < numberOfStreetSegments; i++){
            StreetSegmentInfo Current_streetSegment = getStreetSegmentInfo(i);
            unsigned StreetItBelongsto = Current_streetSegment.streetID; 
            street_Street_Segment_List[StreetItBelongsto].push_back(i);
        }
    }
   
    //STRUCTURE 2
    // an unordered map of street names with a vector of their ID(s)
    // each key is only allowed once
    if (load_successful == true){
        unsigned numberOfStreets = getNumberOfStreets();
        
        for (unsigned i = 0; i < numberOfStreets; i++){
                     
            if (street_map.find(getStreetName(i)) == street_map.end() ){
                std::vector<unsigned> Street_ID;
                Street_ID.push_back(i);
                street_map[ getStreetName(i) ] = Street_ID; 
            }
            else{
                auto Street_ID_vector = street_map[getStreetName(i)];
                Street_ID_vector.push_back(i);
                street_map[ getStreetName(i) ] = Street_ID_vector;
            }
              
        }
    }
    
    
    //STRUCTURE 3
    //A vector of vectors, where
    //the outer vector is the IDs of every intersection, and
    //the inner vector is each street segment connected by the ID
    if (load_successful){
        
        unsigned numberOfIntersections = getNumberOfIntersections();
        
        for (unsigned currentIntersection = 0; currentIntersection < numberOfIntersections; ++currentIntersection){
            
            unsigned numberOfSegmentsHere = getIntersectionStreetSegmentCount(currentIntersection);
            all_intersections_and_their_connected_segments.resize(currentIntersection + 1);
            
            for (unsigned currentSegment = 0; currentSegment < numberOfSegmentsHere; ++currentSegment){
                
                all_intersections_and_their_connected_segments[currentIntersection].push_back(getIntersectionStreetSegment(currentIntersection,currentSegment));
            
            }
        }
    }
    
    //STRUCTURE 4
    // Two Street-Segments Array
    // store length and speed limit of each street segment 
    if (load_successful){
        
        unsigned numberOfStreetSegments = getNumberOfStreetSegments();
        all_street_segments_and_their_lengths = new double[numberOfStreetSegments];
        speed_limit_of_street_segments = new double[numberOfStreetSegments];
        
        double length;
        LatLon startLatLon, endLatLon;
        
        for (unsigned currentStreetSegment = 0; currentStreetSegment < numberOfStreetSegments; currentStreetSegment++){
            
            speed_limit_of_street_segments[currentStreetSegment]=getStreetSegmentInfo(currentStreetSegment).speedLimit;
                    
            //all_street_segments_and_their_lengths.resize(currentStreetSegment + 1);
            length = 0;
            
            startLatLon = getIntersectionPosition(getStreetSegmentInfo(currentStreetSegment).from);
            endLatLon = getIntersectionPosition(getStreetSegmentInfo(currentStreetSegment).to);

            unsigned numCurvePoints = getStreetSegmentInfo(currentStreetSegment).curvePointCount;

            //std::vector <LatLon> segmentPoints = getStreetSegmentCurvePoint(street_segment_id);
            //when there is no curve point, then the length the just the distance between two points
            
            if(numCurvePoints == 0){
                length = find_distance_between_two_points(startLatLon, endLatLon);
            }
            else {
                for (unsigned i = 0; i < numCurvePoints; i++){
                    LatLon segmentPoints = getStreetSegmentCurvePoint(currentStreetSegment, i);
                    //i = 0 is the first point on the segment now, this calculates the length between the start point to the first element of the segment
                    if (i == 0){
                        length += find_distance_between_two_points(startLatLon, segmentPoints);
                    }
                    //this calculates the middle part of the segment
                    else{
                        length += find_distance_between_two_points(getStreetSegmentCurvePoint(currentStreetSegment, i - 1), segmentPoints);
                    }
                    //this calculates the length from the last curve point to the end point
                    if (i == numCurvePoints - 1)
                        length += find_distance_between_two_points(segmentPoints, endLatLon);
                }
            }
            all_street_segments_and_their_lengths[currentStreetSegment] = length;
        }
    }

    
    // structure 5
    // a vector of vector
    // each outer vector represents the streets
    // the inner vector stored all streets' intersections (NO duplicate, SORTED)
    if (load_successful){
        unsigned numberOfStreets = getNumberOfStreets();
       
        street_Street_Intersection_List.resize(numberOfStreets+1);
        
        for (unsigned current_street_id = 0; current_street_id < numberOfStreets; current_street_id++){
            std :: vector<unsigned> all_street_intersections;
    
            // the vector that contains all street segments // street_Street_Segment_List[street_id];
            unsigned numSegmentConnected = street_Street_Segment_List[current_street_id].size();
    
            for (unsigned i = 0; i < numSegmentConnected; i++){
                unsigned street_segment_index = street_Street_Segment_List[current_street_id][i];
                all_street_intersections.push_back( getStreetSegmentInfo(street_segment_index).from );
                all_street_intersections.push_back( getStreetSegmentInfo(street_segment_index).to );
            }

            std :: sort(all_street_intersections.begin(),all_street_intersections.end());
            std :: vector<unsigned>::iterator it;
            it = std :: unique(all_street_intersections.begin(),all_street_intersections.end());
            all_street_intersections.resize(std::distance(all_street_intersections.begin(),it));

            street_Street_Intersection_List[current_street_id]= all_street_intersections;
            
        }
    }
    
    //STRUCTURE 6
    //r-tree storing all intersections as nodes
    if (load_successful){
        unsigned numberOfIntersections = getNumberOfIntersections();

        for(unsigned currentIntersection = 0;  currentIntersection < numberOfIntersections; ++currentIntersection){
            MyLatLon currentLatLon = MyLatLon(getIntersectionPosition(currentIntersection).lat(),getIntersectionPosition(currentIntersection).lon());
            intersectionRtree.insert(std::make_pair(currentLatLon,currentIntersection));
        }
    }
    
    //STRUCTURE 7
    //r-tree storing all points of interests as nodes
     if (load_successful){
        unsigned numberOfPointofInterest = getNumberOfPointsOfInterest();

        for(unsigned currentPointofInterest = 0;  currentPointofInterest < numberOfPointofInterest; ++currentPointofInterest){
            MyLatLon currentLatLon = MyLatLon(getPointOfInterestPosition(currentPointofInterest).lat(),getPointOfInterestPosition(currentPointofInterest).lon());
            pointOfInterestRtree.insert(std::make_pair(currentLatLon,currentPointofInterest));
        }
    }
    
//    //Couting for unit test IMPORTANT
//    std::vector<unsigned> expectedStreetIDs;
//    for (unsigned i = 0; i < getNumberOfStreets(); ++i){
//        if (getStreetName(i) == "<unknown>"){
//            expectedStreetIDs.push_back(i);
//        }
//    }
//    std::sort(expectedStreetIDs.begin(), expectedStreetIDs.end());
//    //std::cout << expectedStreetIDs << std::endl;
//    for (unsigned i = 0; i < expectedStreetIDs.size(); ++i){
//        std::cout << expectedStreetIDs[i] << std::endl;
//    }
    return load_successful;
}


void close_map() {
    //Clean-up your map related data structures here
    delete [] all_street_segments_and_their_lengths;
    delete [] speed_limit_of_street_segments;
    closeStreetDatabase();  
}

// function # 9 
// Returns the distance between two coordinates in meters
double find_distance_between_two_points(LatLon point1, LatLon point2){

    double distance;
    double lat_average;
    lat_average = (point1.lat() + point2.lat()) / 2 * DEG_TO_RAD;
    double x1, x2, y1, y2;
       
    y1 = DEG_TO_RAD * point1.lat();
    y2 = DEG_TO_RAD * point2.lat();
    
    x1 = DEG_TO_RAD * point1.lon() * cos (lat_average);
    x2 = DEG_TO_RAD * point2.lon() * cos (lat_average);
    
    distance = EARTH_RADIUS_IN_METERS * sqrt (pow((y2 -y1), 2) + pow((x2 -x1), 2));
        
    return distance;
     
}

// function # 10
//Returns the length of the given street segment in meters
double find_street_segment_length(unsigned street_segment_id){
    
    return all_street_segments_and_their_lengths[street_segment_id];
    
}

// function # 11 
//Returns the length of the specified street using the length of all the segments in one street in meters
double find_street_length(unsigned street_id){
    
    double length = 0;
    //unsigned street_ID = getStreetSegmentInfo(street_id).streetID;
    
    std::vector<unsigned> allSegmentsInStreet = find_street_street_segments(street_id);
    
    unsigned numberOfSegmentsInStreet = allSegmentsInStreet.size();
    
    for (unsigned i = 0; i < numberOfSegmentsInStreet; i++){
        
        length += find_street_segment_length(allSegmentsInStreet[i]);
    }
    
    return length; 
}


// function #12
//Returns the travel time to drive a street segment in seconds 
//(time = distance/speed_limit)
double find_street_segment_travel_time(unsigned street_segment_id){
    
    double time;
    float distance = find_street_segment_length(street_segment_id);
    double speedLimit = speed_limit_of_street_segments[street_segment_id];
        
    time = 3600 * ((distance / 1000) / (speedLimit));
    return time;
}

// function # 6
//Returns all street segments for the given street
std::vector<unsigned> find_street_street_segments(unsigned street_id){
    std :: vector<unsigned> current (street_Street_Segment_List[street_id]);
    
    std :: sort(current.begin(),current.end());
    std :: vector<unsigned>::iterator it;
    it = std :: unique(current.begin(),current.end());
    current.resize(std::distance(current.begin(),it));
    
    return current;
}




// function # 7
//Returns all intersections along the a given street
// return the Street_intersection vector in the street_vector
std::vector<unsigned> find_all_street_intersections(unsigned street_id){
        
    return street_Street_Intersection_List[street_id];   
}

// function # 8
//Return all intersection ids for two intersecting streets
//This function will typically return one intersection id.
//However street names are not guarenteed to be unique, so more than 1 intersection id
//may exist

std::vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1, 
                                                              std::string street_name2){
  
    std::vector<unsigned> intersection_of_two_streets; 
     
        std::set<unsigned> temp1; // SET is a indexed Binary Search Tree that has NO duplicates

        // get street IDs from street_name input
        std::vector<unsigned> street_id_1 = street_map[street_name1];
        std::vector<unsigned> street_id_2 = street_map[street_name2];

        for (unsigned i = 0; i < street_id_1.size();i++){
            for (unsigned j = 0; j < street_id_2.size();j++){
                std::vector<unsigned> intersection1(street_Street_Intersection_List[street_id_1[i]] );
                std::vector<unsigned> intersection2(street_Street_Intersection_List[street_id_2[j]] );
                
                unsigned size1 = intersection1.size();
                unsigned size2 = intersection2.size();
                unsigned counter1 = 0;
                unsigned counter2 = 0;
                
                // comparing two sorted vector to find the common data
                while ((counter1 < size1) && (counter2 < size2) ){
                    if (intersection1[counter1] < intersection2[counter2]){
                        counter1 = counter1 + 1;
                    }
                    
                    else if (intersection1[counter1] > intersection2[counter2]){
                        counter2 = counter2 + 1;
                    }
                    
                    else if (intersection1[counter1] == intersection2[counter2]){
                        temp1.insert(intersection1[counter1]);
                        counter1 = counter1 + 1;
                        counter2 = counter2 + 1;                      
                    }  
                }
                     
            }
        }   
 
        
        for (auto i : temp1)
            intersection_of_two_streets.push_back(i);
            
    return intersection_of_two_streets;
}


// function 13
//Returns the nearest point of interest to the given position
unsigned find_closest_point_of_interest(LatLon my_position){
  
     std::vector</*std::pair<MyLatLon, unsigned*/ node> closestPointofInterest;
    MyLatLon currentLatLon = MyLatLon(my_position.lat(),my_position.lon());
    pointOfInterestRtree.query(BooGeoIdx::nearest(currentLatLon,1),std::back_inserter(closestPointofInterest));
    
    return closestPointofInterest[0].second;
}

// function 14
//Returns the the nearest intersection to the given position
unsigned find_closest_intersection(LatLon my_position){
   
    std::vector</*std::pair<MyLatLon, unsigned*/ node> closestIntersection;
    MyLatLon currentLatLon = MyLatLon(my_position.lat(),my_position.lon());
    intersectionRtree.query(BooGeoIdx::nearest(currentLatLon,1),std::back_inserter(closestIntersection));
    
    return closestIntersection[0].second;   
}

// function # 1
std::vector<unsigned> find_street_ids_from_name(std::string street_name){
    
    unsigned numberOfStreets = getNumberOfStreets();
       
    std::vector<unsigned> streetIDsWithName;
    
    for (unsigned currentStreetID = 0; currentStreetID < numberOfStreets; ++currentStreetID){
        if (street_name == getStreetName(currentStreetID)){
            streetIDsWithName.push_back(currentStreetID);
        }
    }
    return streetIDsWithName;
}

// function # 2
std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id){
     
    return all_intersections_and_their_connected_segments[intersection_id];
}

// function # 3
//Returns the street names at the given intersection (includes duplicate street 
//names in returned vector)
std::vector<std::string> find_intersection_street_names(unsigned intersection_id){
 
    std::vector<std::string> streetsConnectedToIntersection;
    unsigned numberOfSegmentsConnectedToIntersection = getIntersectionStreetSegmentCount(intersection_id);
    
    for (unsigned currentSegment = 0; currentSegment < numberOfSegmentsConnectedToIntersection; ++currentSegment){
        
        streetsConnectedToIntersection.push_back
        (getStreetName(getStreetSegmentInfo(getIntersectionStreetSegment(intersection_id, currentSegment)).streetID));
        // ** vector "getStreetSegmentInfo" only takes string argument, need to get the street name by its ID 
    }
    
    return streetsConnectedToIntersection;
}

// function 4
bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2){

    unsigned numberOfSegmentsConnectedToID1 = getIntersectionStreetSegmentCount (intersection_id1);
    // unsigned numberOfSegmentsConnectedToID2 = getIntersectionStreetSegmentCount (intersection_id2);

    std::vector<unsigned> segmentsConnectedToID1;
    std::vector<unsigned> segmentsConnectedToID2;
    
    segmentsConnectedToID1 = find_intersection_street_segments (intersection_id1);
    segmentsConnectedToID2 = find_intersection_street_segments (intersection_id2);   
    
    if (intersection_id1 == intersection_id2){
        return true;
    }
    
    for (unsigned currentSegment = 0; currentSegment < numberOfSegmentsConnectedToID1; ++currentSegment){
        if (getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).from == intersection_id1
                && getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).to == intersection_id2){
            return true;
        }
        if (getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).from == intersection_id2
                && getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).to == intersection_id1
                && getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).oneWay == false){
            return true;
        }
    }
    return false;
}


// function 5
std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id){
    
    unsigned size = 0;
    bool found;
    unsigned numberOfSegmentsConnectedToID = getIntersectionStreetSegmentCount (intersection_id);

    std::vector<unsigned> segmentsConnectedToID = find_intersection_street_segments (intersection_id);
    
    unsigned otherEndOfSegment;
    
    std::vector<unsigned> allAdjacentIntersections;
    
    for (unsigned currentSegment = 0; currentSegment < numberOfSegmentsConnectedToID; ++currentSegment){
        
        //Checks which intersection is "from" and which intersection is "to"
        if (getStreetSegmentInfo(segmentsConnectedToID[currentSegment]).from == intersection_id){
            otherEndOfSegment = getStreetSegmentInfo(segmentsConnectedToID[currentSegment]).to;
        }
        else {
            otherEndOfSegment = getStreetSegmentInfo(segmentsConnectedToID[currentSegment]).from;
        }
        
        //Checks if "from" is directly connected to "to"
        if (are_directly_connected (intersection_id, otherEndOfSegment)){

            found = false;
            for (unsigned currentSegCheck = 0; currentSegCheck < size; ++currentSegCheck){
                if (allAdjacentIntersections[currentSegCheck] == otherEndOfSegment){
                    found = true;
                }
            }
            if (!found){
                allAdjacentIntersections.push_back(otherEndOfSegment);
                size ++;
            }
        }
    }
    return allAdjacentIntersections;
}


// additional functions
// return the street_segment ID for two intersection_ids;
// Assume they are not the same intersection
// And intersection 1 is connected to intersection 2

unsigned find_street_segment_from_intersection_IDs(unsigned intersection_id1, unsigned intersection_id2){
    unsigned numberOfSegmentsConnectedToID1 = getIntersectionStreetSegmentCount (intersection_id1);
 
    std::vector<unsigned> segmentsConnectedToID1;
    segmentsConnectedToID1 = find_intersection_street_segments (intersection_id1);
 
    std::vector<unsigned> found;
    for (unsigned currentSegment = 0; currentSegment < numberOfSegmentsConnectedToID1; ++currentSegment){
       
        if (getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).from == intersection_id1
                && getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).to == intersection_id2){
                found.push_back(segmentsConnectedToID1[currentSegment]);          
        }
        
        else if (getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).from == intersection_id2
                && getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).to == intersection_id1
                && getStreetSegmentInfo(segmentsConnectedToID1[currentSegment]).oneWay == false ){
             found.push_back(segmentsConnectedToID1[currentSegment]);  
        }
    }
    
    unsigned shortest_travel_time_segment;
    double shortest_travel_time = 99999999;
    
    for (unsigned currentSegment = 0; currentSegment < found.size(); currentSegment++){
        double current_time = find_street_segment_travel_time(found[currentSegment]);
        if (current_time < shortest_travel_time){
            shortest_travel_time = current_time;
            shortest_travel_time_segment = found[currentSegment];
        }    
    }
    
    return shortest_travel_time_segment;
}


std::vector<unsigned> intersection_to_go_for_POI(std::string POI_name){
    std::vector<unsigned> intersection_to_go;
    std::vector<unsigned> POI_IDs = POI_map[POI_name];
    
        for (unsigned current_POI = 0; current_POI < POI_IDs.size(); current_POI++){
            intersection_to_go.push_back( find_closest_intersection( getPointOfInterestPosition(POI_IDs[current_POI]) ));
        } 
      
    return intersection_to_go;
}
