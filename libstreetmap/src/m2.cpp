/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <iomanip>
#include <cstdio>
#include <ctime>

#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "LatLon.h"
#include "Feature.h"
#include "OSMID.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "graphics.h"
#include <X11/keysym.h>

void drawscreen(void);
void act_on_new_button_func(void (*drawscreen_ptr) (void));
void act_on_button_press(float x, float y, t_event_buttonPressed event);
void act_on_mouse_move(float x, float y);
void act_on_keypress(char c, int keysym);
void show_POI(void (*draw_screen) (void));
void show_POI_name(void (*draw_screen) (void));
void clear(void (*draw_screen) (void));
void find_and_highlight(void (*draw_screen) (void));
double LatLonToX(const LatLon&, double);
double LatLonToY(const LatLon&);
double computePolygonArea(unsigned int, t_point*);
void draw_screen();
void drawErrorMessages(std::string Error);
void highlight_segments (std::vector<unsigned> segments);

unsigned intersection_to_highlight;
#define nullptr nullptr

std::vector <unsigned> highlighted_segments = {};
std::vector <unsigned> highlighted_points = {64489,10192,66348,47055};

// Global flag
bool draw_POI = true;
bool draw_POI_name = true;
bool draw_street_name = true;
bool draw_intersections = true;
bool draw_features = true;
bool draw_streets = true;
bool in_directions = false;
bool in_find_POI = false;

static t_point rubber_pt; // Last point to which we rubber-banded.
static std::vector<t_point> line_pts; // Stores the points entered by user clicks.

struct intersection_data {
    LatLon position;
    std::string name;
};

struct each_segment_data {
    unsigned ID;
    OSMID wayOSMID;   
    LatLon from, to;
    bool oneWay;
    bool highlight;
    unsigned curvePointCount;
    std::vector <LatLon> curvePointsPositions;
    float speedLimit;
    StreetIndex	streetID;  
};

struct colour {
    unsigned r;
    unsigned g;
    unsigned b;
};

struct street_data {
    std::vector <each_segment_data> segments;
    std::string name;
    double length;
    unsigned width;
    unsigned intersectionCount;
    double avgSpeed;
};

struct feature_data {
    std::string name;
    FeatureType type;
    TypedOSMID OSMID;
    unsigned featurePointCount;
    //std::vector <LatLon> featurePointsPositions;
    std::vector <t_point> featurePointsXY;
    bool closed;
    double area;
    double length;
};

struct poi_data {
    std::string name;
    std::string type;
    t_point position;
    OSMID nodeOSMID;
};

void in_directions_mode (void (*draw_screen) (void));
void give_directions (const std::vector<unsigned>& segments_to_direct);
void clear_highlights();
void highlight_segments (std::vector<unsigned> segments);
void give_help(void (*draw_screen) (void));

double LatLonToX (const LatLon& latlon, double lat_avg){
    double x = DEG_TO_RAD * latlon.lon() * cos (lat_avg);
    return x;
}

double LatLonToY (const LatLon& latlon){
    double y = DEG_TO_RAD * latlon.lat();
    return y;
}
double compute_area;
double computePolygonArea(unsigned arraySize, t_point* points){
    compute_area = 0;
    //the last vertex is the previous vertex to the first
    int j = arraySize - 1;
    
    for (unsigned i = 0; i < arraySize; i++){
        compute_area = compute_area + (points[j].x + points[i].x) * (points[j].y - points[i].y);
        j = i;
    }
    return compute_area / 2;
}

//constants and global variables
double max_lat;
double min_lat;
double max_lon;
double min_lon;
double avg_lat;
double const ratio = 0.000001; 
double const big_street_ratio = 0.13;
double const small_street_ratio = 0.05;

//intersections:
double intersection_x;
double intersection_y;
double const intersection_relative_radius = 1;
double intersection_radius = intersection_relative_radius * ratio;
//features:
LatLon currentLatLon;
t_point* closed_points;
t_point* open_points;
double feature_x1;
double feature_y1;
double feature_x2;
double feature_y2;
double big_feature_width;
double small_feature_width;
unsigned feature_points_count;
double feature_area;
double feature_length;
double feature_area_ratio;
double feature_length_ratio;
//streets:
unsigned const street_width = 1;
double street_x1 = 0.0;
double street_y1 = 0.0;
double street_x2 = 0.0;
double street_y2 = 0.0;
double x_tolerence = 0.0;
double y_tolerence = 0.0;
double huge_street_width = 5;
double big_street_width = 3;
double small_street_width = 1;
//POIs:
double POI_x;
double POI_y;
double const POI_relative_radius = 2;
double POI_radius = POI_relative_radius * ratio/1.2;
//Directions:
unsigned click_num;
unsigned from_ID;
unsigned to_ID;

double scale = 1;
double max_width;
double current_width;
t_bound_box visible_world;

std::vector<intersection_data> intersections;
std::vector<street_data> streets;
std::vector<feature_data> features;
std::vector<poi_data> POIs;
std::vector<each_segment_data> segments;

// draw different elements on screen in the following order
// features >> Street >> intersection >> POI
// off screen buffer is used to minimize screen flashes
void draw_screen()
{
    clock_t start;
    //double draw_time;
    start = std::clock();
    
    clearscreen();
    set_drawing_buffer(OFF_SCREEN);
    
    visible_world = get_visible_world();
    current_width = visible_world.right() - visible_world.left();
    scale = current_width / max_width;
  
    
//DRAWING FEATURES    
    if (draw_features){
        unsigned number_of_features = getNumberOfFeatures();
        for (unsigned current_feature = 0; current_feature < number_of_features; ++current_feature){

            if (features[current_feature].closed == true){          
                feature_area_ratio = features[current_feature].area/(max_width * max_width);
               
                if (feature_area_ratio >= (0.00005 * scale * 0.6)){
                    if (features[current_feature].type == Unknown){
                        setcolor(0,0,0,255);
                    }
                    else if (features[current_feature].type == Park){
                        setcolor(102,204,0,255);
                    }
                    else if (features[current_feature].type == Beach){
                        setcolor(255,229,204);
                    }
                    else if (features[current_feature].type == Lake){
                        setcolor(14,162,216,255);
                    }
                    else if (features[current_feature].type == River){
                        setcolor(0,128,255,255);
                    }
                    else if (features[current_feature].type == Island){
                        setcolor(255,204,153,255);
                    }
                    else if (features[current_feature].type == Shoreline){
                        setcolor(255,255,68,255);
                    }
                    else if (features[current_feature].type == Building){
                        setcolor(200,200,200,255);
                    }
                    else if (features[current_feature].type == Greenspace){
                        setcolor(0,204,0,255);
                    }
                    else if (features[current_feature].type == Golfcourse){
                        setcolor(0,204,0,255);
                    }
                    else if (features[current_feature].type == Stream){
                        setcolor(102,178,255,255);
                    }

                    fillpoly(features[current_feature].featurePointsXY.data(), features[current_feature].featurePointCount);
                }
            }

            else {
                feature_length_ratio = features[current_feature].length/max_width;             
                    if (features[current_feature].type == Unknown){
                        setlinewidth (3);
                        setcolor(0,0,0,255);
                    }
                    else if (features[current_feature].type == Park){
                        setcolor(102,204,0,255);
                    }
                    else if (features[current_feature].type == Beach){
                        setcolor(255,229,204);
                    }
                    else if (features[current_feature].type == Lake){
                        setcolor(170,218,255,255);
                    }
                    else if (features[current_feature].type == River){
                        setlinewidth (6);
                        setcolor(0,128,255,255);
                    }
                    else if (features[current_feature].type == Island){
                        setcolor(255,204,153,255);
                    }
                    else if (features[current_feature].type == Shoreline){
                        setlinewidth (6);
                        setcolor(255,255,68,255);
                    }
                    else if (features[current_feature].type == Building){
                        setcolor(237,237,237,255);
                    }
                    else if (features[current_feature].type == Greenspace){
                        setcolor(192,236,174,255);
                    }
                    else if (features[current_feature].type == Golfcourse){
                        setcolor(0,204,0,255);
                    }
                    else if (features[current_feature].type == Stream){
                        setcolor(102,178,255,255);
                    }

                    for (unsigned current_point = 0; current_point < features[current_feature].featurePointCount - 1; ++current_point){
                        feature_x1 = features[current_feature].featurePointsXY[current_point].x;
                        feature_y1 = features[current_feature].featurePointsXY[current_point].y;

                        feature_x2 = features[current_feature].featurePointsXY[current_point + 1].x;
                        feature_y2 = features[current_feature].featurePointsXY[current_point + 1].y;                   
                        drawline(feature_x1, feature_y1, feature_x2, feature_y2);
                    }
              
            }
        }
    }

//DRAWING STREETS  
    
    highlight_segments (highlighted_segments);
    if (draw_streets){
        
        unsigned number_of_streets = getNumberOfStreets();
        
        big_street_width = 4*(1/scale * 0.1);
        small_street_width = 0.1*(1/scale);
        
        for (unsigned current_street = 0; current_street < number_of_streets; ++current_street){
            if (streets[current_street].width == 3 || (streets[current_street].width == 1 && scale <= 0.15)/* || (streets[current_street].width == 1 && scale <= 0.05)*/){
                for (unsigned current_seg = 0; current_seg < streets[current_street].segments.size(); ++current_seg){
                    for (unsigned current_cp = 0; current_cp <= streets[current_street].segments[current_seg].curvePointCount; ++current_cp){
                        if (current_cp == 0){
                            street_x1 = LatLonToX (streets[current_street].segments[current_seg].from, avg_lat);
                            street_y1 = LatLonToY (streets[current_street].segments[current_seg].from);
                        }
                        else {
                            street_x1 = LatLonToX (streets[current_street].segments[current_seg].curvePointsPositions[current_cp - 1], avg_lat);
                            street_y1 = LatLonToY (streets[current_street].segments[current_seg].curvePointsPositions[current_cp - 1]); 
                        }

                        if (current_cp == streets[current_street].segments[current_seg].curvePointCount){
                            street_x2 = LatLonToX (streets[current_street].segments[current_seg].to, avg_lat);
                            street_y2 = LatLonToY (streets[current_street].segments[current_seg].to);
                        }
                        else {
                            street_x2 = LatLonToX (streets[current_street].segments[current_seg].curvePointsPositions[current_cp], avg_lat);
                            street_y2 = LatLonToY (streets[current_street].segments[current_seg].curvePointsPositions[current_cp]); 
                        }
                        if (streets[current_street].segments[current_seg].highlight == true){
                            setlinewidth (big_street_width);
                            //std::cout << streets[current_street].segments[current_seg].ID << std::endl;
                            setcolor(BLUE);
                        }
                        else{
                            if (streets[current_street].width == 3){
                                setcolor(ORANGE);
                                setlinewidth (big_street_width);  
                            }
                            else{
                                setcolor(210, 210, 210, 255);
                                setlinewidth (small_street_width); 
                            } 
                        }
                        drawline(street_x1, street_y1, street_x2, street_y2);                      
                    }

                }
            }
        }

                    
        for (unsigned current_street = 0; current_street < number_of_streets; ++current_street){
            
            if (streets[current_street].width == 3 || (streets[current_street].width == 1 && scale <= 0.15)/* || (streets[current_street].width == 1 && scale <= 0.05)*/){
                for (unsigned current_seg = 0; current_seg < streets[current_street].segments.size(); ++current_seg){
                    for (unsigned current_cp = 0; current_cp <= streets[current_street].segments[current_seg].curvePointCount; ++current_cp){
                        if (current_cp == 0){
                            street_x1 = LatLonToX (streets[current_street].segments[current_seg].from, avg_lat);
                            street_y1 = LatLonToY (streets[current_street].segments[current_seg].from);
                        }
                        else {
                            street_x1 = LatLonToX (streets[current_street].segments[current_seg].curvePointsPositions[current_cp - 1], avg_lat);
                            street_y1 = LatLonToY (streets[current_street].segments[current_seg].curvePointsPositions[current_cp - 1]); 
                        }

                        if (current_cp == streets[current_street].segments[current_seg].curvePointCount){
                            street_x2 = LatLonToX (streets[current_street].segments[current_seg].to, avg_lat);
                            street_y2 = LatLonToY (streets[current_street].segments[current_seg].to);
                        }
                        else {
                            street_x2 = LatLonToX (streets[current_street].segments[current_seg].curvePointsPositions[current_cp], avg_lat);
                            street_y2 = LatLonToY (streets[current_street].segments[current_seg].curvePointsPositions[current_cp]); 
                        }
                        if (streets[current_street].width == 3){
                            setcolor(255, 242, 175, 255);
                            setlinewidth (big_street_width);  
                        }
                        else{
                            setcolor(210, 210, 210, 255);
                            setlinewidth (small_street_width); 
                        }                      
                    }
                                                        
                    if (current_seg % 10 == 0 && getStreetName(current_street)!= "<unknown>"){
                        auto rotation = atan(  (street_y1-street_y2) / (street_x1-street_x2))*180/PI;
                        settextrotation(rotation);
                        setcolor(BLACK);
                        setfontsize(8);
                        x_tolerence = 0.00003;
                        y_tolerence = 0.00003;
                        drawtext((street_x1+street_x2)/2, (street_y1+street_y2)/2, getStreetName(current_street), x_tolerence, y_tolerence);                        
                    }
                    
                     if (current_seg % 3 == 0 && getStreetSegmentInfo(current_seg).oneWay == true ){
                        auto rotation = atan(  (street_y1-street_y2) / (street_x1-street_x2))*180/PI;
                        settextrotation(rotation);
                        setcolor(BLACK);
                        setfontsize(8);
                        x_tolerence = 0.000007;
                        y_tolerence = 0.000007;
                        drawtext((street_x1+street_x2)/2, (street_y1+street_y2)/2, ">>", x_tolerence, y_tolerence);                        
                    }                  
                    setcolor(140, 140, 140, 255);
                }
            }
        }
    }
    
//DRAWING INTERSECTIONS
 
    if (draw_intersections && scale < 0.02 ){
        for (unsigned current_intersection = 0; current_intersection < intersections.size(); ++current_intersection){
            
            intersection_x = LatLonToX (intersections[current_intersection].position, avg_lat);
            intersection_y = LatLonToY (intersections[current_intersection].position);
            fillarc (intersection_x, intersection_y, intersection_radius, 0, 360);
            double  temp_x = LatLonToX (intersections[current_intersection].position,avg_lat);
            double  temp_y = LatLonToY (intersections[current_intersection].position);
            drawtext (temp_x, temp_y + 2*ratio, std::to_string(current_intersection));    
        }
    }
    
   // Draw highlighted points
    setcolor(RED);
    for (unsigned current_intersection = 0; current_intersection < highlighted_points.size(); ++current_intersection){
        intersection_x = LatLonToX (getIntersectionPosition(highlighted_points[current_intersection]), avg_lat);
        intersection_y = LatLonToY (getIntersectionPosition(highlighted_points[current_intersection]));
        fillarc (intersection_x, intersection_y, 0.000010, 0, 360);          
    }
    
// Draw POI    
    if (draw_POI){
        unsigned number_of_POI = getNumberOfPointsOfInterest();        
        if (scale <= 0.08){
            setcolor(255,255,0,255);
            for (unsigned current_POI = 0; current_POI < number_of_POI; ++current_POI){
                            
                bool inbound = true;
                visible_world = get_visible_world();
                
                if ( visible_world.bottom() > POIs[current_POI].position.y ){inbound = false;};
                if ( visible_world.top() < POIs[current_POI].position.y )  {inbound = false;};
                if ( visible_world.left() > POIs[current_POI].position.x ) {inbound = false;};
                if ( visible_world.right() < POIs[current_POI].position.x ){inbound = false;};               
                
                double offset_x = 16;
                double offset_y = 44;
                t_point scrn_point = world_to_scrn( POIs[current_POI].position );
                t_point corrected_point_on_scrn (scrn_point.x - offset_x, scrn_point.y - offset_y);
                        
                t_point point_to_display = scrn_to_world(corrected_point_on_scrn);
                
                
                if ( (POIs[current_POI].type == "restaurant" || POIs[current_POI].type == "fast_food" )  
                        && (inbound == true && scale < 0.02) ){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/bank.png"), point_to_display.x,point_to_display.y);
                } 
                
                else if ( POIs[current_POI].type == "bank"  && inbound == true && scale < 0.02){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/bank.png"), point_to_display.x,point_to_display.y);
                } 
                
                else if ( POIs[current_POI].type == "hospital"  && inbound == true && scale < 0.02){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/hospital.png"),  point_to_display.x,point_to_display.y);
                } 
                
                else if ( POIs[current_POI].type == "cafe"  && inbound == true && scale < 0.02){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/cafe.png"),  point_to_display.x,point_to_display.y);
                } 
                
                else if ( POIs[current_POI].type == "school"  && inbound == true && scale < 0.02){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/school.png"), point_to_display.x,point_to_display.y);
                } 
                
                else if ( POIs[current_POI].type == "pub"  && inbound == true && scale < 0.02){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/pub.png"), point_to_display.x,point_to_display.y);
                } 
                
                else if ( POIs[current_POI].type == "community_center"  && inbound == true && scale < 0.02){    
                    draw_surface(load_png_from_file("/nfs/ug/homes-1/h/heyinxin/ece297/work/mapper/libstreetmap/src/community_center.png"), point_to_display.x,point_to_display.y);
                } 
                
                else {fillarc (POIs[current_POI].position.x,POIs[current_POI].position.y, POI_radius, 0, 360);}
            }
        }
        
     if (draw_POI_name == true ){
            if (scale <= 0.01){
                setcolor(0,0,0,255);
                for (unsigned current_POI = 0; current_POI < number_of_POI; ++current_POI){
                    double temp_x = LatLonToX (getPointOfInterestPosition(current_POI),avg_lat);
                    double temp_y = LatLonToY (getPointOfInterestPosition(current_POI));
                    drawtext (temp_x, temp_y + 2*ratio, POIs[current_POI].name);    
                }
            }
        }    
    }
    
    copy_off_screen_buffer_to_screen();
    
//    draw_time = (std::clock()-start)/(double CLOCKS_PER_SEC);
//    std::cout << "Draw Complete! " << draw_time*1000 << " ms" << std::endl; 
}


void draw_map(){
    
    // Populating all map elements to find the display area
    
    avg_lat = 0.5 * (max_lat + min_lat);  
    double max_lat = getIntersectionPosition(0).lat();
    double min_lat = max_lat;
    double max_lon = getIntersectionPosition(0).lon();
    double min_lon = max_lon;

    intersections.resize(getNumberOfIntersections());
    
    for (unsigned current_intersection = 0; current_intersection < intersections.size(); ++current_intersection)
    {
        intersections[current_intersection].position = getIntersectionPosition(current_intersection);
        intersections[current_intersection].name = getIntersectionName(current_intersection);
    
        max_lat = std::max(max_lat, intersections[current_intersection].position.lat());
        min_lat = std::min(min_lat, intersections[current_intersection].position.lat());
        max_lon = std::max(max_lon, intersections[current_intersection].position.lon());
        min_lon = std::min(min_lon, intersections[current_intersection].position.lon());
    
    }
    
    max_width = max_lon * std::cos(avg_lat) * DEG_TO_RAD - min_lon * std::cos(avg_lat) * DEG_TO_RAD;

//POPULATING SEGMENTS    
    StreetSegmentInfo oldSeg;
    segments.resize(getNumberOfStreetSegments());
    for (unsigned current_seg = 0; current_seg < segments.size(); ++current_seg){
        segments[current_seg].ID = current_seg;
        oldSeg = getStreetSegmentInfo(current_seg);
        segments[current_seg].wayOSMID = oldSeg.wayOSMID;
        segments[current_seg].from = getIntersectionPosition(oldSeg.from);
        segments[current_seg].to = getIntersectionPosition(oldSeg.to);
        segments[current_seg].oneWay = oldSeg.oneWay;
        segments[current_seg].curvePointCount = oldSeg.curvePointCount;
        for (unsigned current_cp = 0; current_cp < oldSeg.curvePointCount; ++current_cp){
            segments[current_seg].curvePointsPositions.push_back(
                getStreetSegmentCurvePoint(current_seg, current_cp));
        }

        segments[current_seg].speedLimit = oldSeg.speedLimit;
        segments[current_seg].streetID = oldSeg.streetID;
    }
    
//POPULATING STREETS
    streets.resize(getNumberOfStreets());
    std::vector<unsigned> all_segments_in_street;
    
    unsigned number_of_segments;
    
    for (unsigned current_street = 0; current_street < streets.size(); ++current_street){
              
        all_segments_in_street = find_street_street_segments(current_street);
        number_of_segments = all_segments_in_street.size();
        streets[current_street].segments.resize(number_of_segments);
        
        for (unsigned current_seg = 0; current_seg < number_of_segments; ++current_seg){
            streets[current_street].segments[current_seg] = segments[all_segments_in_street[current_seg]];            
            streets[current_street].avgSpeed += oldSeg.speedLimit;
        }
        streets[current_street].name = getStreetName(current_street);
        streets[current_street].length = find_street_length(current_street);
        streets[current_street].avgSpeed = streets[current_street].avgSpeed / number_of_segments;
        streets[current_street].intersectionCount = find_all_street_intersections(current_street).size();

        if ((streets[current_street].intersectionCount >= 50 && streets[current_street].intersectionCount <= 500 && 
                streets[current_street].length >= 1000 && streets[current_street].length <= 50000) 
                || streets[current_street].avgSpeed >= 79){
            streets[current_street].width = 3;
        }
        else {
            streets[current_street].width = 1;
        }        
    }

//POPULATING FEATURES

    features.resize(getNumberOfFeatures());
    unsigned pointCount;
    LatLon first;
    LatLon last;
    LatLon temp;
    double dx;
    double dy;
    for (unsigned current_feature = 0; current_feature < features.size(); ++current_feature){
        features[current_feature].name = getFeatureName(current_feature);
        features[current_feature].type = getFeatureType(current_feature);
        features[current_feature].OSMID = getFeatureOSMID(current_feature);
        pointCount = getFeaturePointCount(current_feature);
        features[current_feature].featurePointCount = pointCount;
        
        first = getFeaturePoint(current_feature, 0);
        last = getFeaturePoint(current_feature, pointCount - 1);
        
        //Closed feature
        if (first.lat() == last.lat() && first.lon() == last.lon()){
            features[current_feature].closed = true;
            
            for (unsigned current_point = 0; current_point < pointCount; ++current_point){
                temp = getFeaturePoint (current_feature, current_point);
                features[current_feature].featurePointsXY.push_back(t_point(LatLonToX(temp,avg_lat),LatLonToY(temp)));
            }
            
            features[current_feature].area = computePolygonArea(pointCount, features[current_feature].featurePointsXY.data());
            if (features[current_feature].area < 0){
                features[current_feature].area *= -1;
            }
        }       
        //Open feature
        else {
            features[current_feature].closed = false;
            
            features[current_feature].length = 0;
            for (unsigned current_point = 0; current_point < pointCount; ++current_point){
                temp = getFeaturePoint (current_feature, current_point);
                features[current_feature].featurePointsXY.push_back(t_point(LatLonToX(temp,avg_lat),LatLonToY(temp)));
                if (current_point != pointCount - 1){
                    dx = features[current_feature].featurePointsXY[current_point].x - LatLonToX(getFeaturePoint(current_feature, current_point + 1),avg_lat);
                    dy = features[current_feature].featurePointsXY[current_point].y - LatLonToY(getFeaturePoint(current_feature, current_point + 1));
                    features[current_feature].length = sqrt(dx * dx + dy * dy);
                }
            }
        }       
    }
   
//POPULATING POINTS OF INTEREST
    POIs.resize(getNumberOfPointsOfInterest());
    for (unsigned each_POI = 0; each_POI < POIs.size(); ++each_POI){
        POIs[each_POI].type = getPointOfInterestType (each_POI);
        POIs[each_POI].name = getPointOfInterestName (each_POI);
        POIs[each_POI].position = t_point(LatLonToX(getPointOfInterestPosition(each_POI),avg_lat), LatLonToY(getPointOfInterestPosition(each_POI)));
        POIs[each_POI].nodeOSMID = getPointOfInterestOSMNodeID (each_POI);
    }

// Initialize graphic window (based on the max lat, lon found) and create bottoms
// set event loops to react on different user input    
    init_graphics ("Street Map", WHITE);
    create_button("Zoom Fit", "Find", find_and_highlight );
    create_button("Find","Hide POI", show_POI );
    create_button("Hide POI","Hide POIN" , show_POI_name );
    create_button("Hide POIN", "Directions", in_directions_mode);
    create_button("Directions", "Help", give_help);
    create_button("Help","Clear",clear);
    
    set_visible_world (min_lon * std::cos(avg_lat) * DEG_TO_RAD, min_lat * DEG_TO_RAD,
                       max_lon * std::cos(avg_lat) * DEG_TO_RAD, max_lat * DEG_TO_RAD);
    //enable keypress
   set_keypress_input(true);
 
   event_loop(act_on_button_press, act_on_mouse_move, act_on_keypress, draw_screen);
   
  std::string errorMessage;
//  drawErrorMessages(errorMessage);
   close_graphics();
   
}

// find function that highlights map elements
// also show the number of results found
void find_and_highlight(void (*draw_screen) (void)){
   
   char command = 'I'; 
   std :: cout << "Search for Streets, Points of Interest and Intersections\n" ;
   std :: cout << "Enter S/P/I to search:" ;
   std :: cin >> command;
   
   if (command == 'I'){
       
    std::cin.ignore(256,'\n');
    std::string first_street;
    std::string second_street;
    std::cout << "Enter street name to search for intersection:" << std::endl;
    std::cout << "Enter the first Street:" ;
    std::getline(std::cin,first_street);
    std::cout << "Enter the second Street:" ;
    std::getline(std::cin,second_street);


    std::vector<unsigned> intersection_ids = 
        find_intersection_ids_from_street_names(first_street,second_street);

    if (intersection_ids.size() == 0){
        std::cout << "No intersection is Found" << std::endl;
    }
    else{
        float const intersection_width = 0.0000075;
        float const intersection_height = 0.0000075;

        double intersection_x;
        double intersection_y;

        std :: cout << "Search Complete! " << intersection_ids.size() << " results found" << std::endl;;

         for (unsigned current_intersection = 0; current_intersection <intersection_ids.size(); ++current_intersection){

            LatLon currentLatLon = getIntersectionPosition(intersection_ids[current_intersection]);
            intersection_x = LatLonToX (currentLatLon, avg_lat);
            intersection_y = LatLonToY (currentLatLon);
            std::cout << getIntersectionName[current_intersection]; 
            std::cout << " LatLon (" << currentLatLon.lat() << "," << currentLatLon.lon() << ")" << std::endl;

            setcolor(RED);
            fillrect (intersection_x - (intersection_width/2), intersection_y - (intersection_height/2), 
                    intersection_x + (intersection_width/2), intersection_y + (intersection_height/2)); 
            }     
        }
            setcolor(BLACK);
    }
   
   // search for Point of Interests
   else if (command == 'P'){
       std::cin.ignore(256,'\n');
       std::string POI_to_search;
       std::cout << "Enter name of POI to search:" ;
       std::getline(std::cin,POI_to_search);
       
       int numPOI = 0;
       
        unsigned number_of_POI = getNumberOfPointsOfInterest();
        double POI_x;
        double POI_y;
        unsigned const POI_relative_radius = 0.7;
        double POI_radius = POI_relative_radius * ratio;
    
        setcolor(RED);
        for (unsigned current_POI = 0; current_POI < number_of_POI; ++current_POI){
            if (getPointOfInterestName(current_POI) == POI_to_search ){
                POI_x = POIs[current_POI].position.x;
                POI_y = POIs[current_POI].position.y;
                fillarc (POI_x, POI_y, POI_radius, 0, 360);
                numPOI = numPOI + 1;
            }
        }
        std::cout << "Search Complete ! " << numPOI << " result(s) found." << std::endl;

        if (numPOI != 0 ){
            std:: cout << "\n" << "Would you like to go to the closest " << POI_to_search << " (Y/N) ? \n";
            
            char user_input;
            std::cin >> user_input;
            unsigned previous_click = intersection_to_highlight;                      
            if (user_input == 'Y'){

                highlighted_segments = find_path_to_point_of_interest(intersection_to_highlight,POI_to_search,0.0);
                give_directions (highlighted_segments);
            }    
        }        
                
        setcolor(BLACK);
   }
   
   // search for Streets
   else if (command == 'S'){
       std::cin.ignore(256,'\n');
       std :: string street_to_find;
       std :: cout << "Enter name of street to search:" ;
       std :: getline (std::cin, street_to_find);
       
       setlinewidth (3);
       setcolor(RED);
    
        double street_x1 = 0.0;
        double street_y1 = 0.0;
        double street_x2 = 0.0;
        double street_y2 = 0.0;

        std :: vector<unsigned> street_found = find_street_ids_from_name(street_to_find);  
        
        for (unsigned current_street = 0; current_street < street_found.size(); ++current_street){
            
            for (unsigned current_seg = 0; current_seg < streets[street_found[current_street]].segments.size(); ++current_seg){
                for (unsigned current_cp = 0; current_cp <= streets[street_found[current_street]].segments[current_seg].curvePointCount; ++current_cp){
                if (current_cp == 0){
                    street_x1 = LatLonToX (streets[street_found[current_street]].segments[current_seg].from, avg_lat);
                    street_y1 = LatLonToY (streets[street_found[current_street]].segments[current_seg].from);
                }
                else {
                    street_x1 = LatLonToX (streets[street_found[current_street]].segments[current_seg].curvePointsPositions[current_cp - 1], avg_lat);
                    street_y1 = LatLonToY (streets[street_found[current_street]].segments[current_seg].curvePointsPositions[current_cp - 1]); 
                }

                if (current_cp == streets[street_found[current_street]].segments[current_seg].curvePointCount){
                    street_x2 = LatLonToX (streets[street_found[current_street]].segments[current_seg].to, avg_lat);
                    street_y2 = LatLonToY (streets[street_found[current_street]].segments[current_seg].to);
                }
                else {
                    street_x2 = LatLonToX (streets[street_found[current_street]].segments[current_seg].curvePointsPositions[current_cp], avg_lat);
                    street_y2 = LatLonToY (streets[street_found[current_street]].segments[current_seg].curvePointsPositions[current_cp]); 
                }              
                drawline(street_x1, street_y1, street_x2, street_y2); 
            }
            }    
            
        }
        setcolor(BLACK);
        std :: cout << "Search Complete! " << street_found.size() << " results found" << std::endl;
   }    
    
   
   else{ std::cout << "Invalid commend ! \n";  }  
    copy_off_screen_buffer_to_screen();
}

// determine if a point in x,y is inside the current display
bool pointInsideScreen (t_point& currentPoint) {
    t_bound_box currentWorld = get_visible_world();
    
    return (    (currentPoint.x < currentWorld.right()) &&
                (currentPoint.x > currentWorld.left()) &&
                (currentPoint.y < currentWorld.top()) &&
                (currentPoint.y > currentWorld.bottom()));
}


//void displayPath (std:: vector <StreetSegmentIndex> streetSegments){
//    bool drawDirectionCurve = true;
//    setlinewidth(12);
//    
//    for (unsigned it = 0; it < streetSegments.size(); it++){
//        if(!it)
//            setcolor(255,224,255);
//        else
//            setcolor(0,20,255,100);
//        
//        unsigned streetSegmentIndex = streetSegments[it];
//        StreetSegmentInfo information = getStreetSegmentInfo(streetSegmentIndex);
//        LatLon from = getIntersectionPosition(information.from);
//        for (unsigned j = 0; j<information.curvePointCount; j++)
//        {
//            LatLon to = getStreetSegmentCurvePoint(streetSegmentIndex, j);
//            auto const xfrom = from.lon() * cos(avg_lat) * DEG_TO_RAD;
//            auto const yfrom = from.lat() * DEG_TO_RAD;
//            auto const xto = to.lon() * cos(avg_lat)* DEG_TO_RAD;
//            auto const yto = to.lat() * DEG_TO_RAD;
//            t_point center_curve ((xfrom + xto) / 2.0, (yfrom + yto) / 2.0);
//            drawDirectionCurve = pointInsideScreen(center_curve);
//            
//            if (drawDirectionCurve )
//                drawline(xfrom,yfrom,xto,yto);
//                from = to;
//            }
//            LatLon to = getIntersectionPosition(information.to);
//            auto const xfrom = from.lon() * cos(avg_lat) * DEG_TO_RAD;
//            auto const yfrom = from.lat() * DEG_TO_RAD;
//            auto const xto = to.lon() * cos(avg_lat)* DEG_TO_RAD;
//            auto const yto = to.lat() * DEG_TO_RAD;
//            
//            drawline(xfrom,yfrom, xto, yto);
//        }
//}
    
// get user's mouse location in x,y
void act_on_mouse_move(float x, float y) {
}

// function to handle keyboard press event, the ASCII character is returned
// along with an extended code (keysym) on X11 to represent non-ASCII
// characters like the arrow keys.
void act_on_keypress(char c, int keysym) {

    #ifdef X11 // Extended keyboard codes only supported for X11 for now
    switch (keysym) {
        case XK_Left:
            translate_left(draw_screen);
            break;
        case XK_Right:
            translate_right(draw_screen);
            break;
        case XK_Up:
            translate_up(draw_screen);
            break;
        case XK_Down:
            translate_down(draw_screen);
            break;
        case XK_Page_Down:
            zoom_out(draw_screen);
            break;
        case XK_Page_Up:
            zoom_in(draw_screen);
            break;
        case XK_Escape:
            quit(draw_screen);
            break;
        case XK_p:
            proceed(draw_screen);
            break;
        case XK_Home:
            zoom_fit(draw_screen);
            break;
        case XK_f:
            find_and_highlight(draw_screen);
            break;
        default:
            std::cout << "Undefined keypress" << std::endl;
            break;
    }
    #endif
}

// act on mouse press
// convert the click location in x,y into map location Latlon
// then find the nearest intersection and highlight(redraw in Red)
void act_on_button_press(float x, float y, t_event_buttonPressed event){
    
    double  Lon_clicked = x/(DEG_TO_RAD * cos (avg_lat));
    double  Lat_clicked = y/(DEG_TO_RAD);
    LatLon LatLon_clicked(Lat_clicked,Lon_clicked);
    
    intersection_to_highlight = find_closest_intersection(LatLon_clicked);
    
    POI_x = LatLonToX (getIntersectionPosition(intersection_to_highlight),avg_lat);
    POI_y = LatLonToY (getIntersectionPosition(intersection_to_highlight));
        
    if (!in_directions && !in_find_POI){

        std::cout << intersection_to_highlight << ": ";
        std::cout << getIntersectionName(intersection_to_highlight) << " at ";
        std::cout << "(" << getIntersectionPosition(intersection_to_highlight).lat() << ",";
        std::cout << getIntersectionPosition(intersection_to_highlight).lon() << ")" << std::endl;

        setcolor(RED);

        if (scale > 0.03){
            fillarc (POI_x, POI_y, 0.0001*scale, 0, 360);
        }
        else{
            fillarc (POI_x, POI_y, intersection_radius, 0, 360);
        }
        setcolor(204,204,0,255);
    }
    else if (in_directions){
        
        click_num ++;
        if (click_num == 1){
            from_ID = intersection_to_highlight;
            clear_highlights();
            std::cout << getIntersectionName(intersection_to_highlight) << " to ";
        }
        else if (click_num == 2){
            to_ID = intersection_to_highlight;
            std::cout << getIntersectionName(intersection_to_highlight) << " ";
            highlighted_segments.clear();
            highlighted_segments = find_path_between_intersections(from_ID,to_ID,0);
            give_directions (highlighted_segments);
            click_num = 0;
            in_directions = false;
        }
        
        setcolor(RED);
        if (scale > 0.03){
            fillarc (POI_x, POI_y, 0.0001*scale, 0, 360);
        }
        else{
            fillarc (POI_x, POI_y, intersection_radius, 0, 360);
        }
        setcolor(204,204,0,255);
    }
    copy_off_screen_buffer_to_screen();
}

// Show/Hide POI by change the flag
// Also change the button name to show the current display status 
void show_POI(void (*draw_screen) (void)){
    char old_button_name[200], new_button_name[200];
     
    if (draw_POI == true){
        strcpy (old_button_name, "Show POI");
        strcpy (new_button_name, "Hide POI");   
        change_button_text(old_button_name, new_button_name);
        draw_POI = false;
        draw_screen();
    }
    
    else if (draw_POI == false){
        strcpy (old_button_name, "Hide POI");
        strcpy (new_button_name, "Show POI");
        change_button_text(old_button_name, new_button_name);
        draw_POI = true;
        draw_screen();
    }
}

// Show/Hide POI_name by change the flag
// Also change the button name to show the current display status 
void show_POI_name(void (*draw_screen) (void)){
    char old_button_name[200], new_button_name[200];
    
    if (draw_POI_name == true){
        strcpy (old_button_name, "Show POIN");
        strcpy (new_button_name, "Hide POIN");   
        change_button_text(old_button_name, new_button_name);
        draw_POI_name = false;
        draw_screen();
    }
    
    else if (draw_POI_name == false){
        strcpy (old_button_name, "Hide POIN");
        strcpy (new_button_name, "Show POIN");
        change_button_text(old_button_name, new_button_name);
        draw_POI_name = true;
        draw_screen();
    } 
}    

void in_directions_mode (void (*draw_screen) (void)){
    std::cout << "You would like to get from ";
    in_directions = true;
    draw_screen();
}

void clear_highlights(){
    unsigned number_of_streets = getNumberOfStreets();
    for (unsigned current_street = 0; current_street < number_of_streets; ++current_street){
        for (unsigned current_seg = 0; current_seg < streets[current_street].segments.size(); ++current_seg){
            streets[current_street].segments[current_seg].highlight = false;
        }
    }
}

void highlight_segments (std::vector<unsigned> segments){
    unsigned streetID;
    for (unsigned current_seg = 0; current_seg < segments.size(); current_seg++){
        streetID = getStreetSegmentInfo(segments[current_seg]).streetID;
        for (unsigned current_seg_in_street = 0; current_seg_in_street < streets[streetID].segments.size(); current_seg_in_street ++){
            if (streets[streetID].segments[current_seg_in_street].ID == segments[current_seg]){
                streets[streetID].segments[current_seg_in_street].highlight = true;
            }
        }
    }
}

void give_directions (const std::vector<unsigned>& segments_to_direct){
    std::cout << std::endl << "Here are your directions:" << std::endl;
    LatLon from1;
    LatLon to1;
    LatLon from2;
    LatLon to2;
    double x1;
    double y1;
    double x2;
    double y2;
    double x3;
    double y3;
    double dx;
    double dy;
    double angle;
    double distance = 0;
    std::pair<double, double> vector1;
    std::pair<double, double> vector2;
    each_segment_data first_seg;
    each_segment_data second_seg;
        
    
    for (unsigned each_segment = 1; each_segment < segments_to_direct.size(); each_segment++){
        first_seg = segments[segments_to_direct[each_segment - 1]];
        second_seg = segments[segments_to_direct[each_segment]];
        
        if (first_seg.to.lat() == second_seg.from.lat() &&
            first_seg.to.lon() == second_seg.from.lon()){
            if (first_seg.curvePointCount != 0){
                from1 = first_seg.curvePointsPositions[first_seg.curvePointCount - 1];
            }
            else {
                from1 = first_seg.from;
            }
            if (second_seg.curvePointCount != 0){
                to2 = second_seg.curvePointsPositions[0];
            }
            else {
                to2 = second_seg.to;
            }
            x2 = LatLonToX(first_seg.to, avg_lat);
            y2 = LatLonToY(first_seg.to);
        }
        else if (first_seg.from.lat() == second_seg.from.lat() &&
                 first_seg.from.lon() == second_seg.from.lon()){
            if (first_seg.curvePointCount != 0){
                from1 = first_seg.curvePointsPositions[0];
            }
            else {
                from1 = first_seg.to;
            }
            if (second_seg.curvePointCount != 0){
                to2 = second_seg.curvePointsPositions[0];
            }
            else {
                to2 = second_seg.to;
            }
            x2 = LatLonToX(first_seg.from, avg_lat);
            y2 = LatLonToY(first_seg.from);
        }
        else if (first_seg.to.lat() == second_seg.to.lat() &&
                 first_seg.to.lon() == second_seg.to.lon()){
            if (first_seg.curvePointCount != 0){
                from1 = first_seg.curvePointsPositions[first_seg.curvePointCount - 1];
            }
            else {
                from1 = first_seg.from;
            }
            if (second_seg.curvePointCount != 0){
                to2 = second_seg.curvePointsPositions[second_seg.curvePointCount - 1];
            }
            else {
                to2 = second_seg.from;
            }
            x2 = LatLonToX(first_seg.to, avg_lat);
            y2 = LatLonToY(first_seg.to);
        }
        else if (first_seg.from.lat() == second_seg.to.lat() &&
                 first_seg.from.lon() == second_seg.to.lon()){
            if (first_seg.curvePointCount != 0){
                from1 = first_seg.curvePointsPositions[0];
            }
            else {
                from1 = first_seg.to;
            }
            if (second_seg.curvePointCount != 0){
                to2 = second_seg.curvePointsPositions[second_seg.curvePointCount - 1];
            }
            else {
                to2 = second_seg.from;
            }
            x2 = LatLonToX(first_seg.from, avg_lat);
            y2 = LatLonToY(first_seg.from);
          
        }
        else {
            std::cout << "YOU CAN'T GO TO YOUR DESTINATION! Try taking a boat or an airplane :)" << std::endl;
            x2 = -1;
            y2 = -1;
        }
        
        x1 = LatLonToX(from1, avg_lat);
        y1 = LatLonToY(from1);
        x3 = LatLonToX(to2, avg_lat);
        y3 = LatLonToY(to2);  
              
        dx = x1 - x2;
        dy = y1 - y2;
        vector1 = std::make_pair(dx, dy);
        dx = x2 - x3;
        dy = y2 - y3;
        vector2 = std::make_pair(dx, dy);
        
        
        angle = (vector1.first * vector2.first + vector1.second * vector2.second) / 
                (sqrt(vector1.first * vector1.first + vector1.second * vector1.second) * sqrt(vector2.first * vector2.first + vector2.second * vector2.second));
        
//        std::cout << "angle: " << angle << std::endl;
        angle = acos(angle) * 180 / 3.14159265359;
                
        distance += find_street_segment_length(segments_to_direct[each_segment - 1]);
        if (first_seg.streetID == second_seg.streetID){
        }
        else if (first_seg.streetID != second_seg.streetID){
            std::cout << "Travel " << distance << " meters down " << getStreetName(first_seg.streetID) << "." << std::endl;
            if (angle >= 110){
                std::cout << "Turn right on " << getStreetName(second_seg.streetID) << "" << std::endl;
            }
            else if (angle <= 90){
                std::cout << "Turn left on " << getStreetName(second_seg.streetID) << "" << std::endl;
            }
            else {
                std::cout << "Continue on " << getStreetName(second_seg.streetID) << "" << std::endl;
            }
            distance = 0;
        }
        if (each_segment == segments_to_direct.size() - 1){
            distance = find_street_segment_length(segments_to_direct[each_segment - 1]);
            std::cout << "Travel " << distance << " meters down " << getStreetName(second_seg.streetID) << " to reach your destination." << std::endl << std::endl;
        }
    }
}

void clear(void (*draw_screen) (void)){
    clear_highlights();

}

void give_help(void (*draw_screen) (void)){
    std::cout << std::endl;
    std::cout << "Welcome to MELON GIS 3.0! (now with direction finding functionalities!)" << std::endl << std::endl;
    std::cout << "Use the mouse scroll wheel, or the Zoom In Zoom Out buttons to zoom in and out." << std::endl << std::endl;
    std::cout << "Press F, or press the Find button to search for a street, intersection, of point of interest." << std::endl << std::endl;
    std::cout << "You may choose to hide point of interest names and icons by pressing Hide POI and Hide POIN respectively." << std::endl << std::endl;
    std::cout << "You may find the route between two intersections by pressing the Directions button, then clicking two intersections." << std::endl << std::endl;

}