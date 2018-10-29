/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   preCompData.h
 * Author: dinggaox
 *
 * Created on March 31, 2018, 2:54 PM
 */


#pragma once

class preCompData {
public:
    double travelTime;
    std::string firstStreet;
    std::string lastStreet;
    preCompData (double tt, std::string& fn, std::string& ln) {travelTime = tt; firstStreet = fn; lastStreet = ln;}
    
    double getTravelTime(){return travelTime;}
};