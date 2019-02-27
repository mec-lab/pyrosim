#ifndef __SENSOR__HPP
#define __SENSOR__HPP

#pragma once

#include <vector>

#include "entity.hpp"

class Sensor : public Entity
{
protected:
    float currentSensorValue;
    std::vector<float> sensorValues;
    bool writeBack;
public:
    float getSensorValue(){ return this->currentSensorValue; }
    virtual EntityType getEntityType(void){ return SENSOR; }
    virtual void sense() =0;
    virtual void takeStep(int timeStep, dReal dt){ this->sense(); }
    virtual void writeToPython(void){
        
        // TO DO: Make write back a changeable parameter
        this->writeBack = true;
        if ( this->writeBack == true ){
            std::cout << " " << this->entityID;
            for (auto val : this->sensorValues){
                std::cout << " " << val;
            }           
        }
    };
};

#endif