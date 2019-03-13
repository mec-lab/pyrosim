#ifndef __DISTANCE_TO_SENSOR_HPP__
#define __DISTANCE_TO_SENSOR_HPP__

#pragma once

#include <math.h>

#include "sensor/sensor.hpp"
#include "body/rigidBody.hpp"
#include "pythonReader.hpp"

class DistanceToSensor : public Sensor
{
protected:
    RigidBody *sourceBody;
    RigidBody *targetBody;
    int sourceBodyID;
    int targetBodyID;
public:
    virtual void readFromPython(void){
        readValueFromPython<int>(&this->sourceBodyID, "Source Body ID");
        readValueFromPython<int>(&this->targetBodyID, "Target Body ID" );
        this->readWriteBackFromPython();
    }

    virtual void create(Environment *environment){
        try {
            this->sourceBody = (RigidBody *) environment->getEntity(this->sourceBodyID);
            this->targetBody = (RigidBody *) environment->getEntity( this->targetBodyID );
        }
        catch (...){
            std::cerr << "ERROR:Cannot convert ID ";
            exit(0);
        }
    }

    void sense(){
        const dReal* pos1 = this->sourceBody->getPosition();
        const dReal* pos2 = this->targetBody->getPosition();

        dReal distance = sqrt( pow( pos1[0] - pos2[0], 2 ) +
                               pow( pos1[1] - pos2[1], 2 ) +
                               pow( pos1[2] - pos2[2], 2 ) );
        this->currentSensorValue = distance;
        this->sensorValues.push_back(this->currentSensorValue);
    }
};

#endif