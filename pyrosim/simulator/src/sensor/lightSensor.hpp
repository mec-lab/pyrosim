#ifndef __LIGHT_SENSOR_HPP__
#define __LIGHT_SENSOR_HPP__

#pragma once
#include "sensor/sensor.hpp"
#include "pythonReader.hpp"


class LightSensor : public Sensor
{
protected:
    RigidBody *body;
    int bodyID;
    Environment *environment;
public:
    virtual void readFromPython( void ){
        readValueFromPython<int>( &this->bodyID, "Body ID" );
    }

    virtual void create( Environment *environment ){
        this->body = (RigidBody *) environment->getEntity( this->bodyID );
        this->environment = environment;
    }

    void sense(){
        // get all light sources
        dReal lightSense = 0.0;

        const dReal *myPosition = this->body->getPosition();

        for (auto entity : environment->entities){
            if ( entity->getEntityType() == BODY ){
                RigidBody *otherBody = (RigidBody *) entity;
                std::cerr << "Body light intensity " << otherBody->lightIntensity << std::endl;
                if ( otherBody->lightIntensity > 0.0 ){
                    const dReal *otherPosition = otherBody->getPosition();

                    dReal distanceSquared = pow( myPosition[0] - otherPosition[0], 2.0 ) + 
                                            pow( myPosition[1] - otherPosition[1], 2.0 ) +
                                            pow( myPosition[2] - otherPosition[2], 2.0 );
                    dReal intensity = 1.0 / distanceSquared;
                    lightSense += intensity;
                }
            }
        }
        std::cerr << "Light sense " << lightSense << std::endl;

        this->currentSensorValue = lightSense;
        this->sensorValues.push_back( this->currentSensorValue );
    }
};


#endif