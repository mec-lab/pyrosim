#ifndef _ENVIRONMENT_HPP
#define _ENVIRONMENT_HPP

#pragma once

// standard
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <utility>

// ode
#include <ode/ode.h>

// local
#include "entity.hpp"

class Environment{
public:
    dWorldID world;
    dSpaceID topspace;
    // useful to split this into seperate maps
    std::vector<Entity*> entities; // general entities (bodies, joints, etc)

    // ENTITY, ACTUATOR, JOINT, NEURON, SYNAPSE, BODY, SENSOR
    std::vector< std::vector<int> > entityVectors;

    std::map<std::string, dSpaceID> subspaces;
    std::vector< std::pair <int, int> > collisions;

    Environment(dWorldID world, dSpaceID topspace, int numEntities = 50);
    ~Environment();

    /**
        Adds an entity by reading in from python
    */
    void addToEntityFromPython(void);

    /**
        Adds a collision pair between groups to the `collisions` map

        @param firstID  - the id of the first group
        @param secondID - the id of the second collision group
    */
    void addCollisionPair(int firstID, int secondID);

    // Clears all collision pairs
    void emptyCollisionPairs(void);
    
    // Reads an entities contents from python
    void readEntityFromPython(void);

    // create entities in simualiton
    void createInODE(void);

    /**
        Draws entities to graphics window

        @param drawJoints - If true, visualize joints
        @param drawSpaces - If true, draw spaces as boxes
    */
    void draw(int drawJoints, int drawSpaces);

    // Return ODE world variable
    dWorldID getWorld(){return this->world;};

    /**
        Gets the named space from the ODE world
        
        @param name - the string literal referring to the space
        @return The space
    */
    dSpaceID getSpace(std::string name);

    /**
        Create a space and put it in the space map

        @param name - the string literal to refer to the space as
    */
    void createSpace(std::string name);

    // Returns entity with id i
    Entity* getEntity(int i);

    /**
        Take environmental step

        @param timeStep      - the current time step
        @param dt            - the space between steps
        @param updateNetwork - whether or not to update network during step
    */      
    void takeStep(int timeStep, dReal dt, int updateNetwork);

    /**
        Take a step with the specified entities

        @param entityIDs - the entities to step with
        @param timeStep  - the current time step
        @param dt        - the space between steps
    */
    void takeStepWithEntities(std::vector<int> entityIDs, int timeStep, dReal dt);
    
    // write entities to python
    void writeToPython(void);

};

#endif