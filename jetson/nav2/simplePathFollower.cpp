#include "simplePathFollower.hpp"

void SimplePathFollower::followPath(vector<Odometry>& path){
    //check if angle to target is greater than x degrees, if so turn to target
    //else move forwards toward target with corrective factor for bearing

    const int TURN_THRESH = 10;

    //TODO: fill these variables in with actual rover state values
    //Odometry curOdom;

    //TODO: calculate bearing to target (def exists somewhere in repo)
    double bearingToTarget = 0.0;

    //TODO: calculate dist to target (def alr exists somewhere in repo)
    //double distToTarget = 0.0;

    //double leftPower = 0.0;
    //double rightPower = 0.0;

    if (abs(bearingToTarget) > TURN_THRESH){
        //turn to target
        //TODO: implement PID for this as well (alr exists somewhere
        //double turningPower = bearingToTarget;
    }
    else{
        //move forward to target
        //TODO: implement PID or some sort of function to control forward power based on distance (also def alr exists)
        //double forwardPower = distToTarget;
        //TODO: implement PID or some sort of function to control turning correction power (def a util function or smthing alr written)
        //double turningPower = bearingToTarget;

        // leftPower = forwardPower + turningPower;
        // rightPower = forwardPower - turningPower;
    }

    //TODO: set left and right powers to the rover (call a function)
    

}