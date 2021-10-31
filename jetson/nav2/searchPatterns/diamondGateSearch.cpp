#include "diamondGateSearch.hpp"

void generateDiamondGateSearch(){

    Odometry currOdom = gRover->roverStatus().odometry();
    double diamondWidth = 2 * 1.5; // 2 meter gate width * 1.5 multiplier
    const double targetBearing = gRover->roverStatus().target().bearing;
    const double targetDist = gRover->roverStatus().target().distance;

    // TODO: make figure explaining this in drive and link here
    double distance = sqrt(pow(targetDist, 2) + pow(diamondWidth, 2));
    double theta = atan2(diamondWidth, targetDist) * 180 / PI;
    double relTurn = theta + targetBearing;
    double angle = mod(currOdom.bearing_deg + relTurn, 360); // absolute bearing

    Odometry corner1 = createOdom(currOdom, angle, distance, gRover);

    const double absolute_bear_to_target = mod(currOdom.bearing_deg + targetBearing, 360);
    Odometry corner2 = createOdom(currOdom, absolute_bear_to_target, diamondWidth + targetDist, gRover);

    relTurn = -1 * theta + targetBearing;
    angle = mod(currOdom.bearing_deg + relTurn, 360);
    Odometry corner3 = createOdom(currOdom, angle, distance, gRover);

    Odometry corner4 = createOdom(currOdom, mod(absolute_bear_to_target + 180, 360), diamondWidth - targetDist, gRover);


    Waypoint waypoint1;
    Waypoint waypoint2;
    Waypoint waypoint3;
    Waypoint waypoint4;

    //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
    waypoint1.odom = corner1;
    waypoint2.odom = corner2;
    waypoint3.odom = corner3;
    waypoint4.odom = corner4;

    waypoint1.type = "gateTraversal";
    waypoint2.type = "gateTraversal";
    waypoint3.type = "gateTraversal";
    waypoint4.type = "gateTraversal";

    gRover->roverStatus().course().pushBack(waypoint1);
    gRover->roverStatus().course().pushBack(waypoint2);
    gRover->roverStatus().course().pushBack(waypoint3);
    gRover->roverStatus().course().pushBack(waypoint4);

}
