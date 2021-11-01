
#include "searchPatternUtils.hpp"

void insertWaypointsIntoCourse(Waypoint p1, Waypoint p2, double visionDistance)
{
    double visionDistance = mRoverConfig[ "computerVision" ][ "visionDistance" ].GetDouble();
    const double maxDifference = 2 * visionDistance;

    double distance = estimateNoneuclid( point1, point2 );
    if ( distance > maxDifference )
    {
        int numPoints = int( ceil( distance / maxDifference ) - 1 );
        double newDifference = distance / ( numPoints + 1 );
        double bearing = calcBearing( point1, point2 );
        for ( int j = 0; j < numPoints; ++j )
        {
            Odometry startPoint = mSearchPoints.at( i );
            Odometry newOdom = createOdom( startPoint, bearing, newDifference, mRover );
            Waypoint nextWaypoint;
            //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
            nextWaypoint.odom = nextOdom;
            nextWaypoint.type = "searchPoint";
            gRover->roverStatus().course().push_back(nextWaypoint);
            
            ++i;
        }
         Waypoint nextWaypoint;
        //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
        nextWaypoint.odom = p2;
        nextWaypoint.type = "searchPoint";
        gRover->roverStatus().course().push_back(nextWaypoint);
    }
    else{
        Waypoint nextWaypoint;
        //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
        nextWaypoint.odom = p1;
        nextWaypoint.type = "searchPoint";
        gRover->roverStatus().course().push_back(nextWaypoint);

         Waypoint nextWaypoint;
        //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
        nextWaypoint.odom = p2;
        nextWaypoint.type = "searchPoint";
        gRover->roverStatus().course().push_back(nextWaypoint);

    }


   
    
} // insertIntermediatePoints()