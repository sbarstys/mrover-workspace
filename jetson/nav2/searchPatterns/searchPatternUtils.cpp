
#include "searchPatternUtils.hpp"

void insertWaypointsIntoCourse(Waypoint point1, Waypoint point2, double visionDistance)
{
    const double maxDifference = 2 * visionDistance;

    double distance = estimateNoneuclid( point1.odom, point2.odom );
    if ( distance > maxDifference )
    {
        int numPoints = int( ceil( distance / maxDifference ) - 1 );
        double newDifference = distance / ( numPoints + 1 );
        double bearing = calcBearing( point1.odom, point2.odom );
        for ( int j = 0; j < numPoints; ++j )
        {
            Odometry startPoint = gRover->roverStatus().course().back().odom;
            Odometry nextOdom = createOdom( startPoint, bearing, newDifference, gRover );
            Waypoint nextWaypoint;
            //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
            nextWaypoint.odom = nextOdom;
            nextWaypoint.type = "searchPoint";
            gRover->roverStatus().course().push_back(nextWaypoint);
        }
         Waypoint nextWaypoint;
        //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
        nextWaypoint.odom = point2.odom;
        nextWaypoint.type = "searchPoint";
        gRover->roverStatus().course().push_back(nextWaypoint);
    }
    else{
        Waypoint nextWaypoint;
        //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
        nextWaypoint.odom = point1.odom;
        nextWaypoint.type = "searchPoint";
        gRover->roverStatus().course().push_back(nextWaypoint);
        
        Waypoint nextWaypoint2;
        //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
        nextWaypoint2.odom = point2.odom;
        nextWaypoint2.type = "searchPoint";
        gRover->roverStatus().course().push_back(nextWaypoint2);

    }


   
    
} // insertIntermediatePoints()