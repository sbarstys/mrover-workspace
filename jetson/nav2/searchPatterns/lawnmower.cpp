#include "lawnmower.hpp"

void generateLawnmowerSearchPattern(){
    const double searchBailThresh = roverConfig[ "search" ][ "bailThresh" ].GetDouble();

    vector< pair<short, short> > mSearchPointMultipliers;
    // mSearchPointMultipliers.push_back( pair<short, short> (  0, 0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  0, 1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, 0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -2, 0 ) );


    while( fabs(mSearchPointMultipliers[ 0 ].first * visionDistance) < searchBailThresh )
    {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = gRover->roverStatus().course().peekFront();

            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * visionDistance  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * (2 * searchBailThresh) * rover->longMeterInMinutes() );

            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            Waypoint nextWaypoint;
            //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
            nextWaypoint.odom = nextSearchPoint;
            nextWaypoint.type = "searchPoint";
            gRover->roverStatus().course().pushBack(nextWaypoint);

            mSearchPointMultiplier.first -= 2;
            mSearchPoints.push_back( nextSearchPoint );
        }
    }
    //FIX THIS
    insertIntermediatePoints();
}