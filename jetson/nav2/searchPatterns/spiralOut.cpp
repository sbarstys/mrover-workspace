#include "spiralOut.hpp"

using namespace std;

void generateSpiralOutPattern(){
    //generates the spiral out pattern
    const double visionDistance = gRover->RoverConfig()[ "computerVision" ][ "visionDistance" ].GetDouble();


    vector< pair<short, short> > mSearchPointMultipliers;
    mSearchPointMultipliers.push_back( pair<short, short> (  0,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1, -1 ) );

    bool first;
    Waypoint prev;

    while( mSearchPointMultipliers[ 0 ].second * visionDistance < gRover->RoverConfig()[ "search" ][ "bailThresh" ].GetDouble() ) {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = gRover->roverStatus().course().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * visionDistance  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * visionDistance * gRover->longMeterInMinutes() );
            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            Waypoint nextWaypoint;
            //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
            nextWaypoint.odom = nextSearchPoint;
            nextWaypoint.type = "searchPoint";
            
            //interpolate between previous point and this point (interpolation adds points in range (first, last] w/inc visionDistance  )
            if (!first){
                insertWaypointsIntoCourse(prev, nextWaypoint, gRover->RoverConfig()["search"]["visionDistance"].GetDouble());
            }
            else{
                 gRover->roverStatus().course().push_back(nextWaypoint);
                 first = false;
            }

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;

        }
    }

}