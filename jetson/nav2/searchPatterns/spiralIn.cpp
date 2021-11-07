#include "spiralIn.hpp"

using namespace std;

void generateSpiralInPattern(){
    vector<pair<short, short>> mSearchPointMultipliers;

    mSearchPointMultipliers.clear();
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  0 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1, -1 ) );

    Waypoint prev;
    bool first = true;
    double visionDistance = (gRover->roverStatus().mRoverConfig())["search"]["visionDistance"];
    while( mSearchPointMultipliers[ 0 ].second * visionDistance < (gRover->roverStatus().mRoverConfig())[ "search" ][ "bailThresh" ].GetDouble() ) {
        
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = gRover->roverStatus().course().back().odom;
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
            if (!first){
                insertWaypointsIntoCourse(prev, nextWaypoint, visionDistance);
            }
            else{
                 gRover->roverStatus().course().push_back(nextWaypoint);
                 first = false;
            }
            

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;

            prev = nextWaypoint;

        }
    }
    

}