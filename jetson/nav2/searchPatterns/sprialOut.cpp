#include "spiralOut.hpp"

using namespace std;

void generateSpiralOutPattern(Course& course){
    //generates the spiral out pattern

    vector< pair<short, short> > mSearchPointMultipliers;
    mSearchPointMultipliers.push_back( pair<short, short> (  0,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1,  1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
    mSearchPointMultipliers.push_back( pair<short, short> (  1, -1 ) );

    while( mSearchPointMultipliers[ 0 ].second * visionDistance < roverConfig[ "search" ][ "bailThresh" ].GetDouble() ) {
        for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
        {
            Odometry nextSearchPoint = gRover->roverStatus().path().front().odom;
            double totalLatitudeMinutes = nextSearchPoint.latitude_min +
                ( mSearchPointMultiplier.first * visionDistance  * LAT_METER_IN_MINUTES );
            double totalLongitudeMinutes = nextSearchPoint.longitude_min +
                ( mSearchPointMultiplier.second * visionDistance * rover->longMeterInMinutes() );
            nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
            nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
            nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
            nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

            Waypoint nextWaypoint;
            //TODO: CHANGE THIS TO CORRECT TYPE IMPLEMENATION
            nextWaypoint.odom = nextSearchPoint;
            nextWaypoint.type = "searchPoint";
            

            mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
            mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;

        }
    }
    insertIntermediatePoints();

}