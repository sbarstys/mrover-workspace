#include "utilities.hpp"
#include <iostream> // remove
#include <iomanip>
#include <cmath>
#include <vector>


ostream& operator<<(ostream& os, const Point &p) {
    std::streamsize ss = os.precision();

    os << std::setprecision(8);
    os <<  "Point(" << p.getX() << ", " << p.getY() << ")";
    os << std::setprecision(ss);

    return os;
}

// Coverts the input degree (and optional minute) to radians.
double degreeToRadian( const double degree, const double minute )
{
    return ( PI / 180 ) * ( degree + minute / 60 );
} // degreeToRadian

// Converts the input radians to degrees.
double radianToDegree( const double radian )
{
    return radian * 180 / PI;
}

// create a new odom with coordinates offset from current odom by a certain lat and lon change
Odometry addMinToDegrees( const Odometry & current, const double lat_minutes, const double lon_minutes )
{
    Odometry newOdom = current;
    double total_lat_min = current.latitude_min + lat_minutes;
    int sign_lat = total_lat_min < 0 ? -1 : 1;
    newOdom.latitude_min = mod( fabs( total_lat_min ), 60 ) * sign_lat;
    newOdom.latitude_deg += ( total_lat_min ) / 60;
    double total_lon_min = current.longitude_min + lon_minutes;
    int sign_lon = total_lon_min < 0 ? -1 : 1;
    newOdom.longitude_min = mod( fabs( total_lon_min ), 60 ) * sign_lon;
    newOdom.longitude_deg += ( total_lon_min )/60;

    return newOdom;
}

// Caclulates the non-euclidean distance between the current odometry and the
// destination odometry.
double estimateNoneuclid( const Odometry& current, const Odometry& dest )
{
    double currentLat = degreeToRadian( current.latitude_deg, current.latitude_min );
    double currentLon = degreeToRadian( current.longitude_deg, current.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double diffLat = ( destLat - currentLat );
    double diffLon = ( destLon - currentLon ) * cos( ( currentLat + destLat ) / 2 );
    return sqrt( diffLat * diffLat + diffLon * diffLon ) * EARTH_RADIUS;
}

// create a new Odometry point at a bearing and distance from a given odometry point
// Note this uses the absolute bearing not a bearing relative to the rover.
Odometry createOdom( const Odometry & current, double bearing, const double distance, Rover * rover )
{
    bearing = degreeToRadian( bearing );
    double latChange = distance * cos( bearing ) * LAT_METER_IN_MINUTES;
    double lonChange = distance * sin( bearing  ) * rover->longMeterInMinutes();
    Odometry newOdom = addMinToDegrees( current, latChange, lonChange );
    return newOdom;
}

// create a new Odometry point at a bearing and distance from a given odometry point
// Note this uses the absolute bearing not a bearing relative to the rover.
Odometry createOdom( const Odometry & current, double bearing, const double distance )
{
    bearing = degreeToRadian( bearing );
    double latChange = distance * cos( bearing ) * LAT_METER_IN_MINUTES;
    // TODO check if this works as expected
    double lonChange = distance * sin( bearing  ) * ( 60 / ( EARTH_CIRCUM * cos( degreeToRadian(
                current.latitude_deg, current.latitude_min ) ) / 360 ));

    Odometry newOdom = addMinToDegrees( current, latChange, lonChange );
    return newOdom;
}

// Caclulates the bearing between the current odometry and the
// destination odometry.
double calcBearing( const Odometry& start, const Odometry& dest )
{
    double currentLat = degreeToRadian( start.latitude_deg, start.latitude_min );
    double currentLon = degreeToRadian( start.longitude_deg, start.longitude_min );
    double destLat = degreeToRadian( dest.latitude_deg, dest.latitude_min );
    double destLon = degreeToRadian( dest.longitude_deg, dest.longitude_min );

    double verticleComponentDist = EARTH_RADIUS * sin( destLat - currentLat );
    double noneuclidDist = estimateNoneuclid( start, dest );

    double bearing = acos( verticleComponentDist / noneuclidDist );
    if( currentLon > destLon )
    {
        bearing = 2 * PI - bearing;
    }

    if( verticleComponentDist < 0.001 && verticleComponentDist > -0.001 )
    {
        if( currentLon < destLon )
        {
            bearing = PI / 2;
        }
        else
        {
            bearing = 3 * PI / 2;
        }
    }
    return radianToDegree( bearing );
} // calcBearing()

Odometry calcCarrot(Rover* rover, const Odometry& c1, const Odometry& c2, bool& turn){
    //generate equation for C1C2
    // Determine dy offset of rover from line
    //      find equation perpendicular to C1C2 and passing through R1R2
    //      find intersection point of both lines (I1I2)
    //      calculate distance between R1R2 and I1I2 (this is dy)
    // Determine coordinate of P
    //      Move X distance down the C1C2 line from point I1I2


    double thetaC1C2 = calcBearing(c1,c2);


    // Create "points" representing the two positions
    Point c1p(c1);
    Point c2p(c2);
    Point roverPoint(rover->roverStatus().odometry());

    // Define a line connecting the two carrots
    Line targetCarrotLine(c1p, c2p);

    // Line from the rover to the carrot line (perpendicularly)
    Line beelineToCarrotLine = targetCarrotLine.perpendicularLine(roverPoint);

    // The point where on the line where the rover should turn to (directly)
    Point intersectionPoint = targetCarrotLine.intersection(beelineToCarrotLine);

    // make intersection point to Odometry
    int lonDeg = (int)intersectionPoint.getX();
    double lonMin = (intersectionPoint.getX() - lonDeg) * 60;

    int latDeg = (int)intersectionPoint.getY();
    double latMin = (intersectionPoint.getY() - latDeg) * 60;

    Odometry intersectPt = {latDeg, latMin, lonDeg, lonMin, 0, 0};

    // TODO find the distance dy from the rover to the intersect of line L
    double dy = estimateNoneuclid(rover->roverStatus().odometry(), intersectPt);
    // cout << "rover Odometry latDeg " << rover->roverStatus().odometry().latitude_deg << "\n";
    // cout << "rover Odometry latMin " << rover->roverStatus().odometry().latitude_min << "\n";
    // cout << "rover Odometry lonDeg " << rover->roverStatus().odometry().longitude_deg << "\n";
    // cout << "rover Odometry lonMin " << rover->roverStatus().odometry().longitude_min << "\n";

    // cout << "intersect Odometry latDeg " << intersectPt.latitude_deg << "\n";
    // cout << "intersect Odometry latMin " << intersectPt.latitude_min << "\n";
    // cout << "intersect Odometry lonDeg " << intersectPt.longitude_deg << "\n";
    // cout << "intersect Odometry lonMin " << intersectPt.longitude_min << "\n";

    double threshold = 0.5;
    double dx;

    // how far carrot is along the line (from the intercept point)
    // if rover is within the the threshold
    const double k = 0.52;
    turn = false;

     // calculate delta x
    if( dy > threshold ){
        dx = 0.0;
        turn = true;
    }
    else if( dy == 0.0 ){
        dx = k;
    }
    else {
        // proportional control
        //dx = (constantToFind / dy) + k;
        dx = k;
    }

    std::cout << "Rover bearing: " << rover->roverStatus().odometry().bearing_deg << "\n";
    std::cout << "thetaC1C2: " << thetaC1C2 << "\n";
    std::cout << "dy: " << dy << "\n";
    std::cout << "dx: " << dx << "\n";

    cout << "Create Odom Called\n";
    Odometry carrot = createOdom(intersectPt, thetaC1C2, dx);
    return carrot;
} // calcCarrot()


// // Calculates the modulo of degree with the given modulus.
double mod( const double degree, const int modulus )
{
    double mod = fmod( degree, modulus );
    if( mod < 0 )
    {
        return ( mod + modulus );
    }
    return mod;
}

// Corrects the destination bearing to account for the ability to turn
// through zero degrees.
void throughZero( double& destinationBearing, const double currentBearing )
{
    if( fabs( currentBearing - destinationBearing ) > 180 )
    {
        if( currentBearing < 180 )
        {
            destinationBearing -= 360;
        }
        else
        {
            destinationBearing += 360;
        }
    }
} // throughZero()

// Clears the queue.
void clear( deque<Waypoint>& aDeque )
{
    deque<Waypoint> emptyDeque;
    swap( aDeque, emptyDeque );
} // clear()


// Checks to see if target is reachable before hitting obstacle
// If the x component of the distance to obstacle is greater than
// half the width of the rover the obstacle if reachable
bool isTargetReachable( Rover* rover, const rapidjson::Document& roverConfig )
{
    double distToTarget = rover->roverStatus().leftTarget().distance;
    double distThresh = roverConfig["navThresholds"]["targetDistance"].GetDouble();
    return isLocationReachable( rover, roverConfig, distToTarget, distThresh );
} // istargetReachable()

// Returns true if the rover can reach the input location without hitting the obstacle.
// ASSUMPTION: There is an obstacle detected.
// ASSUMPTION: The rover is driving straight.
bool isLocationReachable( Rover* rover, const rapidjson::Document& roverConfig, const double locDist, const double distThresh )
{
    double distToObs = rover->roverStatus().obstacle().distance;
    double bearToObs = rover->roverStatus().obstacle().bearing;
    double bearToObsComplement = 90 - bearToObs;
    double xComponentOfDistToObs = distToObs * cos( bearToObsComplement );

    bool isReachable = false;

    // if location - distThresh is closer than the obstacle, it's reachable
    isReachable |= distToObs > locDist - distThresh;

    // if obstacle is farther away in "x direction" than rover's width, it's reachable
    isReachable |= xComponentOfDistToObs > roverConfig["roverMeasurements"]["width"].GetDouble() / 2;

    return isReachable;
} // isLocationReachable()

// Returns true if an obstacle is detected, false otherwise.
bool isObstacleDetected( Rover* rover )
{
    return rover->roverStatus().obstacle().distance >= 0;
} // isObstacleDetected()

// Returns true if distance from obstacle is within user-configurable threshold
bool isObstacleInThreshold( Rover* rover, const rapidjson::Document& roverConfig )
{
    return rover->roverStatus().obstacle().distance <= roverConfig["navThresholds"]["obstacleDistanceThreshold"].GetDouble();
} // isObstacleInThreshold()



Line::Line(double m_in, double b_in) : m(m_in), b(b_in) { }

double Line::findY(double x) const {
    // y = mx + b
    return m * x + b;
}
double Line::findX(double y) const {
    // y = mx + b
    // y - b = mx
    // (y - b) / m = x
    return (y - b) / m;
}

Point Line::intersection(const Line &other) const {
    // Find the intersection of two lines
    // This is the math behind it:
    // 1. y = m1 x + b1
    // 2. y = m2 x + b2
    // 3. m1 x + b1 = m2 x + b2
    // 4. (m1 - m2) x = b2 - b1
    // 5. x = (b2 - b1) / (m1 - m2)
    // 6. y = mx + b

    double x = (other.b - b) / (m - other.m);
    double y = findY(x);
    return Point(x, y);
}
Line Line::perpendicularLine(const Point &include) const {
    // Create a line perpendicular to this, including a point specified

    // The slope of a perpendicular line is the negative reciprocal (-1 / m)
    double m2 = -1.0 / m;
    // We have a point on the line (provided as an argument), so use it
    // to find the intercept b.
    double b2 = include.getY() - (m2 * include.getX());
    return Line(m2, b2);
}


Point::Point(double x_in, double y_in) : x(x_in), y(y_in) { }

Point::Point(const Odometry &point) :
    x(point.longitude_deg + point.longitude_min / 60.0),
    y(point.latitude_deg + point.latitude_min / 60.0) { }

Line::Line(const Point &point1, const Point &point2) :
    // Create a line using two points
    // m = rise / run
    m((point2.getY() - point1.getY()) /
            (point2.getX() - point1.getX())),
    // b = y - m*x
    b(point2.getY() - (m * point2.getX())) { }


Odometry Point::toOdometry() {
    int long_deg = (int)getX();
    double long_min = (x - long_deg) * 60.0;
    int lat_deg = (int)getY();
    double lat_min = (y - lat_deg) * 60.0;

    return Odometry{lat_deg, lat_min, long_deg, long_min, 0, 0};
}


double Point::getX() const {
    return x;
}

double Point::getY() const {
    return y;
}

double Point::distance(const Point& other) {
    return (other.x - x) * (other.x - x) + (other.y - y) * (other.y - y);
}
