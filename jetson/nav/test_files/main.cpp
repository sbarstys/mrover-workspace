#include <iostream>


struct Odometry {
    int32_t latitude_deg;
    double latitude_min;
    int32_t longitude_deg;
    double longitude_min;
    double bearing_deg;
    double speed;
}


double calcCarrot(Odometry& rover, const Odometry& c1, const Odometry& c2){
    //generate equation for C1C2
    // Determine dy offset of rover from line
    //      find equation perpendicular to C1C2 and passing through R1R2
    //      find intersection point of both lines (I1I2)
    //      calculate distance between R1R2 and I1I2 (this is dy)
    // Determine coordinate of P
    //      Move X distance down the C1C2 line from point I1I2
    double thetaC1C2 = calcBearing(c1,c2);

    // convert lat, lon into decimal
    // center point 1
    double c1Lat = c1.latitude_deg + c1.latitude_min/60.0;
    double c1Lon = c1.longitude_deg + c1.longitude_min/60.0;
    //center point 2
    double c2Lat = c2.latitude_deg + c2.latitude_min/60.0;
    double c2Lon = c2.longitude_deg + c2.longitude_min/60.0;
    // rover
    double rLat = rover->roverStatus().odometry().latitude_deg + rover->roverStatus().odometry().latitude_min/60.0;
    double rLon = rover->roverStatus().odometry().longitude_deg + rover->roverStatus().odometry().longitude_min/60.0;



    double slope = (c1Lat - c2Lat) / (c1Lon - c2Lon);
    double intercept = c1Lat - (slope * c1Lon);
    Line C1C2Equation(slope, intercept);

    // Line from the rover to the
    double perpendicularSlope = -1 * (1/slope);
    double perpenIntercept = rLat - perpendicularSlope * rLon;
    Line C1C2PerpendicularEquation(perpendicularSlope, perpenIntercept);

    Point intersectionPoint = C1C2Equation.intersection(C1C2PerpendicularEquation);

    // make intersection point to Odometry
    int lonDeg = (int)intersectionPoint.getXCoordinate();
    double lonMin = (intersectionPoint.getXCoordinate() - lonDeg) * 60;

    int latDeg = (int)intersectionPoint.getYCoordinate();
    double latMin = (intersectionPoint.getYCoordinate() - latDeg) * 60;

    Odometry intersectPt = {latDeg, latMin, lonDeg, lonMin, 0, 0};

    // TODO find the distance dy from the rover to the intersect of line L
    cout << "Estimate nonEuclid called\n";
    double dy = estimateNoneuclid(rover->roverStatus().odometry(), intersectPt);
    cout << "Odometry latDeg " << intersectPt.latDeg << "\n";
    cout << "Odometry latMin " << intersectPt.latMin << "\n";
    cout << "Odometry lonDeg " << intersectPt.lonDeg << "\n";
    cout << "Odometry lonMin " << intersectPt.lonMin << "\n";

    double threshold = 0.5;
    double dx;

    // how far carrot is along the line (from the intercept point)
    // if rover is within the the threshold
    const double k = 0.2;

     // calculate delta x
    if( dy > threshold ){
        dx = 0;
    }
    else if( dy == 0 ){
        dx = k;
    }
    else {
        // proportional control
        //dx = (constantToFind / dy) + k;
        dx = k;
    }

    std::cout << "Rover bearing: " << rover.bearing_deg << "\n";
    std::cout << "thetaC1C2: " << thetaC1C2 << "\n";
    std::cout << "dy: " << dy << "\n";
    std::cout << "dx: " << dx << "\n";

    cout << "Create Odom Called\n";
    Odometry carrot = createOdom(intersectPt, thetaC1C2, dx);

    return calcBearing( rover, carrot );
} // calcCarrotBearing()

int main(){





    return 0;
}