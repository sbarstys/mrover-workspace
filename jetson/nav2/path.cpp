#include "path.hpp"

Path::Path(std::vector<Odometry> path_in) : path(path_in) {}

// Returns the path to caller
std::vector<Odometry>& Path::getPath(){
    return path;
}

// Add new point to the path
void Path::addPoint(Odometry& point){
    path.push_back(point);
}

// Completely empties the path
void Path::clear(){
    path.clear();
}

// Calculates the closest point (could this be some kind of sort using the rover's distance?)
std::vector<Odometry>::iterator Path::getClosestPointToRover(){
    //TODO: implement this
    return path.begin();
}