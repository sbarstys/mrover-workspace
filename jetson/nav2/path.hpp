#include "rover_msgs/Odometry.hpp"
#include <vector>

class Path{
    
    public:

    Path(std::vector<Odometry> path_in);

    std::vector<Odometry>& getPath();

    void clear();

    std::vector<Odometry>::iterator getClosestPointToRover();

    void addPoint(Odometry& point);
    
    ~Path();

    private:
    std::vector<Odometry> path;
};