#include "gateNodes.hpp"
#include "rover.hpp"
#include "utilities.hpp"
#include "rover_msgs/Waypoint.lcm"
#include "rover_msgs/Odometry.lcm"

#include <math.h>


namespace gateNodes{

    // TODO: not done
    BT::NodeStatus isFirstGatePostLocKnown(){
        if (gRover->roverStatus().FirstGatePostFound()){
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus genGateTraversalPath(){
        // leftGate ID = 4
        // rightGate ID = 5
        // 2 meters wide
        //camera depth 4-4.5 meters, BEST is 5m
        // waypoints should be no less than 1.5 meters from gate center
        //TODO: json.config file (put constants there later)

        /*
            alternative to calculate new odom for w1/w2
            (using linear equations/slopes)

            lat_mid = (lat1 + lat2) / 2
            long_mid = (long1 + long2) / 2
            m = (lat1 - lat2) / (long1 - long2)
            lat_waypoint1  = lat_mid  + sqrt(DIST^2 / (1 + 1/(m*m)))
            long_waypoint1 = long_mid + sqrt(DIST^2 / (1 + (m*m)))
        */

        const double BETWEEN_POSTS = 2;  // space between posts in meters
        const double DIST_MID_GATE = 1.6;  // distance from the midpoint of the gate to first traversal point
        const double DIST = sqrt(pow(BETWEEN_POSTS/2), 2) + pow(DIST_MID_GATE,2);  // distance from left post to first traversal point
        const uint8_t LEFT_GATE_ID = 4;
        const uint8_t RIGHT_GATE_ID = 5;


        Odometry left_post;
        Odometry right_post;
        // note: path will be generated if at least one post (ID) is identified correctly
        if (gRover->roverStatus().post1().id == LEFT_GATE_ID) {
            left_post = gRover->roverStatus().post1().location;
            right_post = gRover->roverStatus().post2().location;
        }
        else if (gRover->roverStatus().post1().id == RIGHT_GATE_ID){
            right_post = gRover->roverStatus().post1().location;
            left_post = gRover->roverStatus().post2().location;
        }
        else if (gRover->roverStatus().post2().id == LEFT_GATE_ID) {
            right_post = gRover->roverStatus().post1().location;
            left_post = gRover->roverStatus().post2().location;
        }
        else if (gRover->roverStatus().post2().id == RIGHT_GATE_ID){
            left_post = gRover->roverStatus().post1().location;
            right_post = gRover->roverStatus().post2().location;
        }
        else {
            // TODO figure out failure/recover functionality
            return BT::NodeStatus::FAILURE;
        }

        // TODO make sure this is pointing the right direction
        double phi = calcBearing(left_post, right_post);

        double theta1 = radianToDegree(atan2(DIST,BETWEEN_POSTS/2));

        Waypoint w1;
        w1.type = "gateTraversal";
        w1.odom = createOdom(left_post, phi+theta1);

        Waypoint w2;
        w2.type = "gateTraversal";
        w2.odom = createOdom(left_post, phi-theta1);

        gRover->roverStatus().course().push_back(w1);
        gRover->roverStatus().course().push_back(w2);

        return BT::NodeStatus::SUCCESS;
    }


    BT::NodeStatus isGateTraversalPoint(){
        if (gRover->roverStatus().course().peekTop().type == "gateTraversal"){
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    void registerNodes(BT::BehaviorTreeFactory& factory){
        factory.registerSimpleAction("isFirstGatePostLocKnown", std::bind(isFirstGatePostLocKnown));
        factory.registerSimpleAction("isGateTraversalPoint", std::bind(isGateTraversalPoint));
    }

}