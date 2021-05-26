#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>

#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <fstream>

#include <ros/console.h>
#include "ros/ros.h"
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>

#define RAD2DEG(rad) ((rad) * 180./M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float xStart = 0.0, yStart = 0.0, phiStart = 0.0;
float boxDimension = 0.45;
float goal_x[11]; // 10 boxes + starting position
float goal_y[11];
float goal_phi[11];
int goalBoxNumber = -1; // initial starting point
bool gotPlan;

Boxes boxes;
RobotPose robotPose(0,0,0);

// store values for result output
std::vector<float> storeTag;
// store location in order of visit
std::vector<int> storeBoxNumber;
std::vector<float> storeX;
std::vector<float> storeY;
std::vector<float> storePhi;

// Create array of 0, change to 1 if that box has a duplicate tag
int duplicate[10] = {};
// Create array of 0, corresponding to unexamined object, and 1 for examined object
int examined[10] = {};
// Ultimate TSP path between boxes
int pathUlti[10] = {};



//Replace the coords of the image's center with coords that the robot can go to
void goalPosition(){
    for (int i = 0; i < boxes.coords.size(); ++i){
        goal_x[i] = boxes.coords[i][0] + boxDimension * cos(boxes.coords[i][2]);
        goal_y[i] = boxes.coords[i][1] + boxDimension * sin(boxes.coords[i][2]);
        
        if (boxes.coords[i][2] < 0.0)
        {
            goal_phi[i] = (boxes.coords[i][2] + M_PI);
        }
        else
        {
            goal_phi[i] = (boxes.coords[i][2] - M_PI);
        }
    }
    goal_x[10] = xStart;
    goal_y[10] = yStart;
    goal_phi[10] = phiStart;
}


void optimizePath(){
    
    //create a matrix of distance lengths between boxes (and starting point - 11)
    float pathsMatrix [11][11] = {0.0};


    for (int i = 0; i < 11; i++){
        for (int j = 0; j < 11; j++){
            pathsMatrix[i][j] = std::sqrt(std::pow(goal_x[i] - goal_x[j], 2) + std::pow(goal_y[i] - goal_y[j], 2));
        }
    }

    // permutation matrix for orders of boxes to travel to
    int goals[10]; 
    for (int k = 0; k < 10; k++){
        goals[k] = k;
    }

    // check all possible permutations for shortest path
    // documentation / example for next_permutation here: http://www.cplusplus.com/reference/algorithm/next_permutation/
    float minPath = std::numeric_limits<float>::infinity();
    float pathLength = 0.0;
    do {
        //first path length from start Position to first box
        pathLength += pathsMatrix[10][goals[0]];

        //Calculate path between boxes
        for (int k = 0; k < 9; k++){
            pathLength += pathsMatrix[goals[k]][goals[k+1]];
        }

        //last path length from last box to start Position
        pathLength += pathsMatrix[10][goals[9]];
    
        //new shortest path detected! Save length + order of boxes
        if (minPath > pathLength){
            minPath = pathLength;
            std::copy(goals, goals + 10, pathUlti);
        }
        pathLength = 0.0; //reset for next permutation
    } while (std::next_permutation(goals, goals+10)); //check all possible permutations, holding starting point steady

    ROS_INFO("The shortest path length is: %f", minPath);
    ROS_INFO("Visit boxes in order: ");

    for (int i = 0; i < 10; i++){
        ROS_INFO("%i", pathUlti[i]);
    }
}


void checkGoal(ros::ServiceClient &serviceClient, std::vector<float> goal, RobotPose robotPose){
    
    nav_msgs::GetPlan srv;

    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(robotPose.phi);//initial position phi quaternion coordinate
    srv.request.start.header.frame_id ="map";
    srv.request.start.pose.position.x = robotPose.x;//initial position x coordinate
    srv.request.start.pose.position.y = robotPose.y;//initial position y coordinate
    srv.request.start.pose.position.z =  0.0;
    srv.request.start.pose.orientation.x = 0.0;
    srv.request.start.pose.orientation.y = 0.0;
    srv.request.start.pose.orientation.z = phi.z;
    srv.request.start.pose.orientation.w = phi.w;//Orientation

    geometry_msgs::Quaternion phiGoal = tf::createQuaternionMsgFromYaw(goal[2]);
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goal[0];//End point coordinates
    srv.request.goal.pose.position.y = goal[1];
    srv.request.goal.pose.position.z = 0.0;
    srv.request.goal.pose.orientation.x = 0.0;
    srv.request.goal.pose.orientation.y = 0.0;
    srv.request.goal.pose.orientation.z = phiGoal.z;
    srv.request.goal.pose.orientation.w = phiGoal.w;

    srv.request.tolerance = 0.0;//If the goal cannot be reached, the nearest available constraint

    if (serviceClient.call(srv)) {
                 //srv.response.plan.poses is the container for storing the results, traversed and taken out
        if (!srv.response.plan.poses.empty()) {
//            std::for_each(srv.response.plan.poses.begin(),srv.response.plan.poses.end(),myfunction);
            ROS_INFO("make_plan success!");
            gotPlan = true;
        }
    }
    else {
        ROS_WARN("Got empty plan");
        gotPlan = false;
    }
    return;
}


void adjustGoal(ros::ServiceClient make_plan_client){
    int iter;
    // Check if the goal location of the box is valid (use make_plan to check if path exists)
    gotPlan = false;
    std::vector<float> tryGoal;
    tryGoal.push_back(goal_x[goalBoxNumber]);
    tryGoal.push_back(goal_y[goalBoxNumber]);
    tryGoal.push_back(goal_phi[goalBoxNumber]);
    // Iteration count for how many new goals tried if initial goal is not valid
    iter = 1;
    // Check positive direction first
    std::cout << "initial x: " << tryGoal[0] << ", initial y: " << tryGoal[1] << ", initial phi: " << tryGoal[2] << std::endl;

    //Check until valid plan found, or full +-75 degrees is swept
    while (gotPlan == false && iter >= -15){
        //Check if current config results in a traversable goal
        checkGoal(make_plan_client, tryGoal, robotPose);
        //Valid goal is found - set it
        if (gotPlan == true){
            goal_x[goalBoxNumber] = tryGoal[0];
            goal_y[goalBoxNumber] = tryGoal[1];
            goal_phi[goalBoxNumber] = tryGoal[2];
        }
        //Valid goal is not found - increment and try again
        else{
            tryGoal[2] = goal_phi[goalBoxNumber] + iter*DEG2RAD(5);
            // Adjust goal angle to within +-M_PI
            if (tryGoal[2] < -M_PI){
                tryGoal[2] += 2*M_PI;
            }
            else if (tryGoal[2] > M_PI){
                tryGoal[2] -= 2*M_PI;
            }

            tryGoal[0] = boxes.coords[goalBoxNumber][0] - boxDimension * cos(tryGoal[2]);
            tryGoal[1] = boxes.coords[goalBoxNumber][1] - boxDimension * sin(tryGoal[2]);

            std::cout << "testing x: " << tryGoal[0] << "testing y: " << tryGoal[1] << "testing phi: " << tryGoal[2] << std::endl;
        }
        
        // If we checked both left and right angle increments, we try a larger angle offset goal position
        if (iter > 0){
            //sweep 5 degrees CCW
            iter += 1;
        }
        else{
            //sweep 5 degrees CW
            iter -=1;
        }

        //Once we have exceeded 75 degrees CCW sweep
        if (iter >=15){
            // Switch direction of checking to CW
            iter = -1;
        }
        
        std::cout << "gotPlan? (0 = false, 1 = true) " << gotPlan << std::endl;
    }
}


//Mark the stored tags that appear more than once as 1, to show in Results.txt
void checkDuplicate(std::vector<float> storeTag){
    for (int i =0; i < storeTag.size(); ++i){
        for (int j =0; j < storeTag.size(); ++j){
            //if (i != j){
            if (i < j){
                if (storeTag[i] == storeTag[j]){
                    duplicate[j] = 1;
                }
            }
        }
    }
}


//Open, write, and save the stored results after the robot finishes scanning
void storeInfo(std::vector<int> storeBoxNumber, std::vector<float> storeTag, std::vector<float> storeX, std::vector<float> storeY, std::vector<float> storePhi){

    std::ofstream output("/home/turtlebot/catkin_ws/src/mie443_contest2/Results.txt");

    for (int i =0; i < storeTag.size(); ++i){
        std::string blankTag = "tag_blank.jpg";

        output << "Visit #";
        output << i+1;
        output << ": ";
        output << "For box number ";
        output << storeBoxNumber[i];
        output << ", ";
        
        if (storeTag[i] == 0){
            output << blankTag;
            output << " was found at X = ";
            output << storeX[i];
            output << ", Y = ";
            output << storeY[i];
            output << ", Phi = ";
            output << storePhi[i];
            if (duplicate[i] == 1){
                output << ", and this is a duplicate tag.";
            }
            output << "\n";
        }

        else{       
            output << "tag_";
            output << storeTag[i];
            output << ".jpg";
            output << " was found at X = ";
            output << storeX[i];
            output << ", Y = ";
            output << storeY[i];
            output << ", Phi = ";
            output << storePhi[i];
            if (duplicate[i] == 1){
                output << ", and this is a duplicate tag.";
            }
            output << "\n";
        }
    }
    output.close();
}



int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    //RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    //Boxes boxes;

    ros::ServiceClient make_plan_client = n.serviceClient<nav_msgs::GetPlan>("move_base/NavfnROS/make_plan",true);
    if (!make_plan_client) {
        ROS_FATAL("Could not initialize get plan service from %s", make_plan_client.getService().c_str());
        return -1;
    }

    // Keep track of time for 8 minute limit
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // give time for initial starting position to be saved properly
    while (secondsElapsed < 1)
    {
        ros::spinOnce();
        xStart = robotPose.x;
        yStart = robotPose.y;
        phiStart = robotPose.phi;
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    }

    std::cout << "Initial Position to return to: " << "x: " << xStart << " y: " << yStart << " phi: " << phiStart << std::endl;

    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    
    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n);
    int templateID;

    //Replace the coords of the image's center with coords that the robot can go to
    goalPosition();
    optimizePath();
    ROS_INFO("goal positions calculated");
    int goalBoxIndex = 0;

    // Execute strategy, within 8 minute time limit
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        // break out of box-to-box navigation loop and go back to start once the path is complete
        if (goalBoxIndex == -1){
            std::cout << "Returning to the starting coodinates..." << std::endl;
            Navigation::moveToGoal(xStart,yStart,phiStart);
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
            std::cout << "Final Time: " << secondsElapsed << " seconds" << std::endl;
            std::cout << "Finished! The results have been saved at /home/turtlebot/catkin_ws/src/mie443_contest2/Results.txt" << std::endl;
            break;
        }

        // Set current goal box from TSP list
        goalBoxNumber = pathUlti[goalBoxIndex];        
        ROS_INFO("Goal box number: %i", goalBoxNumber);

        if (examined[goalBoxNumber] != 1){
            //Check to see if the goal coordinate is reachable, adjust with angled offset if needed
            adjustGoal(make_plan_client);
            //Move to box location
            Navigation::moveToGoal(goal_x[goalBoxNumber],goal_y[goalBoxNumber],goal_phi[goalBoxNumber]);
            ros::spinOnce();
            //Get image info
            templateID = imagePipeline.getTemplateID(boxes);

            // store result vectors
            storeBoxNumber.push_back(goalBoxNumber);
            //storeInfo(templateID) (goes from 0 to 14 -> tag 1 to 15)
            storeTag.push_back(templateID+1);
            //store the object location
            storeX.push_back(boxes.coords[goalBoxNumber][0]);
            storeY.push_back(boxes.coords[goalBoxNumber][1]);
            storePhi.push_back(boxes.coords[goalBoxNumber][2]);
            //Update the "examined" status of the box
            examined[goalBoxNumber] = 1; //box is now examined

            // Write and save Results.txt every visit just in case of force-quit or unexpected error.
            // Mark the duplicate tags in storeTag
            checkDuplicate(storeTag);
            // Output the info to text file
            storeInfo(storeBoxNumber, storeTag, storeX, storeY, storePhi);
        }

        //Increment to next goal box in list
        if (goalBoxIndex == 10) {
            goalBoxIndex = -1;
        }
        else {
            goalBoxIndex++;
        }

        // Use: robotPose.x, robotPose.y, robotPose.phi
        ros::Duration(0.01).sleep();
        // Update timer
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        std::cout << "Time Elapsed: " << secondsElapsed << " seconds" << std::endl;
    }
    
    return 0;
}
