#include <iostream>
#include "Matcher.h"

int main(int argc, char* argv[])
{

    //allocating possibly large data on the heap and using smart pointer along with move semantics to avoid unneccessary copying
    std::unique_ptr<RangeFinder> pRangeFinder(new RangeFinder(1,0.05,10));
    //simulating rangeFinder
    pRangeFinder->setData(std::vector<float>(20, 1));

    //simulating occupancy grid
    std::vector<std::vector<uint8_t>> occData(400, std::vector<std::uint8_t>(400,0));
    for(int i = 18; i<22; i++){
        occData[18][i] = 255;
        occData[22][i] = 255;
        occData[i][18] = 255;
        occData[i][22] = 255;
    }
    std::unique_ptr<OccGrid> pGrid(new OccGrid(occData, pose{10.0, 10.0, 0}, 20, 20));

    pose worldPose = {10.0, 10.0, 1.0};
    pose poseTolerance = {0.5,0.5,0.5};
    float linearRes = 0.05;
    float angularRes = 0.05;

    Matcher scanMatcher(std::move(pGrid), std::move(pRangeFinder), worldPose, poseTolerance, linearRes, angularRes);

    //since grid origin lies at 10.0, 10.0, the gridpose of the rangefinder should be equal to 0.0, 0.0, 1.0
    pose gridPose = scanMatcher.getGridPose();
    std::cout << "The grid pose is (" << gridPose.x << ", " << gridPose.y << ", " << gridPose.theta << ")" << std::endl;
    pose bestPose = scanMatcher.match();
    std::cout << "The best pose is (" << bestPose.x << ", " << bestPose.y << ", " << bestPose.theta << ")" << std::endl;

    //load new data from rangefinder, no obstructions
    scanMatcher.receiveRangeData(std::vector<float>(20, 15));
    //get another pose estimation, should be same as the provided world pose as rangefinder provides no extra information
    bestPose = scanMatcher.match();
    std::cout << "The best pose is (" << bestPose.x << ", " << bestPose.y << ", " << bestPose.theta << ")" << std::endl;

}