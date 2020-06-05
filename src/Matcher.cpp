#include "Matcher.h"

//costrunt Matcher class
Matcher::Matcher(std::unique_ptr<OccGrid> grid, std::unique_ptr<RangeFinder> rangeData, pose worldPose, pose poseTolerance, float linearSearchRes, float angularSearchRes) :
    _pGrid(std::move(grid)), _pRangeData(std::move(rangeData)), _worldPose(worldPose), _poseTolerance(poseTolerance), _linearSearchRes(linearSearchRes), _angularSearchRes(angularSearchRes)
{

}

//transform a coordinate in a relative frame into the origin frame
pose Matcher::transform(pose relativePose, pose relativeOrigin)
{
    pose refPose;
    refPose.x = std::cos(relativeOrigin.theta)*relativePose.x - std::sin(relativeOrigin.theta)*relativePose.y + relativeOrigin.x;
    refPose.y = std::sin(relativeOrigin.theta)*relativePose.x + std::cos(relativeOrigin.theta)*relativePose.y + relativeOrigin.y;
    refPose.theta = relativePose.theta + relativeOrigin.theta;

    return refPose;
}

//transform a coordinate in origin frame into a relative frame
pose Matcher::inverseTransform(pose originPose, pose relativeOrigin)
{
    pose relativePose;
    relativePose.x = std::cos(relativeOrigin.theta)*(originPose.x - relativeOrigin.x) + std::sin(relativeOrigin.theta)*(originPose.y - relativeOrigin.y);
    relativePose.y = -1 * std::sin(relativeOrigin.theta)*(originPose.x - relativeOrigin.x) + std::cos(relativeOrigin.theta)*(originPose.y - relativeOrigin.y);
    relativePose.theta = originPose.theta - relativeOrigin.theta;

    return relativePose;
}

//returns euclidean distance between two pose
float Matcher::distance(pose p1, pose p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

//return grid pose of rangefinder from world pose
pose Matcher::getGridPose()
{
    return inverseTransform(_worldPose, _pGrid->getPose());
}
//receive new rangefinder data
void Matcher::receiveRangeData(std::vector<float> data)
{
    _pRangeData->setData(data);
}
//generate point clouds from the rangefinder data
std::vector<pose> Matcher::generatePointClouds(pose gridPose)
{
    std::vector<float> rangeData = _pRangeData->getData();
    float fov = _pRangeData->getFov();
    float res = _pRangeData->getRes();
    float maxRange = _pRangeData->getMaxRange();

    //generate point clouds from rangefinder data and transform them into grid coordinates
    //using pose struct to keep everything consistent even though theta is not useful here
    std::vector<pose> pointClouds;
    for (int i=0; i<rangeData.size(); i++)
    {
        //if range value is more than max range, there is no obstuction present
        if (rangeData[i] <= maxRange)
        {
            pose point= {};
            point.theta = -1*fov/2 + i * res;
            //coordinates of point relative to the grid position of the rangefinder
            point.x = rangeData[i] * std::cos(point.theta);
            point.y = rangeData[i] * std::sin(point.theta);

            //transform the point from rangefinder's frame to grid frame
            pose gridPoint = transform(point, gridPose);

            pointClouds.push_back(gridPoint);
        }
    }
    return pointClouds;
}
////return the best estimated pose after matching the given occupancy grid data to the data measured by the rangefinder
pose Matcher::match(){
    //transform the pose of rangefinder from world into grid coordinates
    pose gridPose = getGridPose();

    //generate point clouds
    std::vector<pose> pointClouds = generatePointClouds(gridPose);

    //generate the possible search space based on tolerance and search resolution
    //each possible value of x,y and theta is stored in a separate vector
    std::vector<float> searchSpaceTheta;
    std::vector<float> searchSpaceX;
    std::vector<float> searchSpaceY;

    for(float i = -1*_poseTolerance.theta; i < _poseTolerance.theta; i += _angularSearchRes)
    {
        searchSpaceTheta.push_back(gridPose.theta + i);
    }
    for(float i = -1*_poseTolerance.x; i < _poseTolerance.x; i += _linearSearchRes)
    {
        searchSpaceX.push_back(gridPose.x + i);
    }
    for(float i = -1*_poseTolerance.y; i < _poseTolerance.y; i += _linearSearchRes)
    {
        searchSpaceY.push_back(gridPose.y + i);
    }

    //for each permutation of x,y and theta in search space, rotate and translate the point clouds, 
    //compare the point clouds to the occupancy grid map and generate a score by adding valued of grid 
    //cells that corresponds to each point cloud
    int maxScore = 0;
    pose bestPose = gridPose;
    for(auto theta:searchSpaceTheta){
        for(auto x:searchSpaceX){
            for(auto y:searchSpaceY){
                pose possiblePose = {x, y, theta};
                int score = 0;
                for (auto point:pointClouds)
                {
                    //transform the point cloud relative to the possible pose
                    pose transPoint = transform(point, possiblePose);
                    //check if the corresponding cell in grid is occupied or not and increase the score accordingly
                    score += _pGrid->isOccupied(transPoint);
                }
                //the possiblePose is the best pose if it has highest score
                if (score > maxScore)
                {
                    bestPose = possiblePose;
                    maxScore = score;
                }
                //if score of the possiblePose is equal to highest, then select the pose which is closer to the approximate pose
                if (score == maxScore)
                {
                    if (distance(possiblePose, gridPose)<distance(bestPose,gridPose))
                    {
                        bestPose = possiblePose;
                    }
                }
            }
        }
    }

    //once the best pose has been determined, transform it back to world frame coordinates
    return transform(bestPose, _pGrid->getPose());

}