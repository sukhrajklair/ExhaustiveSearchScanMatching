#ifndef MATCHER_H
#define MATCHER_H
#include <memory>
#include <cmath>

#include "OccGird.h"
#include "RangeFinder.h"



class Matcher
{
    private:
        //using pointers to the data allocated to heap instead of keeping local copies of the data
        std::unique_ptr<OccGrid> _pGrid;
        std::unique_ptr<RangeFinder> _pRangeData;
        pose _worldPose;
        pose _poseTolerance;
        float _linearSearchRes;
        float _angularSearchRes;

        //transform a coordinate in a relative frame into the origin frame
        pose transform(pose relativePose, pose relativeOrigin);

        //returns distance between two points
        float distance(pose p1, pose p2);
        
        //transform a coordinate in origin frame into a relative frame
        pose inverseTransform(pose originPose, pose relativeOrigin);

        //genrate point clouds from rangefinder data
        std::vector<pose> generatePointClouds(pose gridPose);

    public:
        Matcher(std::unique_ptr<OccGrid> grid, std::unique_ptr<RangeFinder> rangeData, pose worldPose, pose poseTolerance, float linearSearchRes, float angularSearchRes);
        //return grid pose of the rangefinder using the world pose
        pose getGridPose();
        //receive new rangefinder data
        //this needs to be implemented here because class Matcher owns the pRangeFinder object
        void receiveRangeData(std::vector<float>);
        //return the best estimated pose after matching the given occupancy grid data to the data measured by the rangefinder
        pose match();
};

#endif