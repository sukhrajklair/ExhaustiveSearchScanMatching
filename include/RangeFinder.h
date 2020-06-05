#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include <vector>

class RangeFinder
{
    private:
        float _fov;
        float _res;
        float _maxRange;
        std::vector<float> _data;
    public:
        //default constructor
        RangeFinder();
        //constructor with rangefinder parameters
        RangeFinder(float fov, float res, float maxRange);
        //set data
        void setData(std::vector<float> data);
        //get data
        std::vector<float> getData();
        //get fov
        float getFov(){ return _fov;}
        //get resolution
        float getRes(){ return _res;}
        //get max range
        float getMaxRange(){ return _maxRange;}
};

#endif