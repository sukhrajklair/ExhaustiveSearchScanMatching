#include "RangeFinder.h"

//default constructor
RangeFinder::RangeFinder()
{
    _fov = 1;
    _res = 0.05;
    _maxRange = 10;
    //data vector of expected number of readings is constructued with values of 1 more than the maxRange
    //this means that the rangefinder doesn't see any obstruction
    _data = std::vector<float>(_fov/_res, _maxRange+1);
}
//constructor with rangefinder parameters
RangeFinder::RangeFinder(float fov, float res, float maxRange): _fov(fov), _res(res), _maxRange(maxRange)
{
    //data vector of expected number of readings is constructued with values of 1 more than the maxRange
    //this means that the rangefinder doesn't see any obstruction
    _data = std::vector<float>(_fov/_res, _maxRange+1);
}
//set data
void RangeFinder::setData(std::vector<float> data){
    _data = data;
}
//get data
std::vector<float> RangeFinder::getData(){
    return _data;
}