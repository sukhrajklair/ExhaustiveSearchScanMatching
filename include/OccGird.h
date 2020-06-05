#ifndef OCCGRID_H
#define OCCGRID_H

#include <vector>
#include <cstdint>

struct pose
{
    float x;
    float y;
    float theta;
};

class OccGrid
{
    private:
        //2d vector to represent grid
        std::vector<std::vector<uint8_t>> _data;
        pose _pose;
        float _height;
        float _width;
        float _res;

    public:
        OccGrid();
        OccGrid(std::vector<std::vector<uint8_t>> data, pose pos, float height, float width);
        //return pose
        pose getPose(){ return _pose;}
        //take a pose in grid coordinates, discretize it into grid cell and returns the state of the cell
        int isOccupied(pose pos); 
};


#endif