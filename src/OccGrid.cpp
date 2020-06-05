
#include "OccGird.h"

//default constructor
OccGrid::OccGrid()
{
    _pose = {0, 0, 0};
    _height = 10;
    _width = 10;
    _res = 0.05;
    //generate an empty grid of widthxheight 
    _data = std::vector<std::vector<uint8_t>> (_width/_res, std::vector<std::uint8_t>(_height/_res,0));
}
OccGrid::OccGrid(std::vector<std::vector<uint8_t>> data, pose pos, float height, float width) : _data(data), _pose(pos), _height(height), _width(width), _res(0.05)
{ }

//take a pose in grid coordinates, discretize it into grid cell and returns the state of the cell
//returns 0 if space is empty and 1 if occupied
int OccGrid::isOccupied(pose pos){
    //discretize the pos into grid cell
    int x = pos.x/_res;
    int y = pos.y/_res;
    
    if (x<0 || y<0)
        return 0;
    // return 0 if space is empty and 1 if occupied
    return _data[x][y] == 0 ? 0 : 1;
}