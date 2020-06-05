# Exhaustive Search Scan Matching
This program estimates the pose of a rangefinder by exhaustively matching the data from rangefinder for every possible pose with the provided occupancy grid. The rangefinder data is rotated and translated for every possible pose within the pose tolerance, and then matched with the provided occupancy grid. The pose, for which the transformed range data matches the best with the occupancy grid, is condidered the best pose estimate. 

# Build/installation
You can use cmake to build this program by following the instructions:
1. Clone the repository by using "git clone https://github.com/sukhrajklair/ExhaustiveSearchScanMatching.git" from command terminal
2. Go to the project directory
3. Issue "make build" command to build the executable
4. Use "build/matcher" to runt the program

# Implementation Details
The program has been constructed by sufficiently abrasting away the data and logic in three classes:
1. RangeFinder
2. OccGrid
3. Matcher

## RangeFinder API
1. RangeFinder(): default constructor, intializes field of view to 1 rad, resolution to 0.05 rad, max range to 10m and data representing no obstruction (range > max range)
2. RangeFinder(float fov, float res, float maxRange): intializes the fov, res and maxrange to the provided values, and data representing no obstruction (range > max range)
3. RangeFinder::getFov: returns field of view
4. RangeFinder::getRes: returns the resolution
5. RangeFinder::getmaxRange:: returns the max range
6. RangeFinder::getData: returns the range data
7. RangeFinder::setData: set the range data

## OOccGrid API
Struct pose is defined to abstract pose data: pose{x, y, theta}
1. OccGrid(): initializes the pose to {0, 0, 0}, height and width to 10, cell resolution to 0.05, and data to represent empty grid
2. OccGrid(std::vector<std::vector<uint8_t>> data, pose pos, float height, float width): Initializes all of the member variables using the values passed to the constructor, and set resolution equal to 0.05
3. getPose(): returns the pose
4. int isOccupied(pose pos): accepts a pose argument, discretize it to a cell in the grid and returns 0 if cell is empty and 1 if cell is occupied

## Matcher API
1. Matcher(std::unique_ptr<OccGrid> grid, std::unique_ptr<RangeFinder> rangeData, pose worldPose, pose poseTolerance, float linearSearchRes, float angularSearchRes): constructor takes grid and range data as unique pointer and other paramters are passed by value. Matcher class will own the grid and range data object pointers. These are created on heap and then moved to the Matcher object. 
2. getGridPose(): returns the pose of the rangefinder relative to the grid
3. receiveRangeData(): receive new range data. RangeFinder object is only created once. The new data coming from the rangefinder is then passed to the Matcher object which owns the RangeFinder object. This allows a continuous scan matching by adding new data coming in from the range finder. This also allows to easily wrap this code in a ROS node and use this method as handler for new data coming in on rangefinder topic.
4. match(): matches the range data with occupancy grid and returns the best estimated pose.