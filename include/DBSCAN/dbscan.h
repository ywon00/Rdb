#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

struct Point
{
    float x, y;  // X, Y, Z position
    int clusterID;  // clustered ID
};

class Dbscan{
public:    
    Dbscan(unsigned int minPts, float eps);
    ~Dbscan(){}

	void dbscan(vector<Point>& points);
    vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
    
private:    
    vector<Point> m_points;

    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif // DBSCAN_H
