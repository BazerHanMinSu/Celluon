#ifndef OPTIMAL_RECT_H_
#define OPTIMAL_RECT_H_

#include <iostream>
#include <opencv/cv.h>


class GeomEdge {
    public:

    int xmin, xmax; /* horiz, +x is right */
    int ymin, ymax; /* vertical, +y is down */
    double m,b; /* y = mx + b */
    bool isTop, isRight; /* position of edge w.r.t. hull */
    
    GeomEdge(){};

    GeomEdge(cv::Point p, cv::Point q){
        xmin = std::min(p.x, q.x);
        xmax = std::max(p.x, q.x);
        ymin = std::min(p.y, q.y);
        ymax = std::max(p.y, q.y);
        m = ((double)(q.y-p.y))/((double)(q.x-p.x));
        b = p.y - m*(p.x);
        isTop = p.x > q.x; //edge from right to left (ccw)
        isRight = p.y > q.y; //edge from bottom to top (ccw)
    }
};

class optimalRect{
public:
    ///
    optimalRect(){
        fixedX = 16;
        fixedY = 9;
        fixed = true;
    }

    /// add point
    void addPoint(cv::Point pt){
        mPointList.push_back(pt);
    }
    
    // comput edge list
    void computeEdgeList();
    
    // compute Largest Rectangle
    cv::Rect computeLargestRectangle();

    int xIntersect(int y, std::vector<GeomEdge> l);
    int yIntersect(int xi,GeomEdge e);

    GeomEdge findEdge(int x, bool isTop, std::vector<GeomEdge> l);
protected:
    std::vector<cv::Point> mPointList;
    std::vector<GeomEdge>  mEdgeList;

    int xmin, xmax; /* horiz, +x is right */
    int ymin, ymax; /* vertical, +y is down */
    int yxmax; //y coord of xmax

    int fixedX;
    int fixedY;
    bool fixed;
};

#endif