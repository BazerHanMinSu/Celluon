#include "stdafx.h"
#include "optimalRect.h"

GeomEdge optimalRect::findEdge(int x, bool isTop, std::vector<GeomEdge> l)
{
    GeomEdge e,emax=l[0];
    //int count = 0;
    for (int i=0; i<l.size(); i++){
        e = l[i];
        if (e.xmin == x){
            //count++;
            //if (count == 1){
            //    emax = e;
            //}
            //else{
            if (e.xmax != e.xmin){
                if ((e.isTop && isTop)||(!e.isTop && !isTop)){
                    emax = e;
                }
            }
        }
            
    }
    return emax;
}

int optimalRect::xIntersect(int y, std::vector<GeomEdge> l)
{
    int x=0;
    double x0=0, x1=0;
    for(int i=0; i<mPointList.size(); i++){
        GeomEdge e = l[i];
        if (e.isRight && e.ymin <= y && e.ymax >= y){
            x0 = (double)(y+0.5 - e.b)/e.m;
            x1 = (double)(y-0.5 - e.b)/e.m;
        }
    }
    x = (int)std::floor(std::min(x0,x1));
    return x;
}

int optimalRect::yIntersect(int xi,GeomEdge e)
{
    int y;
    double yfirst = (e.m) * (xi-0.5) + e.b;
    double ylast  = (e.m) * (xi+0.5) + e.b;
        
    if (!e.isTop){
        y = (int)std::floor(std::min(yfirst, ylast));
    }
    else {
        y = (int)std::ceil(std::max(yfirst, ylast));
    }
    return y;
}

void optimalRect::computeEdgeList()
{
    mEdgeList.clear();

    cv::Point a,b;
    GeomEdge e;

    a = mPointList[mPointList.size()-1];
    int i;

    for(i=0; i<mPointList.size(); i++){
       b = mPointList[i];
       //b = (GeomPoint)this.elementAt(i+1);
            
       if (i==0){
           xmin = a.x;
           xmax = a.x;
           ymin = a.y;
           ymax = a.y;
       }
       else{
           if (a.x < xmin){
               xmin = a.x;
           }
           if (a.x > xmax){
               xmax  = a.x;
               yxmax = a.y;
           }
           if (a.y < ymin){
               ymin = a.y;
           }
           if (a.y > ymax){
               ymax  = a.y;
           }
       }
       e = GeomEdge(a,b);
       mEdgeList.push_back(e);
       a = b;
    } //for
    // b = (GeomPoint)this.elementAt(this.size()-1);
    // a = (GeomPoint)this.elementAt(0);
    // e = new GeomEdge(b,a);
    // l.add(e);
}

cv::Rect optimalRect::computeLargestRectangle()
{
    cv::Rect resultRect;

    computeEdgeList();

    GeomEdge top, bottom;
    int local_ymax, local_ymin, xright, xlo, xhi;
    int area, maxArea = 0;

    int maxAreaAC=0, maxAreaBD=0, maxAreaABC=0, maxAreaABD=0, maxAreaACD=0, maxAreaBCD=0;
    int width, height, maxh=0, maxw=0;
      
    /* all 2-corner and 3-corner largest rectangles */
    int aAC=0,aBD=0,aABC=0,aABD=0,aACD=0,aBCD=0;
    cv::Point pAC, pBD, pABC, pABD, pACD, pBCD;
    int hAC=0,wAC=0,hBD=0,wBD=0,hABC=0,wABC=0,hABD=0,wABD=0,hACD=0,wACD=0,hBCD=0,wBCD=0;
    bool onA, onB, onC, onD;

    int i;
    cv::Point maxp = cv::Point(0,0);
    pAC = maxp; pBD = maxp; pABC = maxp; pABD = maxp; pACD = maxp; pBCD = maxp;
        
    std::vector<cv::Point> xint;
        
    for(i=0;i< ymax;i++){
        int x = xIntersect(i,mEdgeList);
        cv::Point px = cv::Point(x,i);
        xint.push_back(px);
    }

    //find first top and bottom edges
    top = findEdge(xmin, true, mEdgeList);
    bottom = findEdge(xmin, false, mEdgeList);

    //scan for rectangle left position
    for(int xi=xmin; xi<xmax;xi++)
    {
        local_ymin = yIntersect(xi, top);
        local_ymax = yIntersect(xi, bottom);

        for(int ylo = local_ymax;ylo>=local_ymin;ylo--){//ylo from to to bottom
                
            for(int yhi = local_ymin; yhi<=local_ymax; yhi++){
                
                if (yhi>ylo){
                        
                    onA = (yhi == local_ymax && !bottom.isRight);
                    onD = (ylo == local_ymin && !top.isRight);
                        
                    xlo = xint[ylo].x;//xIntersect(ylo,edgeList);
                    xhi = xint[yhi].x;//xIntersect(yhi,edgeList);
                        
                    xright = std::min(xlo,xhi);
                    onC = (xright == xlo && yxmax >= ylo);
                    onB = (xright == xhi && yxmax <= yhi);
                        
                    height = yhi-ylo;
                    width = xright - xi;
                            
                    if (!fixed){                                                      
                    }//!fixed
                    else{
                        int fixedWidth = (int)std::ceil( ((double) height*fixedX)/((double)fixedY));
                        if (fixedWidth <= width){
                            width = fixedWidth;
                        }
                        else{
                            width = 0;
                        }
                    }
                    area = width * height;
                    //AC 
                    if (onA && onC && !onB && !onD){                            
                        if (area > aAC){
                            aAC = area;
                            pAC = cv::Point(xi, ylo);
                            hAC = height;
                            wAC = width;
                        }
                    }
                    //BD
                    if (onB && onD && !onA && !onC){
                        if (area > aBD){
                            aBD = area;
                            pBD = cv::Point(xi, ylo);
                            hBD = height;
                            wBD = width;
                        }
                    }
                    //ABC
                    if (onA && onB && onC){
                        if (area > aABC){
                            aABC = area;
                            pABC = cv::Point(xi, ylo);
                            hABC = height;
                            wABC = width;
                        }
                    }
                    //ABD
                    if (onA && onB && onD){
                        if (area > aABD){
                            aABD = area;
                            pABD = cv::Point(xi, ylo);
                            hABD = height;
                            wABD = width;
                        }
                    }
                    //ACD
                    if (onA && onC && onD){
                        if (area > aACD){
                            aACD = area;
                            pACD = cv::Point(xi, ylo);
                            hACD = height;
                            wACD = width;
                        }
                    }
                    //BCD
                    if (onB && onC && onD){
                        if (area > aBCD){
                            aBCD = area;
                            pBCD = cv::Point(xi, ylo);
                            hBCD = height;
                            wBCD = width;
                        }
                    }
                        
                    if(area>maxArea){
                        maxArea = area;
                        maxp = cv::Point(xi, ylo);
                        maxw = width;
                        maxh = height;
                        // System.out.println(onA + " " + onB + " " + onC + " " + onD);
                    }
                }//yhi > ylo
            }//for yhi
        }//for ylo
        if (xi == top.xmax){
            top = findEdge(xi,  true, mEdgeList);
        }
        if(xi == bottom.xmax){
            bottom = findEdge(xi, false, mEdgeList);
        }
    }// xi

    resultRect = cv::Rect(maxp.x,maxp.y,maxw,maxh);

    return resultRect;
}