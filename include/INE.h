#ifndef INE_H
#define INE_H

#include "ClassType.h"

class INE
{
  public:
    INE();
    virtual ~INE();

  public:
    void readVertexFile(string strFileName);
    void readRoadAndEntity(string strFileName);
    Entity searchNNOfQueryPoint(const Point& queryPoint);


  protected:
    Entity getEntity(const Road& road,const double distance);
    Road searchRoadCoverQueryPoint(const Point& queryPoint);
    double calDistanceOf2Point(const Point& point1,const Point& point2);

  private:
    map<int,Vertex> m_Vertexs;
    map<int,Entity> m_Entitys;
    map<int,Road> m_Roads;

    //VV m_VVs[MAX_NUM];

    map<int,map<int,set<Data>>> m_VVs;

};

#endif // INE_H
