#include <iostream>
#include "include/INE.h"

int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y);

int main(int argc, char **argv) {
    INE ine;

    string strVertexFile;
    //strVertexFile = "Data/Vertexs.txt";
    strVertexFile = "Data/CalNode.txt";
    //cout << "Input the vertex file name" << endl;
    //cin >> strVertexFile;
    ine.readVertexFile(strVertexFile);

    string strRoadEntityFile;
    //strRoadEntityFile = "Data/RoadDatas.txt";
    strRoadEntityFile = "Data/CalRoadData.txt";
    //cout << "Input the relative of road and entity file name" << endl;
    //cin >> strRoadEntityFile;
    ine.readRoadAndEntity(strRoadEntityFile);

    string strQueryPointFile;
    strQueryPointFile = "Data/QueryNode.txt";
    //cout << "Input the query point file" << endl;
    //cin >> strQueryPointFile;

    ofstream ofile("result.txt");

    ifstream ifile(strQueryPointFile);
    string strLine;
    timeval start_tm, end_tm;
    gettimeofday(&start_tm, 0);
    while (getline(ifile, strLine)) {
        vector<string> strBuffs;
        boost::split(strBuffs, strLine, boost::is_any_of(" "));
        Point query_point(stod(strBuffs[1]), stod(strBuffs[2]));
        Entity nn_entity = ine.searchNNOfQueryPoint(query_point);

        ofile << "The nearest neighbor of query point " << stoi(strBuffs[0])
              << " (" << query_point.m_X << "," << query_point.m_Y << ") is"
              << " (" << nn_entity.m_X << "," << nn_entity.m_Y << ")" << endl;

    }
    gettimeofday(&end_tm, 0);
    timeval diff_tm;
    timeval_subtract(&diff_tm,&start_tm,&end_tm);
    cout << "Running time is " << diff_tm.tv_sec*1000+diff_tm.tv_usec/1000 << "ms" << endl;
    ofile.close();
    ifile.close();
}

int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y) {

    if (x->tv_sec > y->tv_sec)
        return -1;

    if ((x->tv_sec == y->tv_sec) && (x->tv_usec > y->tv_usec))
        return -1;

    result->tv_sec = (y->tv_sec - x->tv_sec);
    result->tv_usec = (y->tv_usec - x->tv_usec);

    if (result->tv_usec < 0) {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }

    return result->tv_sec + result->tv_usec / 1000;
}