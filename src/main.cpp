#include<eigen3/Eigen/Geometry>

#include <iostream>
#include <chrono>

#include <math.h>
#include "octree.h"


using namespace Eigen;
using namespace std;

typedef octree::node nodeType;


int main(int argc, char** argv) {
    int nodeNumber = 10;
    float dataOverhead = 2.0;
    int bufferNum = (int)(nodeNumber*dataOverhead);
    int nodeBuffer2[bufferNum];
    Eigen::Vector3f centre(0,0,0);
    float range = 1.0;

    cout << "size of int" << sizeof(octree::node)<< endl;
    cout << "size of int" << sizeof(octree::node::dataBelow)<< endl;
    cout << "size of int" << sizeof(octree::node::nodeInfo)<< endl;
    cout << "size of int" << sizeof(octree::node::parent)<< endl;
    cout << "size of int" << sizeof(octree::node::pointers)<< endl;
    cout << "size of int" << sizeof(nodeBuffer2)<< endl;
    cout << (int)(nodeNumber*dataOverhead) * sizeof(nodeType) << endl;
    nodeType nodeBuffer[(int)(nodeNumber*dataOverhead)];
    int dataBuffer[nodeNumber];

    octree myOct(32,2,centre,range,&nodeBuffer[0],(int)(nodeNumber*dataOverhead));

    //tests
    octree::bounds bound;
    bound.max  = Vector3f(1,1,1);
    bound.min = Vector3f(0,0,0);

    //test octant division.
    if(bound.whichDir(Vector3f(0.75,0.75,0.75)) != 0) cout << "error0!" << endl;
    if(bound.whichDir(Vector3f(0.75,0.25,0.75)) != 1) cout << "error1!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.25,0.75)) != 2) cout << "error2!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.75,0.75)) != 3) cout << "error3!" << endl;

    if(bound.whichDir(Vector3f(0.75,0.75,0.25)) != 4) cout << "error4!" << endl;
    if(bound.whichDir(Vector3f(0.75,0.25,0.25)) != 5) cout << "error5!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.25,0.25)) != 6) cout << "error6!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.75,0.25)) != 7) cout << "error7!" << endl;

    // test bounding cube scaling (simple)
    bound.downScale(1);
    bound.upScale(1);
    if(bound.max !=Vector3f(1,1,1) && bound.min != Vector3f(0,0,0)) cout << "error upscale and downscale" << endl;

    //test insertion and N-closest, all insertions are given data that is incrementing, and a point that is increasingly far (at random direction) from the test point
    // therefore we know that we must get the elements back in order.
    float rad = 1.0/nodeNumber;
    float radInc = rad*0.7;
    for(int i = 0; i < nodeNumber; i++){
        //Eigen::Vector3f randVec(1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX);
        Eigen::Vector3f randVec(2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX);
        randVec -= Eigen::Vector3f(1,1,1);
        randVec.normalize();
        dataBuffer[i] = i+1;
        octree::dataPtr data;
        data.data = dataBuffer[i];
        data.point = rad*randVec + centre;
        int depth = myOct.insert(data);
        rad += radInc;

    }
    int toFind = nodeNumber / 2;
    std::vector<octree::dataPtr> list =  myOct.getNnearest(centre,toFind);
    list =  myOct.getNnearest(centre,toFind);
    for(int i = 0; i < toFind; i++){
        if(list[i].data != i+1) cout << "wrong order!" << endl;
    }


    //test removals
    nodeType nodeBuffer3[(int)(nodeNumber*dataOverhead)];
    octree myOct2(32,2,centre,range,&nodeBuffer3[0],(int)(nodeNumber*dataOverhead));
    Eigen::Vector3f randVec(2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX,2.0*(float)rand()/RAND_MAX);
    randVec -= Eigen::Vector3f(1,1,1);
    randVec.normalize();
    Eigen::Vector3f curPos = -0.9*randVec;
    float posInc = 0.9*2.0/nodeNumber;
    for(int i = 0; i < nodeNumber; i++){
        curPos += posInc * randVec;
        octree::dataPtr data;
        data.data = i+1;
        data.point = curPos;
        myOct2.insert(data);
    }
    curPos = -0.9*randVec;
    for(int i = 0; i < nodeNumber; i++){
        curPos += posInc * randVec;
        octree::dataPtr data;
        data.data = i+1;
        data.point = curPos;
        myOct2.remove(data);
    }
    cout << "finished";
    cout << "finished";


}

