#include<eigen3/Eigen/Geometry>

#include <iostream>
#include <chrono>

#include <math.h>
#include "octree.h"


using namespace Eigen;
using namespace std;

typedef octree::node nodeType;


int main(int argc, char** argv) {
    int nodeNumber = 1000000;
        float dataOverhead = 1.05;
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
    if(bound.whichDir(Vector3f(0.75,0.75,0.75)) != 0) cout << "error0!" << endl;
    if(bound.whichDir(Vector3f(0.75,0.25,0.75)) != 1) cout << "error1!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.25,0.75)) != 2) cout << "error2!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.75,0.75)) != 3) cout << "error3!" << endl;

    if(bound.whichDir(Vector3f(0.75,0.75,0.25)) != 4) cout << "error4!" << endl;
    if(bound.whichDir(Vector3f(0.75,0.25,0.25)) != 5) cout << "error5!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.25,0.25)) != 6) cout << "error6!" << endl;
    if(bound.whichDir(Vector3f(0.25,0.75,0.25)) != 7) cout << "error7!" << endl;

    bound.downScale(1);
    bound.upScale(1);
    if(bound.max !=Vector3f(1,1,1) && bound.min != Vector3f(0,0,0)) cout << "error upscale and downscale" << endl;

    for(int i = 0; i < nodeNumber; i++){
        //Eigen::Vector3f randVec(1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX);
        Eigen::Vector3f randVec(1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX,1.0*(float)rand()/RAND_MAX);
        dataBuffer[i] = i;
        octree::dataPtr data;
        data.data = dataBuffer[i];
        data.point = randVec;
        int depth = myOct.insert(data);
        cout << depth << endl;

    }
    cout << "finished";
    cout << "finished";


}

