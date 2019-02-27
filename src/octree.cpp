#include "octree.h"

octree::octree(int maxLevels, int batching, Eigen::Vector3f centrePoint, float edgeLength, node* nodeBuffer, int nodes){
    levels = maxLevels;
    batch = batching;
    centre = centrePoint;
    range = edgeLength;
    nodeMem = nodeBuffer;
    totalNodes = nodes;
    freeNodeStackPtr = 1;
    currentDepth = 1;
    freeNodeCount = nodes - 1;
    //initialise root node
    head = &nodeMem[0];
    head->parent = NULL;
    initLeaf(head);
    scrapHead = &nodeMem[1];
    node* scrap = scrapHead;
    for(int i = 2; i < nodes; i++){
        scrap->pointers.nodes[0] = &nodeMem[i];
        scrap = scrap->pointers.nodes[0];
    }
}


int octree::insert(node* aNode,dataPtr data, bounds bound,int depth){
    aNode->dataBelow ++;
    int retDepth = depth;
    if(isLeaf(aNode)){
        if(aNode->pointers.data.data[0].data == NULL) aNode->pointers.data.data[0] = data;
        else if(aNode->pointers.data.data[1].data == NULL) aNode->pointers.data.data[1] = data;
        else { //we used both slots we should expand the tree.
            //convert node from leaf to normal, store data and reinsert it lower down.
            aNode->nodeInfo = aNode->nodeInfo & (~ ISLEAF);
            dataPtr A = aNode->pointers.data.data[0];
            dataPtr B = aNode->pointers.data.data[1];
            //clear data/pointers ready for branching.
            for(int i = 0; i < 8; i++){
                aNode->pointers.nodes[i] = NULL;
            }
            int Adir = bound.whichDir(A.point);
            int Bdir = bound.whichDir(B.point);
            if(Adir == Bdir){
                aNode->pointers.nodes[Adir] = allocateNode(aNode);
                aNode->pointers.nodes[Adir]->pointers.data.data[0] = A;
                aNode->pointers.nodes[Adir]->pointers.data.data[1] = B;
                aNode->pointers.nodes[Adir]->dataBelow = 2;
            }
            else{
                aNode->pointers.nodes[Adir] = allocateNode(aNode);
                aNode->pointers.nodes[Bdir] = allocateNode(aNode);
                aNode->pointers.nodes[Adir]->pointers.data.data[0] = A;
                aNode->pointers.nodes[Adir]->dataBelow = 1;
                aNode->pointers.nodes[Bdir]->pointers.data.data[0] = B;
                aNode->pointers.nodes[Bdir]->dataBelow = 1;
            }
            //now insert the data that is new.
            int dataDir = bound.whichDir(data.point);
            if(aNode->pointers.nodes[dataDir] == NULL) aNode->pointers.nodes[dataDir] = allocateNode(aNode);
            bound.downScale(dataDir);
            return retDepth = insert(aNode->pointers.nodes[dataDir], data, bound, depth+1);
        }
        return retDepth;

    }
    else{
        //now insert the data that is new.
        int dataDir = bound.whichDir(data.point);
        if(aNode->pointers.nodes[dataDir] == NULL) aNode->pointers.nodes[dataDir] = allocateNode(aNode);
        bound.downScale(dataDir);
        return retDepth = insert(aNode->pointers.nodes[dataDir], data, bound,depth+1);
    }
}
int octree::insert(dataPtr data){
    bounds bound;
    bound.max = centre + Eigen::Vector3f(range,range,range);
    bound.min = centre - Eigen::Vector3f(range,range,range);
    int depthInserted = insert(head,data,bound,1);
    return depthInserted;
}

void octree::initLeaf(node* aNode){
    aNode->nodeInfo = head->nodeInfo | ISLEAF;
    for(int i = 0; i < 8; i++){
        aNode->pointers.nodes[i] = NULL;
    }
    aNode->dataBelow = 0;
}


int octree::remove(dataPtr data ){

}


int octree::redistribute(node* aNode){

}


int octree::compact(node* aNode){

}


int octree::cullNode(node* aNode){

}


std::vector<octree::dataPtr> getNnearest(Eigen::Vector3f point, int N){

}

//functions related to bounds
// We use ROS coordinates as a standard, Z up, X forward, Y left
void octree::bounds::downScale(int direction){
    switch (direction){
    case 0: // +x +y +z
        // max is left unchanged
        min = min + (max - min)/2.0;
        break;

    case 1:// +x -y +z
        min[0] = half(0);
        max[1] = half(1);
        min[2] = half(2);
        break;

    case 2:// -x -y +z
        max[0] = half(0);
        max[1] = half(1);
        min[2] = half(2);
        break;

    case 3:// -x +y +z
        max[0] = half(0);
        min[1] = half(1);
        min[2] = half(2);
        break;

    case 4:// +x +y -z
        min[0] = half(0);
        min[1] = half(1);
        max[2] = half(2);
        break;

    case 5:// +x -y -z
        min[0] = half(0);
        max[1] = half(1);
        max[2] = half(2);
        break;

    case 6:// -x -y -z
        max[0] = half(0);
        max[1] = half(1);
        max[2] = half(2);
        break;

    case 7:// -x +y -z
        max[0] = half(0);
        min[1] = half(1);
        max[2] = half(2);
        break;
    }
}

int octree::bounds::whichDir(Eigen::Vector3f point){
    Eigen::Vector3f mid = min + (max - min)/2.0;
    if(point[0] >= mid[0] && point[1] >= mid[1] && point[2] >= mid[2]) return 0;

    if(point[0] >= mid[0] && point[1] < mid[1] && point[2] >= mid[2]) return 1;

    if(point[0] < mid[0] && point[1] < mid[1] && point[2] >= mid[2]) return 2;

    if(point[0] < mid[0] && point[1] >= mid[1] && point[2] >= mid[2]) return 3;

    if(point[0] >= mid[0] && point[1] >= mid[1] && point[2] < mid[2]) return 4;

    if(point[0] >= mid[0] && point[1] < mid[1] && point[2] < mid[2]) return 5;

    if(point[0] < mid[0] && point[1] < mid[1] && point[2] < mid[2]) return 6;

    if(point[0] < mid[0] && point[1] >= mid[1] && point[2] < mid[2]) return 7;

    return -1;
}

void octree::bounds::upScale(int dir){
    switch (dir){
    case 0: // +x +y +z
        // max is left unchanged
        min[0] = extendBack(0);
        min[1] = extendBack(1);
        min[2] = extendBack(2);
        break;

    case 1:// +x -y +z
        min[0] = extendBack(0);
        max[1] = extendForward(1);
        min[2] = extendBack(2);
        break;

    case 2:// -x -y +z
        max[0] = extendForward(0);
        max[1] = extendForward(1);
        min[2] = extendBack(2);
        break;

    case 3:// -x +y +z
        max[0] = extendForward(0);
        min[1] = extendBack(1);
        min[2] = extendBack(2);
        break;

    case 4:// +x +y -z
        min[0] = extendBack(0);
        min[1] = extendBack(1);
        max[2] = extendForward(2);
        break;

    case 5:// +x -y -z
        min[0] = extendBack(0);
        max[1] = extendForward(1);
        max[2] = extendForward(2);
        break;

    case 6:// -x -y -z
        max[0] = extendForward(0);
        max[1] = extendForward(1);
        max[2] = extendForward(2);
        break;

    case 7:// -x +y -z
        max[0] = extendForward(0);
        min[1] = extendBack(1);
        max[2] = extendForward(2);
        break;
    }
}

void octree::bounds::upScale(node* aNode){
    if(aNode->parent == NULL) return;
    node* parent = aNode->parent;
    int direction;
    for(direction = 0; direction < 8; direction++){
        if(parent->pointers.nodes[direction] == aNode) break;
    }
    upScale(direction);


}

float octree::bounds::half(int dir){
    return min[dir] + (max[dir] - min[dir])/ 2.0;
}

float octree::bounds::extendBack(int dir){
    return  max[dir] + 2.0*(min[dir] - max[dir]);
}
float octree::bounds::extendForward(int dir){
    return  2.0*max[dir]  - min[dir];
}

octree::node* octree::allocateNode(node* parentIn){
    node* aNode = scrapHead;
    scrapHead = scrapHead->pointers.nodes[0];
    freeNodeCount --;
    //clean up new node;
    initLeaf(aNode);
    aNode->parent = parentIn;
    return aNode;
}

void octree::deallocateNode(node* aNode){
    aNode->pointers.nodes[0] = scrapHead;
    scrapHead = aNode;
    freeNodeCount ++;
    return;
}
