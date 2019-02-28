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

    topLevelBound.max = centre + Eigen::Vector3f(range,range,range);
    topLevelBound.min = centre - Eigen::Vector3f(range,range,range);
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
    bounds bound= defaultBounds();
    int depthInserted = insert(head,data,bound,1);
    return depthInserted;
}

void octree::initLeaf(node* aNode){
    aNode->nodeInfo  = 0;
    aNode->nodeInfo = head->nodeInfo | ISLEAF;
    for(int i = 0; i < 8; i++){
        aNode->pointers.nodes[i] = NULL;
    }
    aNode->dataBelow = 0;
}


bool octree::remove(dataPtr data ){
    node* aNode = getNode(data);
    if(aNode != NULL){
        int countDec = 0;
        if(aNode->pointers.data.data[0] == data){
            aNode->pointers.data.data[0].data = NULL;
            countDec ++;
        }
        if(aNode->pointers.data.data[1]  == data){
            aNode->pointers.data.data[1].data = NULL;
            countDec ++;
        }
        iterateDecCount(aNode,countDec);
        compact(aNode->parent);
        return true;
    }
    return false;
}

void octree::iterateDecCount(node* aNode, int countDec){
    if(aNode != NULL){
        aNode->dataBelow -= countDec;
        iterateDecCount(aNode->parent,countDec);
    }
    return;
}

octree::node*  octree::getNode(dataPtr data){
    bounds bound = defaultBounds();
    return getNode(head,bound,data);
}

octree::node*  octree::getNode(node* aNode,bounds &bound,dataPtr data){
    if(isLeaf(aNode)){
        if(aNode->pointers.data.data[0] == data){
            return aNode;
        }
        if(aNode->pointers.data.data[1]  == data){
            return aNode;
        }
        return NULL;
    }
    else{
        int dir = bound.whichDir(data.point);
        bound.downScale(dir);
        return getNode(aNode->pointers.nodes[dir],bound,data);
    }
}



int octree::redistribute(node* aNode){

}

// checks to see if we can remove child notes and move data to this node; only should be called on elements that are one above a leaf level.
// we also rely on the data structure to be correctly maintained, ie compact called whenever a node is removed etc.
int octree::compact(node* aNode){
    if(aNode != NULL){
        if(aNode->dataBelow <=2 && !isLeaf(aNode)){
            dataPtr ptrs[2];
            int ptrsFound = 0;
            for(int i = 0; i < 8; i++){
                if(aNode->pointers.nodes[i] != NULL){
                    if(aNode->pointers.nodes[i]->pointers.data.data[0].data != NULL){
                        ptrs[ptrsFound] = aNode->pointers.nodes[i]->pointers.data.data[0];
                        ptrsFound++;
                    }
                    if(aNode->pointers.nodes[i]->pointers.data.data[1].data != NULL){
                        ptrs[ptrsFound] = aNode->pointers.nodes[i]->pointers.data.data[1];
                        ptrsFound++;
                    }

                    deallocateNode(aNode->pointers.nodes[i]);
                    aNode->pointers.nodes[i]  = NULL;
                    //if(ptrsFound == 2) break;
                }
            }
            initLeaf(aNode);
            aNode->dataBelow = ptrsFound;
            for(int i = 0; i < ptrsFound; i++){
                aNode->pointers.data.data[i] = ptrs[i];
            }


        }
    }
}


int octree::cullNode(node* aNode){

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
    if(point[0] >= mid[0]){
        if(point[1]>=mid[1]){
            if(point[2] >=mid[2]){
                return 0;
            }
            else{
                return 4;
            }
        }
        else{
            if(point[2] >=mid[2]){
                return 1;
            }
            else{
                return 5;
            }
        }
    }
    else{
        if(point[1]>=mid[1]){
            if(point[2] >=mid[2]){
                return 3;
            }
            else{
                return 7;
            }
        }
        else{
            if(point[2] >=mid[2]){
                return 2;
            }
            else{
                return 6;
            }
        }
    }
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

octree::bounds octree::defaultBounds(){
    bounds bound;
    bound.max = topLevelBound.max;
    bound.min = topLevelBound.min;
    return bound;
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

octree::node* octree::getWouldBeNode(node* aNode,Eigen::Vector3f target,octree::bounds &bound){
    if(isLeaf(aNode)){
        return aNode;
    }
    else{
        int dir = bound.whichDir(target);
        bound.downScale(dir);
        return getWouldBeNode(aNode->pointers.nodes[dir],target,bound);
    }
}

Eigen::Vector3f octree::nearestPointOnCube(Eigen::Vector3f point, octree::bounds  bound){
    bool nearX = false;
    bool farX = false;
    bool nearY = false;
    bool farY = false;
    bool nearZ = false;
    bool farZ = false;
    if(point[0] < bound.min[0]) nearX = true;
    if(point[1] < bound.min[1]) nearY = true;
    if(point[2] < bound.min[2]) nearZ = true;
    if(point[0] >= bound.max[0]) farX = true;
    if(point[1] >= bound.max[1]) farY = true;
    if(point[2] >= bound.max[2]) farZ = true;

    Eigen::Vector3f retVec = point;

    if(farX) retVec[0] = bound.max[0];
    if(farY) retVec[1] = bound.max[1];
    if(farZ) retVec[2] = bound.max[2];
    if(nearX) retVec[0] = bound.min[0];
    if(nearY) retVec[1] = bound.min[1];
    if(nearZ) retVec[2] = bound.min[2];

    return retVec;

}


std::vector<octree::dataPtr> octree::getNnearest(Eigen::Vector3f point, int N){
    std::priority_queue<distAndPointer,std::vector<distAndPointer>, std::greater<distAndPointer>  > pq;
    distAndPointer headDistPtr;
    headDistPtr.aNode = head;
    headDistPtr.bound.max = centre + Eigen::Vector3f(range,range,range);
    headDistPtr.bound.min = centre - Eigen::Vector3f(range,range,range);

    headDistPtr.dist = 0;
    pq.push(headDistPtr);
    std::vector<node*> touchedVec;

    std::vector<octree::dataPtr> returnVec;
    while(!pq.empty() && !(returnVec.size() >= N)){
        distAndPointer toConsider = pq.top();
        pq.pop();
        if(isLeaf(toConsider.aNode) && (!hasCheckB(toConsider.aNode) || !hasCheckA(toConsider.aNode))){

            setCheckA(toConsider.aNode);
            setCheckB(toConsider.aNode);
            if(toConsider.aNode->pointers.data.data[0].data != NULL){
                distAndPointer distAndPointA;
                distAndPointA.aNode = toConsider.aNode;
                distAndPointA.bound = toConsider.bound;
               distAndPointA.dist = (distAndPointA.aNode->pointers.data.data[0].point - point).norm();
               pq.push(distAndPointA);
            }
            if(toConsider.aNode->pointers.data.data[1].data != NULL){
                distAndPointer distAndPointB;
                distAndPointB.aNode = toConsider.aNode;
                distAndPointB.bound = toConsider.bound;
               distAndPointB.dist = (distAndPointB.aNode->pointers.data.data[1].point - point).norm();
               pq.push(distAndPointB);
            }
        }
        else if(isLeaf(toConsider.aNode)){


            // work out if it is the left or right one.
            float distA =std::numeric_limits<float>::max();
            if(toConsider.aNode->pointers.data.data[0].data != NULL) distA = (toConsider.aNode->pointers.data.data[0].point - point).norm();

            float distB =std::numeric_limits<float>::max();
            if(toConsider.aNode->pointers.data.data[1].data != NULL) distB = (toConsider.aNode->pointers.data.data[1].point - point).norm();

            if(distA == toConsider.dist){
                returnVec.push_back(toConsider.aNode->pointers.data.data[0]);

            }
            else if(distB == toConsider.dist){
                returnVec.push_back(toConsider.aNode->pointers.data.data[1]);
            }
            touchedVec.push_back(toConsider.aNode);

            continue;
        }
        //this  is an octant node. put the children on the list, with the dist value set to the minimum possible.
        else if(!isLeaf(toConsider.aNode)){
            for(int i = 0; i < 8; i++){
                if(toConsider.aNode->pointers.nodes[i] != NULL){
                    distAndPointer thisDistAndPtr;
                    thisDistAndPtr.bound = toConsider.bound;
                    thisDistAndPtr.bound.downScale(i);
                    thisDistAndPtr.aNode = toConsider.aNode->pointers.nodes[i];
                    thisDistAndPtr.dist = (point - nearestPointOnCube(point, thisDistAndPtr.bound)).norm();
                    pq.push(thisDistAndPtr);
                }
            }
        }
    }
    //tidy up checked markers, only elements in the list or in the final vector should have been tagged.
    while(!pq.empty()){
        clearCheckA(pq.top().aNode);
        clearCheckB(pq.top().aNode);
        pq.pop();
    }
    for(int i = 0; i < touchedVec.size(); i++){
        clearCheckA(touchedVec[i]);
        clearCheckB(touchedVec[i]);
    }
    return returnVec;

}

