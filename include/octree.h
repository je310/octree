#ifndef OCTREE_H
#define OCTREE_H

#include <eigen3/Eigen/Geometry>

class octree{
#define ISLEAF 1<<0


public: class node;
    public: struct dataPtr{
        Eigen::Vector3f point;
        int data;
    };
    struct dataPtrs{
        dataPtr data[2];
    };
public: union pBatch {
    node* nodes[8];
    dataPtrs data;
    pBatch(){new(&data) dataPtrs();}
};
public: class node{

        public:
        char nodeInfo;  // this will define some properties of the octree node. for instance we can change whether this is considered a leaf node.
                        // If it is a leaf node then the pointers act as pointers to data.
        int dataBelow;
        node* parent;
        pBatch pointers;

    };

    class bounds{
    public:
        Eigen::Vector3f min;
        Eigen::Vector3f max;

        void downScale(int direction);
        int whichDir(Eigen::Vector3f pointIn);
        void upScale(node* aNode);
        void upScale(int dir);
        float half(int dir);
        float extendBack(int dir);
        float extendForward(int dir);
    };



    int levels;
    int batch;


    Eigen::Vector3f centre;
    float range;

    int freeNodeStackPtr;
    int freeNodeCount;
    int totalNodes;
    node* nodeMem;
    node* head;
    node* scrapHead;
    int currentDepth;
//    typedef nodeBatch node*[8];







    octree(int maxLevels, int batching, Eigen::Vector3f centrePoint, float edgeLength, node* nodeBuffer, int nodes);
    int insert(node* aNode, dataPtr data, bounds bound, int depth);
    int insert(octree::dataPtr data );
    node* allocateNode(node *parentIn);
    void deallocateNode(node* aNode);
    void initLeaf(node* aNode);
    int remove(dataPtr data);
    bool isLeaf(node* aNode){
        return aNode->nodeInfo & ISLEAF!=0;
    }
    int redistribute(node* aNode);
    int compact(node* aNode);
    int cullNode(node* aNode);
    float spaceLeft() { return (float)freeNodeStackPtr/totalNodes;}

    std::vector<octree::dataPtr> getNnearest(Eigen::Vector3f point, int N);
};






















#endif
