#pragma once
#include <algorithm>
#include <unordered_map>
//#include "Util.h"
#include "../libmorton/include/libmorton/morton.h"



struct OctreeNode{
    OctreeNode *Children[8];
    OctreeNode *Parent;
    OctreeNode *PhiParent;
    uint_fast64_t index;
    uint_fast16_t attribute, level, x, y, z, h;
    float g;
    bool isLeaf = false;
    bool isFull = false;
    bool isChanged = false;
    bool isOccupied = false;


    OctreeNode(){
        for(auto &a: Children){
            a= nullptr;
        }
        Parent = nullptr;
        index = 0;
    }

    OctreeNode(uint_fast16_t x,uint_fast16_t y,uint_fast16_t z){
        for(auto &a: Children){
            a= nullptr;
        }
        Parent = nullptr;
        this->x = x;
        this->y = y;
        this->z = z;
        index = libmorton::morton3D_64_encode(x, y, z);

    }

    bool operator<(const OctreeNode &other) const {
        return std::tie(index) < std::tie(other.index);
    }

    bool operator==(const OctreeNode &other) const {
        return (x ==other.x && y == other.y && z == other.z);
    }

    bool isEmpty() const {
        bool empty = false;
        for(auto &child: Children){
            if(child != nullptr) empty = true;
        }
        return empty;
    }

    bool getFull() {
        bool full = true;
        for(auto &child: Children){
            if(child == nullptr) full = false;
        }
        return full;
    }

    OctreeNode* getChild() {
        for(auto &child: Children){
            if(child != nullptr) return child;
        }
        return nullptr;
    }



    ~OctreeNode() = default;
};

struct SparseVoxelOctree {
    std::vector<std::map<uint_fast64_t, OctreeNode *>> mTree;
    std::vector<std::unordered_map<uint_fast64_t , OctreeNode *>> mQTree;
    SparseVoxelOctree()= default;

    ~SparseVoxelOctree() = default;

    unsigned int getDepth() const {
        return mTree.size();
    }



};

