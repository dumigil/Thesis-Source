#ifndef DLITE_H
#define DLITE_H
#include "Magnum/Magnum.h"
#include <Magnum/GL/Mesh.h>
#include <map>
#include "Voxel.h"
#include <mutex>
#include <queue>
#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Color.h>
#include "Simdata.h"
#include "Node.h"
#include "Util.h"
#include <set>
#include <unordered_set>
#include <fstream>
#include <iostream>


void d_star_search_grid(DynamicGrid &dynamicGrid, std::mutex &mutex,std::vector<InstanceData> &_instanceData, float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental);

void d_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData, float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental);

void updateVertex(Node* u, Node* goal,Node* start, PriorityQueue<Node*, std::pair<double, double>> &frontier, DynamicGrid &dynamicGrid, int id,std::set<Node*> &visited, float k_m, std::unordered_set<Node*> &openList, std::ofstream &out);

void computeShortestPath(PriorityQueue<Node*, std::pair<double, double>> &frontier,Node* start, Node* goal, DynamicGrid &dynamicGrid, int id,std::set<Node*> &visited, float k_m, std::unordered_set<Node*> &openList, std::ofstream &out);

std::pair<double, double> calculateKey(Node* current, Node* start, float k_m);

bool checkVoxelStatus(Node* current, DynamicGrid &dynamicGrid, int id);

std::vector<Node*> reconstructPath(std::set<Node*> &visited, Node* goal, Node* start);

std::vector<Node*> getSucc(Node *current, DynamicGrid &dynamicGrid, int voxelID);
#endif /* dlite_h */