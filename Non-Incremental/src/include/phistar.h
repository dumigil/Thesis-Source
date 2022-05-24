#ifndef PHISTAR_H
#define PHISTAR_H
#include "Magnum/Magnum.h"
#include <Magnum/GL/Mesh.h>
#include <map>
#include "Voxel.h"
#include "VoxelGrid.h"
#include <mutex>
#include <queue>
#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Color.h>
#include "Simdata.h"
#include "Node.h"
#include "Util.h"
#include <chrono>

using namespace Magnum;
using namespace Math::Literals;


void phi_star_search_grid(VoxelGrid &voxels, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                          const float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                          int sim_num);

void phi_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                         float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                         int sim_num);

bool lineOfSight_grid(VoxelGrid &voxels, Node &start, Node &end, float voxelsize);

static int heuristic(Voxel &a, Voxel&b);

static int heuristic(Node *a, Node*b);

void initializeVertex(Node *node);

void updateVertex(Node *current, Node *next, Node *goal, VoxelGrid &voxels, float voxelsize, PriorityQueue<Node*, float> &queue, int voxelID);

void computeCost(Node *current, Node *next, VoxelGrid &voxels, float voxelsize, int voxelID);

std::vector<Node> raytrace(Node *n0, Node *n1, VoxelGrid & voxels, int voxelID);

bool lineOfSight(std::vector<Node*> &line, VoxelGrid &voxels, int voxelID);

std::vector<Node*> reconstruct_path(Node *start,Node *goal, std::map<Node*, Node*> &came_from, std::mutex &mtx);

std::vector<Node*> raytrace3d(Node *n0, Node *n1, VoxelGrid & voxels, int voxelID);

static void push_to_buffer(OctreeNode *nodes, Color3 &pathcol, int id, std::mutex &mtx, float voxelsize, std::vector<InstanceData> &_instanceData);

static void push_to_buffer(Node *nodes, Color3 &pathcol, int id, std::mutex &mtx, float voxelsize, std::vector<InstanceData> &_instanceData);

std::vector<OctreeNode*> raytrace3d(OctreeNode *n0, OctreeNode *n1, SparseVoxelOctree &octree, int voxelID);

std::vector<OctreeNode*> reconstruct_path(OctreeNode *start,OctreeNode *goal, std::map<OctreeNode*, OctreeNode*> &came_from, std::mutex &mtx);

bool lineOfSight(std::vector<OctreeNode*> &line, SparseVoxelOctree &octree, int voxelID);

bool initializeVertex(OctreeNode *node);

void updateVertex(OctreeNode *current, OctreeNode *next, OctreeNode *goal, SparseVoxelOctree &octree, float voxelsize, PriorityQueue<OctreeNode*, float> &queue, int voxelID);

void computeCost(OctreeNode *current, OctreeNode *next, SparseVoxelOctree &octree, float voxelsize, int voxelID);

static bool pathChanged(std::vector<OctreeNode*> &path, std::mutex &mtx);

#endif /* phistar_h */