#ifndef ASTAR_H
#define ASTAR_H
#include "VoxelGrid.h"
#include "Voxel.h"
#include <map>
#include <set>
#include <unordered_set>
#include "Magnum/Magnum.h"
#include <Magnum/GL/Mesh.h>
#include <map>
#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Color.h>
#include <mutex>
#include <queue>
#include <thread>
#include "Util.h"
#include "Octree.h"


using namespace Magnum;
using namespace Math::Literals;

static int heuristicOctree(OctreeNode * a, OctreeNode * b);

static int heuristic(Voxel &a, Voxel &b);

static int heuristic(uint_fast64_t a, uint_fast64_t b);

static int heuristic(OctreeNode *a, OctreeNode *b);

bool isID(InstanceData instance, int id);

void getBounds(Voxel& a, Voxel &b, int &min_x, int &max_x, int &min_y, int &max_y,int &min_z, int &max_z);

std::vector<Voxel> reconstruct_path(Voxel start, Voxel goal, std::map<Voxel, Voxel> &came_from);

static std::vector<OctreeNode *> reconstruct_path_SVO(OctreeNode* start, OctreeNode* goal, std::map<OctreeNode*,OctreeNode*> &came_from);

void a_star_search_grid(VoxelGrid &voxels, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                        const float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                        int sim_num);

void a_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                       float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental, int sim_num);

static void push_to_buffer(OctreeNode *vx, Color3 &pathcol, int voxelID, std::mutex &mtx, float vSize, std::vector<InstanceData> &_instanceData);

static void push_to_buffer(Voxel &vx, Color3 &pathcol, int voxelID, std::mutex &mtx, float vSize, std::vector<InstanceData> &_instanceData);


static bool pathChanged(std::vector<OctreeNode*> &path, std::mutex &mtx);




#endif /* astar_h */