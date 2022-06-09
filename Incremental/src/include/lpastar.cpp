#include "lpastar.h"
#include "Simdata.h"
#include <map>
#include <unordered_map>
#include <thread>
#include <fstream>
#include <condition_variable>

/**
 * Performs Incremental A* pathfinding on the Sparse Voxel Morton Grid.
 *
 *
 * @param octree Sparse Voxel Octree with only walkable space as nodes.
 * @param voxels A VoxelGrid object.
 * @param mutex a mutable mutex to handle data access.
 * @param _instanceData instance data owned by Simulator class (shared across threads).
 * @param voxelsize voxel size used to caculate travel time.
 * @param id every thread and path gets a unique ID.
 * @param speed walking speed.
 * @param running atomic boolean owned by Simulator to manage all the running threads.
 * @param incremental boolean to switch between incremental and non incremental mode.
 *
 * @returns Sends path to instance buffer to visualise.
 */
void lpa_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                         float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                         int sim_num) {
    std::string base = "/Users/michieldejong/Documents/Graduation/Simulator_lpa/results/grid/incremental_a_star/result_inc_a_star";
    std::string ext = ".csv";
    auto t = std::chrono::system_clock::now();
    std::time_t ts = std::chrono::system_clock::to_time_t(t);
    std::string t_start = std::to_string(ts);
    std::string threadID = std::to_string(id);
    base+=threadID;
    base+=t_start;
    base+=ext;
    std::condition_variable cv;
    std::ofstream out(base);
    out<<"start, goal, path_length, nodes_visited, time, num_evax\n";

    double hExit = heuristic(simPoints().rooms[id], simPoints().exits[0]);
    auto startIndex = libmorton::morton3D_64_encode(simPoints().rooms[id].x, simPoints().rooms[id].y, simPoints().rooms[id].z);
    auto goalIndex = libmorton::morton3D_64_encode(simPoints().exits[0].x, simPoints().exits[0].y, simPoints().exits[0].z);
    auto start = octree.mTree[0][startIndex];
    auto exits = octree.mTree[0][goalIndex];
    for (auto all: simPoints().exits) {
        double tmp = heuristic(simPoints().rooms[id], all);
        if (tmp < hExit) {
            auto index = libmorton::morton3D_64_encode(all.x, all.y, all.z);
            exits = octree.mTree[0][index];
        } else {
        }
    }

    static float vSize = voxelsize;
    Color3 pathcol = Color3(randomFloat(0, 1), randomFloat(0, 1), randomFloat(0, 1));
    const char* inc;
    int voxelSpeed;
    static uint_fast64_t goal_morton = exits->index;
    static uint_fast64_t start_morton = start->index;
    std::deque<OctreeNode*> path;
    std::vector<OctreeNode *> visited = {};
    bool first = true;
    int voxelID = 11 + id;
    start->isOccupied = true;

    int level = 0;

    while(running) {
        if(pathChanged(path, mutex) || first || !succesfulPath(path)) {
            for(auto all: path){
                if(all->attribute == voxelID) all->attribute=2;
            }
            path.clear();

            std::thread::id this_id = std::this_thread::get_id();
            auto tStart = std::chrono::high_resolution_clock::now();


            std::map<OctreeNode *, OctreeNode *> came_from = {};
            std::map<OctreeNode *, int> cost_so_far = {};
            PriorityQueue<OctreeNode *, int> frontier = {};
            frontier.put(start, 0);
            came_from[start] = start;
            cost_so_far[start] = 0;
            while(!frontier.empty()) {
                auto current = frontier.get();
                if(current->index == goal_morton){
                    break;
                }
                if(current->Parent == nullptr){
                    continue;
                }
                for (const auto &conn: getNeighbours18_morton(current->x, current->y, current->z, current->index)) {
                    if(!octree.mQTree[level].contains(conn)){
                        continue;
                    }
                    auto neighbours = octree.mQTree[0][conn];
                    if(neighbours->attribute == voxelID) {
                        continue;
                    }
                    int new_cost = cost_so_far[current] + distDiagonal(current, neighbours);

                    if (cost_so_far.find(neighbours) == cost_so_far.end()
                        || new_cost < cost_so_far[neighbours]) {
                        cost_so_far[neighbours] = new_cost;

                        int priority = new_cost + heuristic(neighbours, exits);
                        frontier.put(neighbours, priority);
                        mutex.lock();
                        if(neighbours != exits) {
                            neighbours->attribute = voxelID;
                        }
                        mutex.unlock();
                        came_from[neighbours] = current;
                        visited.push_back(neighbours);

                    }

                }
            }
            for(const auto &nodes: visited){
                mutex.lock();
                if(nodes->attribute == voxelID) nodes->attribute=2;
                mutex.unlock();
            }
            visited={};

            assert(path.empty());
            path = reconstruct_path_SVO_LPA(start, exits, came_from, voxelID);

            double path_length = (voxelsize * 2.11538462) * path.size();
            double path_time = path_length / speed;


            cost_so_far = {};
            if(path.front()->isOccupied) std::this_thread::sleep_for(std::chrono::milliseconds(50));
            path.front()->isOccupied = true;
            push_to_buffer(path.front(), pathcol, voxelID, mutex, vSize, _instanceData);




            auto tFinish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> tThread = tFinish - tStart;


            out<<start->x<<" "<<start->y<<" "<<start->z<<", "<<exits->x<<" "<<exits->y<<" "<<exits->z<<", "<<path.size()<<", "<<came_from.size()<<", "<<tThread.count()<<", "<<sim_num<<"\n";
            came_from = {};

            std::this_thread::sleep_for(std::chrono::milliseconds(110));

            first = false;
        }else{
            if(path.empty()) break;
            if(near(path.front(), exits)){
                mutex.lock();
                start->attribute = 2;
                start->isOccupied = false;
                for(auto all: path){
                    all->attribute = 2;
                    all->isOccupied = false;
                }
                mutex.unlock();
                std::cout<<"Thread "<<id<<" has exited\n";
                break;
            }
            mutex.lock();
            start->attribute = 2;
            start->isOccupied = false;
            if(!path.front()->isOccupied && !path.front()->isChanged) {
                start = path.front();
                start->isOccupied = true;
                path.pop_front();
                mutex.unlock();

                push_to_buffer(start, pathcol, voxelID, mutex, vSize, _instanceData);


                std::this_thread::sleep_for(std::chrono::milliseconds(110));
            }else {
                std::cout<<"conflict at "<<path.front()->index<<"\n";
                mutex.unlock();
            }
        }


    }
    //out.close();
}
static int heuristic(OctreeNode *a, OctreeNode *b){
    return dist(a, b);
}

static int heuristic(Voxel &a, Voxel &b){
    return dist(a,b);
}

std::deque<OctreeNode *>
reconstruct_path_SVO_LPA(OctreeNode *start, OctreeNode *goal, std::map<OctreeNode *, OctreeNode *> &came_from,
                         const int voxelID) {

    std::deque<OctreeNode*> path;
    assert(path.empty());
    //std::cout<<"Reconstructing path from ["<<start->x<<", "<<start->y<<", "<<start->z<<"] to ["<<goal->x<<", "<<goal->y<<", "<<goal->z<<"]\n";
    auto current = goal;
    while (current != start){
        if(current != nullptr){
            current->attribute = voxelID;
            path.push_back(current);
            current = came_from[current];
        } else {
            std::cout<<" No path found!\n";
            break;
        }

    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());

    return path;
}



void push_to_buffer(OctreeNode *vx, Color3 &pathcol, int voxelID, std::mutex &mtx, float vSize, std::vector<InstanceData> &_instanceData){
    InstanceData _instance;
    Matrix4 _m = Matrix4::translation(
            {static_cast<float>(vx->x * vx->level * vSize + 0.5 * vSize),
             static_cast<float>(vx->y * vx->level * vSize + 0.5 * vSize),
             static_cast<float>(vx->z * vx->level * vSize + 0.5 * vSize)}) * Matrix4::scaling({vSize, vSize, vSize});
    _instance.transformation = _m;
    _instance.normalMatrix = _m.normalMatrix();
    _instance.color = pathcol;
    _instance.id = voxelID;
    mtx.lock();
    _instanceData.push_back(_instance);
    mtx.unlock();
}

/**
 * Function to compute which voxels have been changed
 *
 * @test
 *
 * @param path a path that has been generated by the reconstructPath() function
 * @param changed a vector of voxels that have been affected by the fire
 * @param mtx a mutex owned by Simulator
 * @return
 */
bool pathChanged(std::deque<OctreeNode*> &path, std::mutex &mtx){
    bool change = false;




    for(auto &e: path){
        if(e->isChanged){
            change = true;
            return change;
        }
    }
    return change;
}