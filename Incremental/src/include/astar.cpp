#include "astar.h"
#include "Magnum/Magnum.h"
#include <map>
#include <Magnum/Math/Matrix4.h>
#include <thread>
#include "Simdata.h"
#include <cassert>
#include <fstream>

/**
 *
 * This is the main A* function for searching on a dense uniform voxelgrid (as defined in VoxelGrid.h)
 * The code has been adapted from https://www.redblobgames.com/.
 *
 *
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
void a_star_search_grid(VoxelGrid &voxels, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                        const float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                        int sim_num) {
    std::string base = "/Users/michieldejong/Documents/Graduation/Simulator/results/grid/a_star/result_astar";
    std::string ext = ".csv";
    auto t = std::chrono::system_clock::now();
    std::time_t ts = std::chrono::system_clock::to_time_t(t);
    std::string t_start = std::to_string(ts);
    std::string threadID = std::to_string(id);
    base+=threadID;
    base+=t_start;
    base+=ext;
    //std::ofstream out(base);
    //out<<"start, goal, path_length, nodes_visited, time, num_evax\n";

    Voxel start = simPoints().rooms[id];
    Color3 pathcol = Color3(randomFloat(0, 1), randomFloat(0, 1), randomFloat(0, 1));
    int voxelSpeed;
    const char* inc;
    if(incremental){
        voxelSpeed = std::round(1 /(speed / (voxelsize * 2))) * 1000;
        inc = "incremental ";
    }
    else {
        voxelSpeed = 2000;
        inc = "";
    }
    int pos = 1;
    while(running) {
        mutex.lock();
        Voxel exits = simPoints().exits[0];
        std::thread::id this_id = std::this_thread::get_id();
        std::cout << "Starting "<<inc<<" A* pathfinding from " << start.x << " " << start.y << " " << start.z << " on thread "
                  << this_id << " thread #" << (id + 1) << "\n";
        std::map<Voxel, Voxel> came_from;
        std::map<Voxel, int> cost_so_far;
        double hExit = heuristic(start, exits);
        for (auto all: simPoints().exits) {
            double tmp = heuristic(start, all);
            if (tmp < hExit) {
                exits = all;
            }
        }
        static float vSize = voxelsize;
        auto tStart = std::chrono::high_resolution_clock::now();

        PriorityQueue<Voxel, int> frontier;
        mutex.unlock();

        assert(frontier.empty());
        frontier.put(start, 0);
        came_from[start] = start;
        cost_so_far[start] = 0;
        int voxelID = 11 + id;
        while (!frontier.empty()) {
            Voxel current = frontier.get();
            if (current == exits) {
                break;
            }
            for (auto next: current.getNeighbours18()) {
                if ((next.x && next.y && next.z >= 0) && next.x <= voxels.max_x - 1 && next.y <= voxels.max_y - 1 &&
                    next.z <= voxels.max_z - 1) {
                    if (voxels(next.x, next.y, next.z) != 0
                        && voxels(next.x, next.y, next.z) != voxelID
                        && voxels(next.x, next.y, next.z) != 1
                        && voxels(next.x, next.y, next.z) != 3
                        && voxels(next.x, next.y, next.z) != 4
                        && voxels(next.x, next.y, next.z) != 5) {
                        int new_cost = cost_so_far[current] + distDiagonal(current, next);

                        if (cost_so_far.find(next) == cost_so_far.end()
                            || new_cost < cost_so_far[next]) {
                            cost_so_far[next] = new_cost;
                            int priority = new_cost + heuristic(next, exits);
                            mutex.lock();
                            if (next != exits) {
                                voxels(next.x, next.y, next.z) = voxelID;
                            }
                            frontier.put(next, priority);
                            came_from[next] = current;
                            mutex.unlock();
                        }
                    }
                }
            }
        }

        std::vector<Voxel> path;
        assert(path.empty());
        path = reconstruct_path(start, exits, came_from);
        std::cout<<"To find a path "<<came_from.size()<<" nodes have been visited\n";

        double path_length = (voxelsize * 2.11538462) * path.size();
        double path_time = path_length / speed;

        std::cout << "This path is " << path_length << " meters long, and will take " << path_time
                  << " seconds to traverse \n";

        for (auto &vx: path) {
            push_to_buffer(vx, pathcol, voxelID, mutex, vSize , _instanceData);

        }
        auto tFinish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> tThread = tFinish - tStart;
        std::cout << "Thread " << this_id << "(#" << (id + 1) << ") has found a path in " << tThread.count()
                  << " seconds\n";
        //out<<start.x<<" "<<start.y<<" "<<start.z<<", "<<exits.x<<" "<<exits.y<<" "<<exits.z<<", "<<path.size()<<", "<<came_from.size()<<", "<<tThread.count()<<", "<<sim_num<<"\n";
        came_from.erase(came_from.begin(), came_from.end());
        cost_so_far.erase(cost_so_far.begin(), cost_so_far.end());
        if(incremental) {
            if (path[pos] != exits) {
                if (path.size() >= pos) {
                    if (voxels(path[voxelSpeed / 2].x, path[voxelSpeed / 2].y, path[voxelSpeed / 2].z) != 3
                        && voxels(path[voxelSpeed].x, path[voxelSpeed].y, path[voxelSpeed].z) != 3) {
                        start = path[pos];
                    } else
                        start = simPoints().rooms[id];

                } else {
                    break;
                }
            } else {
                break;
            }

        }
        path.erase(path.begin(), path.end());

        std::this_thread::sleep_for(std::chrono::milliseconds(voxelSpeed));
        mutex.lock();
        std::replace(voxels.voxels.begin(), voxels.voxels.end(), voxelID, 2);
        _instanceData.erase(
                remove_if(_instanceData.begin(), _instanceData.end(),
                          [&voxelID](InstanceData instanceData) { return instanceData.id == voxelID; }
                ), _instanceData.end());
        mutex.unlock();
    }
    //out.close();
    /*
    mutex.lock();
    Voxel exits = simPoints().exits[0];
    std::thread::id this_id = std::this_thread::get_id();
    std::cout << "Starting "<<inc<<" A* pathfinding from " << start.x << " " << start.y << " " << start.z << " on thread "
              << this_id << " thread #" << (id + 1) << "\n";
    std::map<Voxel, Voxel> came_from;
    std::map<Voxel, int> cost_so_far;
    double hExit = heuristic(start, exits);
    for (auto all: simPoints().exits) {
        double tmp = heuristic(start, all);
        if (tmp < hExit) {
            exits = all;
        }
    }
    if(!incremental) {
        static float vSize = voxelsize;
        auto tStart = std::chrono::high_resolution_clock::now();
        int minx, maxx, miny, maxy, minz, maxz;
        getBounds(start, exits, minx, maxx, miny, maxy, minz, maxz);
        PriorityQueue<Voxel, int> frontier;
        mutex.unlock();

        assert(frontier.empty());
        frontier.put(start, 0);
        came_from[start] = start;
        cost_so_far[start] = 0;
        int voxelID = 11 + id;

        while (!frontier.empty()) {
            Voxel current = frontier.get();
            if (current == exits) {
                break;
            }
            for (auto next: current.getNeighbours26()) {
                if ((next.x && next.y && next.z >= 0) && next.x <= voxels.max_x - 1 && next.y <= voxels.max_y - 1 &&
                    next.z <= voxels.max_z - 1) {
                    if (voxels(next.x, next.y, next.z) != 0
                        && voxels(next.x, next.y, next.z) != voxelID
                        && voxels(next.x, next.y, next.z) != 1
                        && voxels(next.x, next.y, next.z) != 3
                        && voxels(next.x, next.y, next.z) != 4
                        && voxels(next.x, next.y, next.z) != 5) {
                        int new_cost = cost_so_far[current] + heuristic(current, next);

                        if (cost_so_far.find(next) == cost_so_far.end()
                            || new_cost < cost_so_far[next]) {
                            cost_so_far[next] = new_cost;
                            int priority = new_cost + heuristic(next, exits);
                            mutex.lock();
                            if (next != exits) {
                                voxels(next.x, next.y, next.z) = voxelID;
                            }
                            frontier.put(next, priority);
                            came_from[next] = current;
                            mutex.unlock();
                        }
                    }

                }

            }
        }


        std::vector<Voxel> path;
        assert(path.empty());
        //std::cout<<"came_from has a size of "<<came_from.size()<<"\n";
        path = reconstruct_path(start, exits, came_from);
        double path_length = (voxelsize * 2.11538462) * path.size();
        double path_time = path_length / speed;
        std::cout<<"To find a path "<<came_from.size()<<" nodes have been visited\n";


        std::cout << "This path is " << path_length << " meters long, and will take " << path_time
                  << " seconds to traverse \n";

        for (auto &vx: path) {
            push_to_buffer(vx, pathcol, voxelID, mutex, vSize , _instanceData);
        }
        //voxelMesh.setInstanceCount(_instanceData.size());
        auto tFinish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> tThread = tFinish - tStart;

        came_from.erase(came_from.begin(), came_from.end());
        cost_so_far.erase(cost_so_far.begin(), cost_so_far.end());
        std::cout << "Thread " << this_id << "(#" << (id + 1) << ") has found a definite path in " << tThread.count()
                  << " seconds\n";
    }
     */

}
/**
 * Performs A* pathfinding on the Sparse Voxel Octree.
 *
 * @todo MAKE USE OF HIERARCHY
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
void a_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                       float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental, int sim_num) {
    std::string base = "/Users/michieldejong/Documents/Graduation/Simulator/results/svo/a_star/result_a_star";
    std::string ext = ".csv";
    auto t = std::chrono::system_clock::now();
    std::time_t ts = std::chrono::system_clock::to_time_t(t);
    std::string t_start = std::to_string(ts);
    std::string threadID = std::to_string(id);
    base+=threadID;
    base+=t_start;
    base+=ext;
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
    std::vector<OctreeNode*> path;
    std::vector<OctreeNode *> visited = {};
    bool first = true;
    int voxelID = 11 + id;

    if(incremental){
        voxelSpeed = std::round(1 /(speed / (voxelsize * 2))) * 1000;
        inc = "incremental ";
    }
    else {
        voxelSpeed = 2000;
        inc = "";
    }
    int pos = 1;
    int level = 0;

    while(running) {
        if(pathChanged(path, mutex) || first) {
            for(const auto &nodes: visited){
                mutex.lock();
                nodes->attribute = 2;
                mutex.unlock();

            }
            path.clear();
            _instanceData.erase(
                    remove_if(_instanceData.begin(), _instanceData.end(),
                              [&voxelID](InstanceData instanceData) { return instanceData.id == voxelID; }
                    ), _instanceData.end());
            std::thread::id this_id = std::this_thread::get_id();

            auto tStart = std::chrono::high_resolution_clock::now();

            std::cout << "Starting " << inc << " A* pathfinding from " << start->x << " " << start->y << " " << start->z
                      << " on thread "
                      << this_id << " thread #" << (id + 1) << "\n";
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
                    //mutex.lock();
                    if(!octree.mQTree[level].contains(conn)) continue;
                    //mutex.unlock();
                    auto neighbours = octree.mQTree[0][conn];
                    if(neighbours->attribute == voxelID) continue;
                    int new_cost = cost_so_far[current] + distDiagonal(current, neighbours);

                    if (cost_so_far.find(neighbours) == cost_so_far.end()
                        || new_cost < cost_so_far[neighbours]) {
                        cost_so_far[neighbours] = new_cost;
                        //std::cout<<neighbours->x<<" "<<neighbours->y<<" "<<neighbours->z<<" \n";

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


            assert(path.empty());
            path = reconstruct_path_SVO(start, exits, came_from);

            double path_length = (voxelsize * 2.11538462) * path.size();
            double path_time = path_length / speed;
            std::cout << "To find a path " << came_from.size() << " nodes have been visited\n";

            std::cout << "This path is " << path_length << " meters long, and will take " << path_time
                      << " seconds to traverse \n";




            for (auto &vx: path) {
                push_to_buffer(vx,pathcol, voxelID, mutex, vSize, _instanceData);

            }


            auto tFinish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> tThread = tFinish - tStart;
            std::cout << "Thread " << this_id << "(#" << (id + 1) << ") has found a path in " << tThread.count()
                      << " seconds\n";

            out<<start->x<<" "<<start->y<<" "<<start->z<<", "<<exits->x<<" "<<exits->y<<" "<<exits->z<<", "<<path.size()<<", "<<came_from.size()<<", "<<tThread.count()<<", "<<sim_num<<"\n";

            std::this_thread::sleep_for(std::chrono::milliseconds(voxelSpeed));

            first = false;
        }

    }
    out.close();
}

/**
 * Heuristic for A* performance. Currently Manhattan distance.
 *
 * @param a starting Voxel
 * @param b goal Voxel
 * @return Manhattan distance between a and b
 */
static int heuristic(Voxel &a, Voxel &b){
    return dist(a,b);
}

static int heuristicOctree(OctreeNode * a, OctreeNode * b){
    return std::abs(a->x * a->level - b->x * b->level) + std::abs(a->y * a->level - b->y * b->level) + std::abs(a->z * a->level - b->z * b->level);
}

static int heuristic(uint_fast64_t a, uint_fast64_t b){
    uint_fast32_t xa, xb, ya, yb, za, zb;
    libmorton::morton3D_64_decode(a, xa, ya, za);
    libmorton::morton3D_64_decode(a, xb, yb, zb);
    auto av = Voxel{xa, ya, za};
    auto bv = Voxel{xb, yb, zb};
    return distManhattan(av, bv);
}
static int heuristic(OctreeNode *a, OctreeNode *b){
    return dist(a, b);
}


/**
 * Reconstructs path backwards using the came_from map.
 *
 * @param start starting voxel
 * @param goal ending voxel
 * @param came_from a <Voxel, Voxel> map to recalculate the path.
 * @return a vector of voxels representing a path that can be sent to the instance buffer.
 */
std::vector<Voxel> reconstruct_path(const Voxel start, const Voxel goal, std::map<Voxel,Voxel> &came_from){

    std::vector<Voxel> path;
    assert(path.empty());
    std::cout<<"Reconstructing path from ["<<start.x<<", "<<start.y<<", "<<start.z<<"] to ["<<goal.x<<", "<<goal.y<<", "<<goal.z<<"]\n";
    Voxel current = goal;
    while (current != start){
        if(current != Voxel{0,0,0}){
            path.push_back(current);
            current = came_from[current];
        } else {
            std::cout<<"No path found!\n";
            break;
        }

    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<OctreeNode *> reconstruct_path_SVO(OctreeNode* start, OctreeNode* goal, std::map<OctreeNode*,OctreeNode*> &came_from){
    const char* pathfile = "/Users/michieldejong/Documents/Graduation/Simulator/dataset/path.txt";
    std::ofstream pf(pathfile);
    std::vector<OctreeNode*> path;
    assert(path.empty());
    std::cout<<"Reconstructing path from ["<<start->x<<", "<<start->y<<", "<<start->z<<"] to ["<<goal->x<<", "<<goal->y<<", "<<goal->z<<"]\n";
    auto current = goal;
    while (current != start){
        if(current != nullptr){
            //pf<<(current->x * current->level)<<" "<<(current->y * current->level)<<" "<<(current->z * current->level)<<" \n";
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



bool isID(InstanceData instance, int id){
    return instance.id == id;
}

/**
 * Gets the bbox of a space bounded by two voxels
 *
 * @param a
 * @param b
 * @param min_x
 * @param max_x
 * @param min_y
 * @param max_y
 * @param min_z
 * @param max_z
 */
void getBounds(Voxel& a, Voxel &b, int &min_x, int &max_x, int &min_y, int &max_y,int &min_z, int &max_z){
    min_x  = std::min(a.x, b.x);
    min_y  = std::min(a.y, b.y);
    min_z  = std::min(a.z, b.z);
    max_x  = std::max(a.x, b.x);
    max_y  = std::max(a.y, b.y);
    max_z  = std::max(a.z, b.z);
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

void push_to_buffer(Voxel &vx, Color3 &pathcol, int voxelID, std::mutex &mtx, float vSize, std::vector<InstanceData> &_instanceData){
    InstanceData _instance;
    Matrix4 _m = Matrix4::translation(
            {static_cast<float>(vx.x * vSize + 0.5 * vSize), static_cast<float>(vx.y * vSize + 0.5 * vSize),
             static_cast<float>(vx.z * vSize + 0.5 * vSize)}) * Matrix4::scaling({vSize, vSize, vSize});
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
bool pathChanged(std::vector<OctreeNode*> &path, std::mutex &mtx){
    bool change = false;
    for(auto &e: path){
        if(e->isChanged){
            change = true;
            return change;
        }
    }
    return change;
}



