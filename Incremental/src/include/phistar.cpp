#include "phistar.h"
#include <thread>
#include <cmath>
#include <mutex>
#include <set>
#include <fstream>


/**
 * Performs Phi* (Currently Theta*, wip) algorithm. Code adapted from Nash and Koenig (2010)
 *
 * @bug Doesn't work for multiples paths, and paths look weird.
 *
 * @param voxels A VoxelGrid object.
 * @param mutex a mutable mutex to handle data access.
 * @param _instanceData instance data owned by Simulator class (shared across threads).
 * @param voxelsize voxel size used to calculate travel time and for rendering purposes
 * @param id every thread and path gets a unique ID.
 * @param speed walking speed.
 * @param running atomic boolean owned by Simulator to manage all the running threads.
 * @param incremental boolean to switch between incremental and non incremental mode.
 *
 * @returns Nothing, but send path to instance buffer to visualise.
 */
void phi_star_search_grid(VoxelGrid &voxels, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                          const float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                          int sim_num) {
    std::string base = "/Users/michieldejong/Documents/Graduation/Simulator/results/grid/theta/result_theta";
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
    int voxelID = 11 + id;


    std::map<Node*, Node*> came_from;
    std::vector<std::pair<Node*, float>> open;
    std::set<Node*> visited;
    while(running) {
        auto start = new Node();
        start->x = simPoints().rooms[id].x;
        start->y = simPoints().rooms[id].y;
        start->z = simPoints().rooms[id].z;


        auto goal = new Node();
        goal->x = simPoints().exits[0].x;
        goal->y = simPoints().exits[0].y;
        goal->z = simPoints().exits[0].z;
        static float vSize = voxelsize;
        auto tStart = std::chrono::high_resolution_clock::now();
        auto this_id = std::this_thread::get_id();
        std::cout << "Starting "<<inc<<"Phi* pathfinding from " << start->x << " " << start->y << " " << start->z << " on thread #" << (id + 1)<<" at address "<<
                  this_id <<"\n";
        Voxel s = simPoints().rooms[id];
        Voxel e = simPoints().exits[0];
        double hExit = heuristic(s, e);
        for (auto all: simPoints().exits) {
            double tmp = heuristic(s, all);
            if (tmp < hExit) {
                goal->x = all.x;goal->y = all.y;goal->z = all.z;
            }
        }
        PriorityQueue<Node*, float> frontier;
        assert(frontier.empty());
        came_from[start] = start;
        initializeVertex(start);
        initializeVertex(goal);
        start->parent = start;
        start->g = 0;
        frontier.put(start, start->g + dist(start, goal));
        visited.insert(start);
        while (frontier.top() < goal->g) {
            auto current = frontier.get();
            for (auto next: current->getChildren()) {
                if ((next->x && next->y && next->z >= 0) && next->x <= voxels.max_x - 1 && next->y <= voxels.max_y - 1 &&
                    next->z <= voxels.max_z - 1) {
                    if (voxels(next->x, next->y, next->z) == voxelID && (next->x != goal->x && next->y != goal->y && next->z != goal->z)) {
                        continue;
                    }
                    if (voxels(next->x, next->y, next->z) != 0
                        && voxels(next->x, next->y, next->z) != 1
                        && voxels(next->x, next->y, next->z) != 3
                        && voxels(next->x, next->y, next->z) != 4
                        && voxels(next->x, next->y, next->z) != 5) {
                        initializeVertex(next);
                        came_from[next] = current;
                        updateVertex(current, next, goal, voxels, voxelsize, frontier, voxelID);
                        //outfile<<next->x<<" "<<next->y<<" "<<next->z<<" \n";
                        mutex.lock();
                        voxels(next->x, next->y, next->z) = voxelID;
                        mutex.unlock();

                    }
                    else {
                        delete next;
                    }
                } else {
                    delete next;
                }
                if (next->x == goal->x && next->y == goal->y && next->z == goal->z) {
                    goal->g = current->g;
                    goal->parent = current;
                    came_from[goal] = current;
                }
            }
            visited.insert(current);
        }

        std::vector<Node*> path = reconstruct_path(start, goal, came_from, mutex);
        std::cout<<"To find a path "<<came_from.size()<<" nodes have been visited\n";
        double path_length = (voxelsize * 2.11538462) * path.size();
        double path_time = path_length / speed;
        std::cout << "This path is " << path_length << " meters long, and will take " << path_time
                  << " seconds to traverse \n";

        for (const auto &nodes: path) {
            push_to_buffer(nodes, pathcol, voxelID, mutex, voxelsize, _instanceData);


        }
        auto tFinish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> tThread = tFinish - tStart;
        std::cout << "Thread " << this_id << "(#" << (id + 1) << ") has found a path in " << tThread.count()
                  << " seconds\n";


        //out<<start->x<<" "<<start->y<<" "<<start->z<<", "<<goal->x<<" "<<goal->y<<" "<<goal->z<<", "<<path.size()<<", "<<came_from.size()<<", "<<tThread.count()<<", "<<sim_num<<"\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(voxelSpeed));
        //Clean up grid
        mutex.lock();

        std::replace(voxels.voxels.begin(), voxels.voxels.end(), voxelID, 2);

        //Delete all pointers to old nodes --> this could be more efficient to keep only those that are unchanged
        for(auto all: visited){
            delete all;
        }
        visited.clear();

        //Clear the map
        came_from.erase(came_from.begin(), came_from.end());

        //Clear the path
        path.erase(path.begin(), path.end());

        //Clear the path from the instance buffer
        _instanceData.erase(
                remove_if(_instanceData.begin(), _instanceData.end(),
                          [&voxelID](InstanceData instanceData) { return instanceData.id == voxelID; }
                ), _instanceData.end());
        mutex.unlock();
    }
    //out.close();
}

void phi_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData,
                         float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental,
                         int sim_num) {
    std::string base = "/Users/michieldejong/Documents/Graduation/Simulator/results/svo/theta/result_theta_test";
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
    int voxelID = 11 + id;
    std::string idx = std::to_string(voxelID);
    std::vector<OctreeNode *> path;
    std::vector<OctreeNode*> visited;
    int level = 0;
    bool first = true;
    int i = 1;


    while(running) {
        if (pathChanged(path, mutex) || first || !succesfulPath(path)) {
            double hExit = dist(simPoints().rooms[id], simPoints().exits[0]);

            path.clear();
            _instanceData.erase(
                    remove_if(_instanceData.begin(), _instanceData.end(),
                              [&voxelID](InstanceData instanceData) { return instanceData.id == voxelID; }
                    ), _instanceData.end());

            auto startIndex = libmorton::morton3D_64_encode(simPoints().rooms[id].x, simPoints().rooms[id].y,
                                                            simPoints().rooms[id].z);
            auto goalIndex = libmorton::morton3D_64_encode(simPoints().exits[0].x, simPoints().exits[0].y,
                                                           simPoints().exits[0].z);
            auto start = octree.mTree[0][startIndex];
            auto goal = octree.mTree[0][goalIndex];
            for (auto all: simPoints().exits) {
                double tmp = dist(simPoints().rooms[id], all);
                if (tmp < hExit) {
                    auto index = libmorton::morton3D_64_encode(all.x, all.y, all.z);
                    goal = octree.mTree[0][index];
                }
            }
            static uint_fast64_t goal_morton = goal->index;
            static uint_fast64_t start_morton = start->index;
            static float vSize = voxelsize;
            auto tStart = std::chrono::high_resolution_clock::now();
            auto this_id = std::this_thread::get_id();
            std::cout << "Starting " << inc << "Phi* pathfinding from " << start->x << " " << start->y << " "
                      << start->z << " on thread #" << (id + 1) << " at address " <<
                      this_id << "\n";

            PriorityQueue<OctreeNode *, float> frontier = {};
            std::map<OctreeNode*, OctreeNode*> came_from = {};
            std::vector<std::pair<OctreeNode*, float>> open = {};
            assert(frontier.empty());
            came_from[start] = start;
            initializeVertex(start);
            initializeVertex(goal);
            start->PhiParent = start;
            start->g = 0;
            frontier.put(start, start->g + dist(start, goal));
            visited.push_back(start);
            while (frontier.top() < goal->g) {
                auto current = frontier.get();
                for (const auto &conn: getNeighbours26_morton(current->x, current->y, current->z, current->index)) {
                    if (!octree.mQTree[level].contains(conn)) continue;
                    auto neighbours = octree.mQTree[0][conn];
                    if (neighbours->attribute == voxelID) continue;
                    if(neighbours == nullptr) continue;

                    if(initializeVertex(neighbours)) { ;
                        came_from[neighbours] = current;
                        updateVertex(current, neighbours, goal, octree, voxelsize, frontier, voxelID);
                        visited.push_back(neighbours);

                        mutex.lock();
                        neighbours->attribute = voxelID;
                        mutex.unlock();


                        if (neighbours->index == goal_morton) {
                            goal->g = current->g;
                            goal->PhiParent = current;
                            came_from[goal] = current;
                            break;
                        }
                    } else {
                        continue;
                    }
                }
            }

            path = reconstruct_path(start, goal, came_from, mutex);
            std::cout << "To find a path " << came_from.size() << " nodes have been visited\n";
            double path_length = (voxelsize * 2.11538462) * path.size();
            double path_time = path_length / speed;
            std::cout << "This path is " << path_length << " meters long, and will take " << path_time
                      << " seconds to traverse \n";

            for (const auto &nodes: path) {
                push_to_buffer(nodes, pathcol, voxelID, mutex, voxelsize, _instanceData);
            }
            auto tFinish = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> tThread = tFinish - tStart;
            //out<<start->x<<" "<<start->y<<" "<<start->z<<", "<<goal->x<<" "<<goal->y<<" "<<goal->z<<", "<<path.size()<<", "<<came_from.size()<<", "<<tThread.count()<<", "<<sim_num<<"\n";
            std::cout << "Thread " << this_id << "(#" << (id + 1) << ") has found a path in " << tThread.count()
                      << " seconds\n";

            for(const auto &nodes: visited){
                mutex.lock();
                nodes->attribute = 2;
                nodes->PhiParent = nullptr;
                nodes->g = INFINITY;
                mutex.unlock();

            }


            std::this_thread::sleep_for(std::chrono::milliseconds(voxelSpeed));



            first = false;

        }
    }
    //out.close();
}


bool lineOfSight_grid(VoxelGrid &voxels, Node &start, Node &end, float voxelsize){
    bool result = true;
    int x = start.x;
    int y = start.y;
    int z = start.z;
    int dx = end.x - start.x;
    float tDeltaX = voxelsize / dx;
    int dz = end.z - start.z;
    float tDeltaZ = voxelsize / dz;
    float tMaxX = tDeltaX * (1.0f - (x/ voxelsize));
    float tMaxZ = tDeltaZ * (1.0f - (z / voxelsize));
    int stepX, stepZ;
    if(end.x > start.x){
        stepX = 1;
    } else {
        stepX = -1;
    }
    if(end.z > start.z){
        stepZ = 1;
    } else {
        stepZ = -1;
    }
    while(true){
        if( tMaxX < tMaxZ) {
            tMaxX += tDeltaX;
            x += stepX;
        } else {
            tMaxZ += tDeltaZ;
            z += stepZ;
        }
        if( tMaxX > 1 && tMaxZ > 1) {
            break;
        }
        if((x && y && z >= 0) && x <= voxels.max_x - 1 && y <= voxels.max_y - 1 && z <= voxels.max_z - 1) {
            if (voxels(x, y, z) != 2) {
                result = false;
                break;
            }
        }else {
            result = false;
            break;
        }
    }



    return result;
}

static int heuristic(Node *a, Node *b){
    return dist(a,b);
}

static int heuristic(Voxel &a, Voxel &b){
    return dist(a,b);
}

void initializeVertex(Node *node) {
    node->g = INFINITY;
    node->parent = nullptr;
}

void updateVertex(Node *current, Node *next, Node *goal, VoxelGrid &voxels, float voxelsize, PriorityQueue<Node*, float> &queue, int voxelID){
    float g_old = next->g;
    computeCost(current, next, voxels, voxelsize, voxelID);
    if(next->g < g_old && voxels(next->x, next->y, next->z) != voxelID){
        queue.put(next, next->g + dist(next, goal));
    }
}

void computeCost(Node *current, Node *next, VoxelGrid &voxels, float voxelsize, int voxelID) {
    std::vector<Node*> line = raytrace3d(current->parent, next, voxels, voxelID);
    if(lineOfSight(line, voxels, voxelID)){
        line.erase(line.begin(), line.end());
        float new_g = current->parent->g + dist(current->parent, next);

        /* Path 2 */
        if(new_g < next->g){
            next->parent = current->parent;
            next->g = new_g;
        }
    }else{
        /* Path 1 */
        float new_g = current->g + dist(current, next);

        if(new_g < next->g){
            next->parent = current;
            next->g = new_g;
        }
    }
}

std::vector<Node> raytrace(Node *n0, Node *n1, VoxelGrid & voxels, int voxelID) {
    std::vector<Node> line;
    int x0, z0, y0, y1, x1, z1;
    x0 = n0->x;
    x1 = n1->x;
    z0 = n0->z;
    z1 = n1->z;
    y0 = n0->y;
    y1 = n1->y;
    int dx = abs(x1 - x0);
    int dz = abs(z1 - z0);
    int dy = abs(y1 - y0);
    int x = x0;
    int y = y0;
    int z = z0;
    int n = 1 + dx + dz + dy;

    int x_inc = (x1 > x0) ? 1 : -1;
    int z_inc = (z1 > z0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dz;
    dx *= 2;
    dz *= 2;

    for (; n > 0; --n)
    {
        if(voxels(x,y,z)==2 || voxels(x,y,z)==voxelID){
            line.emplace_back(Node(x, y, z));
            if (error > 0)
            {
                x += x_inc;
                error -= dz;
            }
            else
            {
                z += z_inc;
                error += dx;
            }
        }
        else {
            break;
        }


    }
    return line;
}

bool lineOfSight(std::vector<Node*> &line, VoxelGrid &voxels, int voxelID){
    bool result = true;
    for(const auto &node: line){
        //std::cout<<voxels(node.x, node.y, node.z)<<" is the value at ["<<node.x<<", "<< node.y<<", "<< node.z<<"]\n";
        if(voxels(node->x, node->y, node->z) != 2 && voxels(node->x, node->y, node->z) != voxelID){
            result = false;
        }

    }
    return result;
}

std::vector<Node*> reconstruct_path( Node *start,  Node *goal, std::map<Node*, Node*> &came_from, std::mutex &mtx) {

    std::vector<Node*> path;
    assert(path.empty());
    std::cout<<"Reconstructing path from ["<<start->x<<", "<<start->y<<", "<<start->z<<"] to ["<<goal->x<<", "<<goal->y<<", "<<goal->z<<"]\n";
    auto current = goal;
    while (current != start){
        if(current != nullptr){

            path.push_back(current);
            //current = current->parent;
            current = came_from[current];
        } else {
            std::cout<<" No path found\n";
            break;
        }

    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<Node*> raytrace3d(Node *n0, Node *n1, VoxelGrid & voxels, int voxelID){
    /*
    * C code from the article
    * "Voxel Traversal along a 3D Line"
    * by Daniel Cohen, danny@bengus.bgu.ac.il
    * in "Graphics Gems IV", Academic Press, 1994
    */
    std::vector<Node*> line;
    int n, sx, sy, sz, exy, exz, ezy, ax, ay, az, bx, by, bz, dx, dy, dz, x0, z0, y0, y1, x1, z1;
    x0 = n0->x;
    x1 = n1->x;
    z0 = n0->z;
    z1 = n1->z;
    y0 = n0->y;
    y1 = n1->y;
    dx = abs(x1 - x0);
    dz = abs(z1 - z0);
    dy = abs(y1 - y0);
    sx = sgn(dx);
    sy = sgn(dy);
    sz = sgn(dz);
    ax = abs(dx);
    ay = abs(dy);
    az = abs(dz);
    bx = 2*ax;
    by = 2*ay;
    bz = 2*az;
    exy = ay-ax;
    exz = az-ax;
    ezy = ay-az;
    n = ax+ay+az;
    int x = x0;
    int y = y0;
    int z = z0;
    while ( n-- ) {
        if(voxels(x,y,z)==2) {
            auto tmp = new Node();
            tmp->x = x;tmp->y = y;tmp->z = z;
            line.emplace_back(tmp);
            if (exy < 0) {
                if (exz < 0) {
                    x += sx;
                    exy += by;
                    exz += bz;
                } else {
                    z += sz;
                    exz -= bx;
                    ezy += by;
                }
            } else {
                if (ezy < 0) {
                    z += sz;
                    exz -= bx;
                    ezy += by;
                } else {
                    y += sy;
                    exy -= bx;
                    ezy -= bz;
                }
            }
        }
    }

    return line;
}

void push_to_buffer(OctreeNode *nodes, Color3 &pathcol, int id, std::mutex &mtx, float voxelsize, std::vector<InstanceData> &_instanceData){
    InstanceData _instance;
    Matrix4 _m = Matrix4::translation(
            {static_cast<float>(nodes->x * voxelsize + 0.5 * voxelsize),
             static_cast<float>(nodes->y * voxelsize + 0.5 * voxelsize),
             static_cast<float>(nodes->z * voxelsize + 0.5 * voxelsize)}) *
                 Matrix4::scaling({voxelsize, voxelsize, voxelsize});
    _instance.transformation = _m;
    _instance.normalMatrix = _m.normalMatrix();
    _instance.color = pathcol;
    _instance.id = id;
    mtx.lock();
    _instanceData.push_back(_instance);
    mtx.unlock();
}

void push_to_buffer(Node *nodes, Color3 &pathcol, int id, std::mutex &mtx, float voxelsize, std::vector<InstanceData> &_instanceData){
    InstanceData _instance;
    Matrix4 _m = Matrix4::translation(
            {static_cast<float>(nodes->x * voxelsize + 0.5 * voxelsize),
             static_cast<float>(nodes->y * voxelsize + 0.5 * voxelsize),
             static_cast<float>(nodes->z * voxelsize + 0.5 * voxelsize)}) *
                 Matrix4::scaling({voxelsize, voxelsize, voxelsize});
    _instance.transformation = _m;
    _instance.normalMatrix = _m.normalMatrix();
    _instance.color = pathcol;
    _instance.id = id;
    mtx.lock();
    _instanceData.push_back(_instance);
    mtx.unlock();
}

std::vector<OctreeNode*> raytrace3d(OctreeNode *n0, OctreeNode *n1, SparseVoxelOctree &octree, int voxelID){
    /*
    * C code from the article
    * "Voxel Traversal along a 3D Line"
    * by Daniel Cohen, danny@bengus.bgu.ac.il
    * in "Graphics Gems IV", Academic Press, 1994
    */
    std::vector<OctreeNode*> line;
    if(n0 == nullptr) return line;

    int n, sx, sy, sz, exy, exz, ezy, ax, ay, az, bx, by, bz, dx, dy, dz, x0, z0, y0, y1, x1, z1;
    x0 = n0->x;
    x1 = n1->x;
    z0 = n0->z;
    z1 = n1->z;
    y0 = n0->y;
    y1 = n1->y;
    dx = abs(x1 - x0);
    dz = abs(z1 - z0);
    dy = abs(y1 - y0);
    sx = sgn(dx);
    sy = sgn(dy);
    sz = sgn(dz);
    ax = abs(dx);
    ay = abs(dy);
    az = abs(dz);
    bx = 2*ax;
    by = 2*ay;
    bz = 2*az;
    exy = ay-ax;
    exz = az-ax;
    ezy = ay-az;
    n = ax+ay+az;
    int x = x0;
    int y = y0;
    int z = z0;
    while ( n-- ) {
        auto a = libmorton::morton3D_64_encode(x, y, z);
        if(octree.mQTree[0].contains(a)) {
            auto tmp =octree.mQTree[0][a];
            line.emplace_back(tmp);
            if (exy < 0) {
                if (exz < 0) {
                    x += sx;
                    exy += by;
                    exz += bz;
                } else {
                    z += sz;
                    exz -= bx;
                    ezy += by;
                }
            } else {
                if (ezy < 0) {
                    z += sz;
                    exz -= bx;
                    ezy += by;
                } else {
                    y += sy;
                    exy -= bx;
                    ezy -= bz;
                }
            }
        }
    }
    return line;
}

std::vector<OctreeNode*> reconstruct_path(OctreeNode *start,OctreeNode *goal, std::map<OctreeNode*, OctreeNode*> &came_from, std::mutex &mtx){
    std::vector<OctreeNode*> path;
    assert(path.empty());
    std::cout<<"Reconstructing path from ["<<start->x<<", "<<start->y<<", "<<start->z<<"] to ["<<goal->x<<", "<<goal->y<<", "<<goal->z<<"]\n";
    auto current = goal;
    while (current != start){
        if(current != nullptr){

            path.push_back(current);
            current = came_from[current];
        } else {
            std::cout<<" No path found\n";
            break;
        }

    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());

    return path;
}

bool lineOfSight(std::vector<OctreeNode*> &line, SparseVoxelOctree &octree, int voxelID){
    bool result = true;
    if(line.size() == 0) return false;
    for(const auto &node: line){
        //std::cout<<voxels(node.x, node.y, node.z)<<" is the value at ["<<node.x<<", "<< node.y<<", "<< node.z<<"]\n";
        if(!octree.mQTree[0].contains(node->index) || node->attribute != voxelID){
            result = false;
        }

    }
    return result;
}

bool initializeVertex(OctreeNode *node){
    if(node->attribute < 11) {
        node->g = INFINITY;
        node->PhiParent = nullptr;
        return true;
    } else {
        return false;
    }
}

void updateVertex(OctreeNode *current, OctreeNode *next, OctreeNode *goal, SparseVoxelOctree &octree, float voxelsize, PriorityQueue<OctreeNode*, float> &queue, int voxelID){
    auto g_old = next->g;
    computeCost(current, next, octree, voxelsize, voxelID);
    if(next->g < g_old && next->attribute != voxelID){
        queue.put(next, next->g + dist(next, goal));
    }
}

void computeCost(OctreeNode *current, OctreeNode *next, SparseVoxelOctree &octree, float voxelsize, int voxelID){
    std::vector<OctreeNode*> line = raytrace3d(current->PhiParent, next, octree, voxelID);
    if(lineOfSight(line, octree, voxelID)){
        line.erase(line.begin(), line.end());
        float new_g = current->PhiParent->g + dist(current->PhiParent, next);

        /* Path 2 */
        if(new_g < next->g){
            next->PhiParent = current->PhiParent;
            next->g = new_g;
        }
    }else{
        /* Path 1 */
        float new_g = current->g + dist(current, next);

        if(new_g < next->g){
            next->PhiParent = current;
            next->g = new_g;
        }
    }
}

static bool pathChanged(std::vector<OctreeNode*> &path, std::mutex &mtx){
    bool change = false;
    for(auto &e: path){
        if(e->isChanged){
            change = true;
            return change;
        }
    }
    return change;
}


