#include "dlite.h"



void d_star_search_grid(DynamicGrid &dynamicGrid, std::mutex &mutex,std::vector<InstanceData> &_instanceData, float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental){
    //TODO
    auto startVoxel = simPoints().rooms[id];
    auto endVoxel = simPoints().exits[0];
    id = id + 11;
    auto start = dynamicGrid(startVoxel.x, startVoxel.y, startVoxel.z);
    auto goal = dynamicGrid(endVoxel.x, endVoxel.y, endVoxel.z);
    auto last = start;
    start->rhs = INFINITY;
    start->g = INFINITY;
    float k_m = 0;
    const char* pathfile = "/Users/michieldejong/Documents/Graduation/Simulator/dataset/path.txt";
    std::ofstream pf(pathfile);
    PriorityQueue<Node*, std::pair<double, double>> frontier = {};
    std::set<Node*> visited = {};
    std::unordered_set<Node*> openList={};
    goal->rhs = 0;

    frontier.put(goal, calculateKey(goal, start, k_m));
    openList.insert(goal);
    computeShortestPath(frontier, start, goal, dynamicGrid, id, visited, k_m, openList, pf);
    std::cout<<"Computed shortest path\n";
    //auto p = reconstructPath(visited, goal, start);
}

void d_star_search_svo(SparseVoxelOctree &octree, std::mutex &mutex, std::vector<InstanceData> &_instanceData, float voxelsize, int id, float speed, std::atomic_bool &running, bool incremental){
    //TODO
}

void updateVertex(Node* u, Node* goal,Node* start, PriorityQueue<Node*, std::pair<double, double>> &frontier, DynamicGrid &dynamicGrid, int id,std::set<Node*> &visited, float k_m, std::unordered_set<Node*> &openList, std::ofstream &out) {
    if(u != goal){
        double tmp1 = INFINITY;
        double tmp2;

        for(auto neighbours: getSucc(u, dynamicGrid, id)){
            if(!checkVoxelStatus(neighbours, dynamicGrid, id)) continue;
            out<<neighbours->x<<" "<<neighbours->y<<" "<<neighbours->z<<" "<<neighbours->g<<" "<<neighbours->rhs<<" \n";
            tmp2 = neighbours->g + dist( u, neighbours);
            if (tmp2 < tmp1) {
                tmp1 = tmp2;
            }
            neighbours->parent = u;
        }
        u->rhs = tmp1;
    }
    if(!near(u->g,u->rhs)) {
        frontier.put(u, calculateKey(u, start, k_m));
        openList.insert(u);
    } else {
        openList.erase(u);
    }
}

void computeShortestPath(PriorityQueue<Node*, std::pair<double, double>> &frontier,Node* start, Node* goal, DynamicGrid &dynamicGrid, int id,std::set<Node*> &visited, float k_m, std::unordered_set<Node*> &openList, std::ofstream &out) {
    auto a = calculateKey(start, start, k_m);
    while(!frontier.empty() && frontier.top() < a || start->rhs != start->g){
        auto k_old = frontier.top();
        auto current = frontier.get();
        if(!openList.contains(current)) continue;
        auto b =calculateKey(current, start, k_m);
        if(k_old < b){
            frontier.put(current,calculateKey(current, start, k_m));
            openList.insert(current);
        } else if ( current->g > current->rhs) {
            current->g = current->rhs;
            for(auto neighbours: getSucc(current, dynamicGrid, id)){
                if(checkVoxelStatus(neighbours, dynamicGrid, id)) {
                    out<<neighbours->x<<" "<<neighbours->y<<" "<<neighbours->z<<" "<<neighbours->g<<" "<<neighbours->rhs<<" \n";
                    updateVertex(neighbours, goal, start, frontier, dynamicGrid, id, visited, k_m, openList, out);
                    neighbours->parent = current;
                }
            }
        } else {
            current->g = INFINITY;
            for(auto neighbours: getSucc(current, dynamicGrid, id)){
                if(checkVoxelStatus(neighbours, dynamicGrid, id)) {
                    out<<neighbours->x<<" "<<neighbours->y<<" "<<neighbours->z<<" "<<neighbours->g<<" "<<neighbours->rhs<<" \n";

                    updateVertex(neighbours, goal,start, frontier, dynamicGrid, id, visited, k_m, openList, out);
                    neighbours->parent = current;
                }
            }
            updateVertex(current, goal,start, frontier, dynamicGrid, id, visited, k_m, openList, out);

        }
        current->id = id;
    }
}

std::pair<double, double> calculateKey(Node* current, Node* start, float k_m) {
    auto val = std::fmin(current->rhs, current->g);
    auto k1 = val + dist(current, start) + k_m;
    auto k2 = val;
    return {k1, k2};
}

bool checkVoxelStatus(Node* next, DynamicGrid &dynamicGrid, int id){
        if (dynamicGrid(next->x, next->y, next->z)->id != 0
            && dynamicGrid(next->x, next->y, next->z)->id != id
            && dynamicGrid(next->x, next->y, next->z)->id != 1
            && dynamicGrid(next->x, next->y, next->z)->id != 3
            && dynamicGrid(next->x, next->y, next->z)->id != 4
            && dynamicGrid(next->x, next->y, next->z)->id != 5) {
            return true;
        }
        return false;
}

std::vector<Node*> reconstructPath(std::set<Node*> &visited, Node* goal, Node* start){
    const char* pathfile = "/Users/michieldejong/Documents/Graduation/Simulator/dataset/path.txt";
    std::ofstream pf(pathfile);
    std::vector<Node*> path;
    for(auto e: visited){
        pf<<e->x<<" "<<e->y<<" "<<e->z<<" \n";
    }
    return  path;
}

std::vector<Node*> getSucc(Node *current, DynamicGrid &dynamicGrid, int voxelID){
    std::vector<Node*> nbrs;
    for(auto neighbours: current->getNeighbours26()){
        if ((neighbours.x && neighbours.y && neighbours.z >= 0) && neighbours.x <= dynamicGrid.max_x - 1 && neighbours.y <= dynamicGrid.max_y - 1 &&
            neighbours.z <= dynamicGrid.max_z - 1) {
            auto tmp = dynamicGrid(neighbours.x, neighbours.y, neighbours.z);
            if(tmp->id == 2) {
                tmp->k.first = -1;
                tmp->k.second = -1;
                if(tmp->id != voxelID)
                    nbrs.push_back(tmp);
            }
        }
    }
    return nbrs;
}




