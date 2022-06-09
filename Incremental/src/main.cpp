#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <queue>
#include <map>
#include "include/VoxelGrid.h"
#include "include/Rows.h"
#include "include/Voxel.h"
#include "include/Octree.h"
#include "include/Util.h"

bool TILTED = false;

int runSimulator(VoxelGrid &voxels, SparseVoxelOctree &svo, int argc, char** argv);
void floodFill(VoxelGrid &voxels);
void extractWalkableSpace(VoxelGrid &voxels);
void printFilledVoxels(VoxelGrid &voxelGrid, const char* file_out, int attribute);
std::queue<uint_fast64_t> generateMorton(VoxelGrid &voxels);
std::map<uint_fast64_t, OctreeNode *>
createLevel(std::map<uint_fast64_t, OctreeNode *> &buffer, std::map<uint_fast64_t, OctreeNode *> &level, int nLevel);
SparseVoxelOctree createSVO(std::queue<uint_fast64_t> &mLeafs);
std::map<uint_fast64_t, OctreeNode*> createFirstLevel(std::queue<uint_fast64_t> &buffer, std::map<uint_fast64_t , OctreeNode*> &level);
void createQuickAccessTree(SparseVoxelOctree &octree);
Vec3f getMetaDataOBJ(const char* filename);

int main(int argc, char** argv){
    std::vector<Voxel> _dataPoints;
    Rows rows;
    const char* file;
    const char *meshfile;

    if(TILTED){
        rows = Rows(228, 85, 256);
        file = "/Users/michieldejong/Documents/Graduation/Simulator_lpa/dataset_tilted/dataset_1_tilted_228x85x256.txt";
        meshfile = "/Users/michieldejong/Documents/Graduation/Simulator_lpa/dataset_tilted/dataset_1_tilted_228x85x256_fixed.obj";
    }else{
        rows = Rows(163, 101, 256);
        file = "/Users/michieldejong/Documents/Graduation/Simulator_lpa/dataset/dataset_2_163x101x256.txt";
        meshfile = "/Users/michieldejong/Documents/Graduation/Simulator_lpa/dataset/dataset_163_101_256_fixed_2.obj";
    }

    VoxelGrid voxelGrid(rows.x, rows.y, rows.z);
    const char *out = "/Users/michieldejong/Documents/Graduation/Simulator_lpa/dataset/walkable.txt";


    std::ifstream infile(file);
    std::ofstream outfile(out);

    if(!infile){
        std::cerr<<"Input file not found.\n";
    }
    if(!infile.is_open()) throw std::runtime_error("Could not open file");
    std::cout<<"*** Creating voxel grid ***\n";
    std::cout<<"Reading file: "<< file << std::endl;
    std::string line;
    float val;
    while(std::getline(infile, line)){
        std::stringstream ss(line);
        int idx = 0;
        std::vector<float> pt;
        while(ss >> val){
            pt.push_back(val);
            if(ss.peek() == ',') ss.ignore();
            idx++;
        }
        Voxel vertex = Voxel{pt[0], pt[1], pt[2]};
        _dataPoints.push_back(vertex);
    }
    int count = 0;
    for(auto &all: _dataPoints){
        voxelGrid(all.x, all.y, all.z) = 1;
        count++;
    }
    std::cout<<"Extracting metadata from "<<meshfile<<"\n";
    auto bbox = getMetaDataOBJ(meshfile);
    auto vSize = bbox.x / (rows.x + 1);

    voxelGrid.mVoxelsize = vSize;
    if(TILTED) voxelGrid.tilted = true;


    std::cout<<vSize<<"\n";
    std::cout<<count<<" solid voxels\n";

    floodFill(voxelGrid);

    extractWalkableSpace(voxelGrid);

    //printFilledVoxels(voxelGrid, out, 2);

    /*** Creating the Sparse Voxel Octree (initial) ***/
    auto start = std::chrono::high_resolution_clock::now();
    std::cout<<"*** Building Sparse Voxel Octree ***\n";


    auto mortonStack = generateMorton(voxelGrid);

    auto octree = createSVO(mortonStack);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout<<"Built Sparse Voxel Octree in "<<diff.count()<<" seconds\n";
    std::cout<<"The SVO has a depth of "<<octree.getDepth()<<"\n";
    std::cout<<"*** Building SVO done *** \n";
    std::cout<<" \n";
    std::cout<<"Starting application...\n";



    runSimulator(voxelGrid, octree, argc, argv);


    return 0;
}
/**
 * Function that fills the hollow walls/floors/ceilings in a VoxelGrid
 *
 * @param voxels a VoxelGrid with the (hollow) walls being set to 1.
 */
void floodFill(VoxelGrid &voxels) {
    /** Fill internal voxels **/
    Voxel seed;
    if(TILTED){
        seed = Voxel(7, 1, 79);

    }else{
        seed = Voxel(29, 1, 23);
    }
    std::stack<Voxel> stack;
    assert(stack.empty());
    stack.push(seed);
    int fillCount = 0;
    while(!stack.empty()){
        Voxel current = stack.top();
        stack.pop();
        voxels(current.x, current.y,current.z) =1;
        fillCount++;
        for(auto &p: current.getNeighbours6()){
            if((p.x && p.y && p.z >= 0) && p.x <= voxels.max_x - 1 && p.y <= voxels.max_y - 1 && p.z <= voxels.max_z - 1){
                if(voxels(p.x, p.y, p.z) != 1){
                    stack.push(p);
                }else{
                    continue;
                }
            }
        }
    }
    std::cout<<fillCount<<" extra solid voxels have been detected\n";
}
/**
 * Prints coordinates of voxels that satisfy a certain parameter.
 *
 * @param voxelGrid VoxelGrid created from point cloud.
 * @param file_out Path to output file on.
 * @param attribute Which voxels to print (0 is air, 1 is wall, 2 is walkable, 3 is fire, etc.).
 */
void printFilledVoxels(VoxelGrid &voxelGrid, const char* file_out, int attribute){
    std::ofstream outfile(file_out, std::ofstream::out);
    for (int i=0; i< voxelGrid.max_x-1; i++) {
        for (int j = 0; j < voxelGrid.max_y-1; j++) {
            for (int k = 0; k < voxelGrid.max_z-1; k++) {
                if (voxelGrid(i, j, k) == attribute) {
                    outfile<< i <<" "<<j<<" "<<k<<std::endl;
                }
            }
        }
    }
    outfile.close();
    std::cout<<"Filled voxels written to "<<file_out<<std::endl;
}
/**
 * Extracts voxels that are walkable (i.e. voxels that have ~2m of air above them.
 *
 * @param voxels VoxelGrid. This VoxelGrid is semantically enriched by adding the walkable space parameter to voxels
 * that are walkable.
 */
void extractWalkableSpace(VoxelGrid &voxels){
    int count = 0;
    for ( int x = 0; x < voxels.max_x-1; ++x) {
        for ( int y = 0; y < voxels.max_y-25; ++y) {
            for ( int z = 0; z < voxels.max_z-1; ++z) {
                if(voxels(x,y,z)==1) {
                    bool space = true;
                    if(TILTED){
                        for(int i =1; i <15; i++){
                            if(voxels(x, (y + i), z) != 0) {
                                space = false;
                            }
                        }
                    }else{
                        for(int i =1; i <22; i++){
                            if(voxels(x, (y + i), z) != 0) {
                                space = false;
                            }
                        }
                    }
                    if(space) {
                        if (voxels(x, y + 1, z) == 0) {
                            voxels(x, (y + 1), z) = 2;
                        }
                        if (voxels(x, y + 2, z) == 0) {
                            voxels(x, (y + 2), z) = 2;
                        }
                        if (voxels(x, y + 3, z) == 0) {
                            voxels(x, (y + 3), z) = 2;
                        }
                        count += 3;
                    }
                }
            }
        }
    }
    std::cout<<count<<" walkable voxels extracted\n";
    std::cout<<"*** Building voxelgrid done ***\n";
    std::cout<<" \n";
}
/**
 * Generates morton codes for all the walkable voxels in the VoxelGrid.
 *
 * @param voxels A voxelgrid that has been semantically enriched to know which voxels are walkable an which aren't.
 * @return returns a queue of Morton Codes that have been ordered (ascending).
 */
std::queue<uint_fast64_t> generateMorton(VoxelGrid &voxels) {
    std::vector<uint_fast64_t> codes;
    std::queue<uint_fast64_t> mortonStack;
    for (int i=0; i< voxels.max_x-1; i++) {
        for (int j = 0; j < voxels.max_y-1; j++) {
            for (int k = 0; k < voxels.max_z-1; k++) {
                //if(attribute == 3){std::cout<<voxelGrid(i, j, k)<<std::endl;}
                if (voxels(i, j, k) == 2) {
                    auto code = libmorton::morton3D_64_encode(i, j, k);
                    codes.emplace_back(code);

                }
            }
        }
    }
    std::sort(codes.begin(), codes.end());
    for(auto all: codes){
        mortonStack.push(all);
    }
    return mortonStack;
}

/**
 * Creates a SVO from a queue of Morton codes (these are the leaf nodes).
 *
 * @param mLeafs input queue of sorted Morton codes (leaf nodes) to build octree from (bottoms up)
 * @return a vector of maps, with each map containing the level of the octree, with every map sorted by Morton key for
 * easy access.
 */
SparseVoxelOctree createSVO(std::queue<uint_fast64_t> &mLeafs) {
    SparseVoxelOctree octree;
    std::map<uint_fast64_t, OctreeNode *> level;
    auto parents = createFirstLevel(mLeafs, level);
    octree.mTree.push_back(level);
    int n = 1;
    while(parents.size() != 1){
        std::map<uint_fast64_t, OctreeNode *> lv;
        parents = createLevel(parents, lv, n);
        octree.mTree.push_back(lv);
        n++;
    }
    std::map<uint_fast64_t, OctreeNode *> lv;
    octree.mTree.push_back(parents);



    /*** Setting nodes to full if *all* their children are full. This way, with pathfinding, we can safely assume that
     * this entire subtree is full. For now we will not delete the nodes in the full subtree, but this could be
     * something to do. ***/
    for(int n =0; n < octree.getDepth(); n++) {
        for(const auto &nodes: octree.mTree[n]){
            if(nodes.second->Parent == nullptr) continue;
            if(nodes.second->Parent->isFull) continue;
            if((nodes.second->Parent->getFull() && nodes.second->isLeaf) ||
            nodes.second->Parent->getFull() && nodes.second->isFull){
                nodes.second->Parent->isFull = true;
            }
        }
    }
    createQuickAccessTree(octree);
    return octree;
}




/**
 * Creates the leaf (first) level of the SVO.
 *
 * @param buffer queue of ordered Morton codes used to generate a the leaf level of a Sparse Voxel Octree
 * @param level an empty map to put the created level in
 * @return a parent level in the form of a map of Morton Codes and OctreeNode*
 */
std::map<uint_fast64_t, OctreeNode*> createFirstLevel(std::queue<uint_fast64_t> &buffer, std::map<uint_fast64_t , OctreeNode*> &level) {
    std::map<uint_fast64_t, OctreeNode*> parents;
    uint_fast64_t n = 0;
    uint_fast64_t index = 0;
    while(!buffer.empty()) {
        bool empty = true;
        auto parent = new OctreeNode();
        parent->index = index;

        for (int i = 0; i < 8; i++) {
            auto tmp = buffer.front();
            if(tmp == n) {
                auto * leaf = new OctreeNode();
                leaf->Parent = parent;
                parent->Children[i] = leaf;
                leaf->index = tmp;
                leaf->isLeaf = true;
                leaf->attribute = 2;
                leaf->level = 1;
                uint_fast32_t  x, y, z;
                libmorton::morton3D_64_decode(tmp, x, y, z);
                leaf->x = x;leaf->y = y;leaf->z = z;
                buffer.pop();
                level[tmp] = leaf;
                empty = false;
            }
            n++;
        }
        if(!empty) {
            parents[index] = parent;
            uint_fast32_t  x, y, z;
            libmorton::morton3D_64_decode(index, x, y, z);
            parent->x = x;parent->y = y;parent->z = z;
        } else {
            delete parent;
        }
        index ++;

    }
    return parents;
}
/**
 * Creates a non-leaf level of the SVO
 *
 * @param buffer the level below.
 * @param level the level being created
 * @return the level above
 */
std::map<uint_fast64_t, OctreeNode *>
createLevel(std::map<uint_fast64_t, OctreeNode *> &buffer, std::map<uint_fast64_t, OctreeNode *> &level, int nLevel) {
    std::map<uint_fast64_t, OctreeNode*> parents;
    uint_fast64_t n = 0;
    uint_fast64_t index = 0;
    int lParent, lLeaf;
    lLeaf = pow(2, nLevel);
    lParent = pow(2, nLevel + 1);
    while(!buffer.empty()) {
        bool empty = true;
        auto parent = new OctreeNode();
        parent->index = index;

        for (int i = 0; i < 8; i++) {
            auto tmp = buffer.begin()->first;
            if(tmp == n) {
                auto * leaf = buffer.begin()->second;
                leaf->Parent = parent;
                parent->Children[i] = leaf;
                leaf->index = tmp;
                leaf->isLeaf = false;
                leaf->attribute = 2;
                leaf->level = lLeaf;
                uint_fast32_t  x, y, z;
                libmorton::morton3D_64_decode(tmp, x, y, z);
                leaf->x = x;leaf->y = y;leaf->z = z;
                buffer.erase(buffer.begin());
                level[tmp] = leaf;
                empty = false;
            }
            n++;
        }
        if(!empty) {
            parents[index] = parent;
            uint_fast32_t  x, y, z;
            libmorton::morton3D_64_decode(index, x, y, z);
            parent->x = x;parent->y = y;parent->z = z;
        } else {
            delete parent;
        }
        index ++;

    }
    return parents;
}

void createQuickAccessTree(SparseVoxelOctree &octree){
    int n = 0;
    while(n < octree.getDepth()) {
        std::unordered_map<uint_fast64_t, OctreeNode*> level;
        for (auto all: octree.mTree[n]) {
            level.emplace(all);
        }
        octree.mQTree.push_back(level);
        n++;
    }
}

/**
 *
 * @param filename path to .obj file to get metadata from
 * @return the max values in all axis direction (to be used for voxelsize calculation).
 */
Vec3f getMetaDataOBJ(const char* filename){
    std::ifstream infile(filename);
    std::string line= "";
    float minX, minY, minZ, maxX, maxY, maxZ;
    while (std::getline(infile, line)) {
        std::stringstream ss;
        ss.clear();
        ss.str(line);
        std::string prefix = "";
        ss >> prefix;
        if (prefix == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            if (z<minZ){minZ = z;}
            if (z>maxZ){maxZ = z;}
            if (x<minX){minX = x;}
            if (x>maxX){maxX = x;}
            if (y<minY){minY = y;}
            if (y>maxY){maxY = y;}
        }
    }
    return {maxX, maxY, maxZ};
}








