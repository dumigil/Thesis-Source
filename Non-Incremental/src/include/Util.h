#ifndef UTIL_H
#define UTIL_H
#include <cmath>
#include <array>
#include <sys/time.h>
#include <sys/resource.h>
#include "Node.h"
#include "Octree.h"
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include "../libmorton/include/libmorton/morton.h"
#define eps (10e-10)
#define M_SQRT3 1.732050807568877293527446341505872366942805253810380628055806




static double distSquared(Voxel &a, Voxel& b);
static int phi(Voxel &a, Voxel&b, Voxel &c);
static double dist(Voxel &a, Voxel& b);
static float dist(Node *a, Node *b);
static double dist(OctreeNode *a, OctreeNode *b);
static double distSquared(Node *a, Node *b);
static double distSquared(OctreeNode *a, OctreeNode *b);

static int distManhattan(Voxel &a, Voxel &b);
static int distDiagonal(Voxel &a, Voxel &b);
static int distDiagonal(OctreeNode *a, OctreeNode *b);
static int distDiagonal(Node *a, Node *b);


/**
 * Returns the Euclidean distance between to two voxels
 * @param a Voxel a
 * @param b Voxel b
 * @return Euclidean distance
 */
static double dist(Voxel &a, Voxel &b) {
    return std::sqrt(distSquared(a,b));
}
/**
 * Returns the Euclidean distance between to two voxels
 * @param a Node a
 * @param b Node b
 * @return Euclidean distance
 */
static float dist(Node *a, Node *b) {
    return std::sqrt(distSquared(a,b));
}

/**
 * Returns the Euclidean distance between to two OctreeNodes
 * @param a OctreeNode a
 * @param b OctreeNode b
 * @return Euclidean distance
 */
static double dist(OctreeNode *a, OctreeNode *b) {
    return std::sqrt(distSquared(a,b));
}

/**
 * Returns the squared Euclidean distance between to two voxels
 * @param a Voxel a
 * @param b Voxel b
 * @return Euclidean distance squared
 */
static double distSquared(Voxel &a, Voxel &b) {
    return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}
/**
 * Returns the squared Euclidean distance between to two voxels
 * @param a Node a
 * @param b Node b
 * @return Euclidean distance squared
 */
static double distSquared(Node *a, Node *b) {
    return (a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y) + (a->z - b->z)*(a->z - b->z);
}

/**
 * Returns the squared Euclidean distance between to two voxels
 * @param a Node a
 * @param b Node b
 * @return Euclidean distance squared
 */
static double distSquared(OctreeNode *a, OctreeNode *b) {
    return (a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y) + (a->z - b->z)*(a->z - b->z);
}

/**
 * Returns the Manhattan distance between to two voxels
 * @param a Voxel a
 * @param b Voxel b
 * @return Manhattan distance (int)
 */
static int distManhattan(Voxel &a, Voxel &b){
    //Returns the Manhattan distance between to two voxels
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
}
/**
 * Returns the Manhattan distance between to two voxels
 * @param a Node a
 * @param b Node b
 * @return Manhattan distance (int)
 */
static int distManhattan(Node *a, Node *b){
    //Returns the Manhattan distance between to two nodes
    return std::abs(a->x - b->x) + std::abs(a->y - b->y) + std::abs(a->z - b->z);
}

/**
 * Returns the Manhattan distance between to two OctreeNodes
 * @param a OctreeNode a
 * @param b OctreeNode b
 * @return Manhattan distance (int)
 */
static int distManhattan(OctreeNode &a, OctreeNode &b){
    //Returns the Manhattan distance between to two nodes
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
}

/**
 * Returns the Manhattan distance between to two OctreeNodes
 * @param a OctreeNode a
 * @param b OctreeNode b
 * @return Manhattan distance (int)
 */
static int distManhattan(OctreeNode *a, OctreeNode *b){
    //Returns the Manhattan distance between to two nodes
    return std::abs(a->x - b->x) + std::abs(a->y - b->y) + std::abs(a->z - b->z);
}

static int distDiagonal(Voxel &a, Voxel &b){
    int dx = abs(a.x - b.x);
    int dy = abs(a.y - b.y);
    int dz = abs(a.z - b.z);
    int dmin = std::min({dx, dy, dz});
    int dmax = std::max({dx, dy, dz});
    int dmid = dx + dy + dz - dmin - dmax;
    int D1 = 1;
    double D2 = M_SQRT2;
    double D3 = M_SQRT3;
    return (D3 - D2) * dmin + (D2 - D1) * dmid + D1 * dmax;
}
static int distDiagonal(OctreeNode *a, OctreeNode *b){
    int dx = abs(a->x - b->x);
    int dy = abs(a->y - b->y);
    int dz = abs(a->z - b->z);
    int dmin = std::min({dx, dy, dz});
    int dmax = std::max({dx, dy, dz});
    int dmid = dx + dy + dz - dmin - dmax;
    int D1 = 1;
    double D2 = M_SQRT2;
    double D3 = M_SQRT3;
    return (D3 - D2) * dmin + (D2 - D1) * dmid + D1 * dmax;
}

static int distDiagonal(Node *a, Node *b){
    int dx = abs(a->x - b->x);
    int dy = abs(a->y - b->y);
    int dz = abs(a->z - b->z);
    int dmin = std::min({dx, dy, dz});
    int dmax = std::max({dx, dy, dz});
    int dmid = dx + dy + dz - dmin - dmax;
    int D1 = 1;
    double D2 = M_SQRT2;
    double D3 = M_SQRT3;
    return (D3 - D2) * dmin + (D2 - D1) * dmid + D1 * dmax;
}




/**
 * Computes Phi angle between 3 voxels
 * @param a Voxel a
 * @param b Voxel b
 * @param c Voxel c
 * @return Phi in degrees
 */
static int phi(Voxel &a, Voxel &b, Voxel &c) {
    int angle = static_cast<int>((180/M_PI) * (-atan2(a.z-b.z, a.x-b.x) + atan2(c.z-b.z, c.x-b.x)));
    if( angle > 180){
        angle = - (180 - (angle%180));// Set the angle between -180 and 180
    }
    return angle;
}
/**
 * Computes Phi angle between 3 voxels
 * @param a Node a
 * @param b Node b
 * @param c Node c
 * @return Phi in degrees
 */
static int phi(Node &a, Node &b, Node &c) {
    int angle = static_cast<int>((180/M_PI) * (-atan2(a.z-b.z, a.x-b.x) + atan2(c.z-b.z, c.x-b.x)));
    if( angle > 180){
        angle = - (180 - (angle%180));// Set the angle between -180 and 180
    }
    return angle;
}

/**
 * @param x an integer x
 * @return the sign of integer x
 */
static int sgn(int x) {
    //Returns the sign of an integer
    return (x > 0) - (x < 0);
}

/**
 * @param a Lower bound
 * @param b Upper bound
 * @return A random float between the values a and b
 */
static float randomFloat(float a, float b){
    //Returns a random float between one and zero
    float random = ((float) rand() / (float) RAND_MAX);
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

using namespace Magnum;
using namespace Math::Literals;

/**
 * Data struct to send to instance buffer.
 */
struct InstanceData {
    Matrix4 transformation;
    Matrix3x3 normalMatrix;
    Color3 color;
    int id{};
};


/**
 * Wrapper around std::priority_queuem adapted from https://www.redblobgames.com/.
 * @tparam T Voxel of Node
 * @tparam priority_t Priority value
 */
template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
            std::greater<PQElement>> elements;

    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    inline void erase(){
        while(!elements.empty()){
            elements.pop();
        }
    }

    priority_t top() {
        priority_t best_item = elements.top().first;
        return best_item;
    }

    T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

/**
 * A simple 3 dimensional integer vector
 */
struct Vec3i{
    uint_fast32_t x, y, z;
};
/**
 * A simple 3 dimensional float vector
 */
struct Vec3f{
    float x, y, z;
};

/**
 * Lookup table for fast neighbour access. Implementation used from
 * https://bitbucket.org/volumesoffun/polyvox/issue/61/experiment-with-morton-ordering-of-voxel
 */
namespace lookup {
    static const std::array<int32_t, 256> deltaX = {1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 3511, 1, 7, 1, 55,
                                                    1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 28087, 1, 7, 1, 55, 1, 7, 1,
                                                    439, 1, 7, 1, 55, 1, 7, 1, 3511, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1,
                                                    55, 1, 7, 1, 224695, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7,
                                                    1, 3511, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 28087, 1,
                                                    7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 3511, 1, 7, 1, 55, 1,
                                                    7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 1797559, 1, 7, 1, 55, 1, 7, 1, 439,
                                                    1, 7, 1, 55, 1, 7, 1, 3511, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55,
                                                    1, 7, 1, 28087, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1,
                                                    3511, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 224695, 1, 7,
                                                    1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1, 7, 1, 3511, 1, 7, 1, 55, 1, 7,
                                                    1, 439, 1, 7, 1, 55, 1, 7, 1, 28087, 1, 7, 1, 55, 1, 7, 1, 439, 1,
                                                    7, 1, 55, 1, 7, 1, 3511, 1, 7, 1, 55, 1, 7, 1, 439, 1, 7, 1, 55, 1,
                                                    7, 1};
    static const std::array<int32_t, 256> deltaY = {2, 14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 7022, 2, 14,
                                                    2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 56174, 2, 14, 2,
                                                    110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 7022, 2, 14, 2, 110, 2,
                                                    14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 449390, 2, 14, 2, 110, 2, 14,
                                                    2, 878, 2, 14, 2, 110, 2, 14, 2, 7022, 2, 14, 2, 110, 2, 14, 2, 878,
                                                    2, 14, 2, 110, 2, 14, 2, 56174, 2, 14, 2, 110, 2, 14, 2, 878, 2, 14,
                                                    2, 110, 2, 14, 2, 7022, 2, 14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110,
                                                    2, 14, 2, 3595118, 2, 14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2,
                                                    14, 2, 7022, 2, 14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2,
                                                    56174, 2, 14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 7022,
                                                    2, 14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 449390, 2,
                                                    14, 2, 110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 7022, 2, 14, 2,
                                                    110, 2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 56174, 2, 14, 2, 110,
                                                    2, 14, 2, 878, 2, 14, 2, 110, 2, 14, 2, 7022, 2, 14, 2, 110, 2, 14,
                                                    2, 878, 2, 14, 2, 110, 2, 14, 2};
    static const std::array<int32_t, 256> deltaZ = {4, 28, 4, 220, 4, 28, 4, 1756, 4, 28, 4, 220, 4, 28, 4, 14044, 4,
                                                    28, 4, 220, 4, 28, 4, 1756, 4, 28, 4, 220, 4, 28, 4, 112348, 4, 28,
                                                    4, 220, 4, 28, 4, 1756, 4, 28, 4, 220, 4, 28, 4, 14044, 4, 28, 4,
                                                    220, 4, 28, 4, 1756, 4, 28, 4, 220, 4, 28, 4, 898780, 4, 28, 4, 220,
                                                    4, 28, 4, 1756, 4, 28, 4, 220, 4, 28, 4, 14044, 4, 28, 4, 220, 4,
                                                    28, 4, 1756, 4, 28, 4, 220, 4, 28, 4, 112348, 4, 28, 4, 220, 4, 28,
                                                    4, 1756, 4, 28, 4, 220, 4, 28, 4, 14044, 4, 28, 4, 220, 4, 28, 4,
                                                    1756, 4, 28, 4, 220, 4, 28, 4, 7190236, 4, 28, 4, 220, 4, 28, 4,
                                                    1756, 4, 28, 4, 220, 4, 28, 4, 14044, 4, 28, 4, 220, 4, 28, 4, 1756,
                                                    4, 28, 4, 220, 4, 28, 4, 112348, 4, 28, 4, 220, 4, 28, 4, 1756, 4,
                                                    28, 4, 220, 4, 28, 4, 14044, 4, 28, 4, 220, 4, 28, 4, 1756, 4, 28,
                                                    4, 220, 4, 28, 4, 898780, 4, 28, 4, 220, 4, 28, 4, 1756, 4, 28, 4,
                                                    220, 4, 28, 4, 14044, 4, 28, 4, 220, 4, 28, 4, 1756, 4, 28, 4, 220,
                                                    4, 28, 4, 112348, 4, 28, 4, 220, 4, 28, 4, 1756, 4, 28, 4, 220, 4,
                                                    28, 4, 14044, 4, 28, 4, 220, 4, 28, 4, 1756, 4, 28, 4, 220, 4, 28,
                                                    4};
}
/**
 * Get morton code of 1 voxel away from voxel in x direction
 * @param x x coordinate of voxel
 * @param index morton code of voxel
 * @param positive boolean if positive or negative
 * @return morton code of neighbour
 */
static uint_fast64_t getNeighbour_morton_x(int x,uint_fast64_t index, bool positive){
    if(positive) {
        return index + lookup::deltaX[x];
    }
    else{
        auto dx = x - 1;
        if(dx < 0) return index;
        return index - lookup::deltaX[dx];
    }
}

/**
 * Get morton code of 1 voxel away from voxel in y direction
 * @param x y coordinate of voxel
 * @param index morton code of voxel
 * @param positive boolean if positive or negative
 * @return morton code of neighbour
 */
static uint_fast64_t getNeighbour_morton_y(int y,uint_fast64_t index, bool positive){
    if(positive) {
        return index + lookup::deltaY[y];
    }
    else{
        auto dy = y - 1;
        if(dy < 0) return index;
        return index - lookup::deltaY[dy];
    }
}

/**
 * Get morton code of 1 voxel away from voxel in z direction
 * @param x z coordinate of voxel
 * @param index morton code of voxel
 * @param positive boolean if positive or negative
 * @return morton code of neighbour
 */
static uint_fast64_t getNeighbour_morton_z(int z,uint_fast64_t index, bool positive){
    if(positive) {
        return index + lookup::deltaZ[z];
    }
    else{
        auto dz = z - 1;
        if(dz < 0) return index;
        return index - lookup::deltaZ[dz];
    }
}

/**
 * Get 6 connectivity neighbours of a voxel.
 *
 * @param x x coordinate of voxel
 * @param y y coordinate of voxel
 * @param z z coordinate of voxel
 * @param index morton code of voxel
 * @return vector of morton codes of all 6 neighbours
 */
static std::vector<uint_fast64_t> getNeighbours6_morton(int x, int y, int z, uint_fast64_t index)  {
    std::vector<uint_fast64_t> neighbours = {
            getNeighbour_morton_x(x, index, true),
            getNeighbour_morton_y(y, index, true),
            getNeighbour_morton_z(z, index, true),
            getNeighbour_morton_x(x, index, false),
            getNeighbour_morton_y(y, index, false),
            getNeighbour_morton_z(z, index, false)
    };
    return neighbours;
}
/**
 * Get 18 connectivity neighbours of a voxel.
 *
 * @param x x coordinate of voxel
 * @param y y coordinate of voxel
 * @param z z coordinate of voxel
 * @param index morton code of voxel
 * @return vector of morton codes of all 6 neighbours
 */
static std::vector<uint_fast64_t> getNeighbours18_morton(int x, int y, int z, uint_fast64_t index)  {
    auto x_plus_1 = getNeighbour_morton_x(x, index, true);
    auto y_plus_1 = getNeighbour_morton_y(y, index, true);
    auto z_plus_1 = getNeighbour_morton_z(z, index, true);
    auto x_min_1 = getNeighbour_morton_x(x, index, false);
    auto y_min_1 = getNeighbour_morton_y(y, index, false);
    auto z_min_1 = getNeighbour_morton_z(z, index, false);

    auto x_plus_1_y_plus_1 = getNeighbour_morton_y(y, x_plus_1, true);
    auto x_plus_1_y_min_1 = getNeighbour_morton_y(y, x_plus_1, false);

    auto x_min_1_y_plus_1 = getNeighbour_morton_y(y, x_min_1, true);
    auto x_min_1_y_min_1 = getNeighbour_morton_y(y, x_min_1, false);

    auto y_plus_1_z_plus_1 = getNeighbour_morton_z(z, y_plus_1, true);
    auto y_plus_1_z_min_1 = getNeighbour_morton_z(z, y_plus_1, false);

    auto y_min_1_z_plus_1 = getNeighbour_morton_z(z, y_min_1, true);
    auto y_min_1_z_min_1 = getNeighbour_morton_z(z, y_min_1, false);

    auto z_plus_1_x_plus_1 = getNeighbour_morton_x(x, z_plus_1, true);
    auto z_plus_1_x_min_1 = getNeighbour_morton_x(x, z_plus_1, false);

    auto z_min_1_x_plus_1 = getNeighbour_morton_x(x, z_min_1, true);
    auto z_min_1_x_min_1 = getNeighbour_morton_x(x, z_min_1, false);

    std::vector<uint_fast64_t> neighbours = {
            x_plus_1,
            y_plus_1,
            z_plus_1,
            x_min_1,
            y_min_1,
            z_min_1,
            x_plus_1_y_plus_1,
            x_plus_1_y_min_1,
            x_min_1_y_plus_1,
            x_min_1_y_min_1,
            y_plus_1_z_plus_1,
            y_plus_1_z_min_1,
            y_min_1_z_plus_1,
            y_min_1_z_min_1,
            z_plus_1_x_plus_1,
            z_plus_1_x_min_1,
            z_min_1_x_plus_1,
            z_min_1_x_min_1
    };
    return neighbours;
}


/**
 * Get 26 connectivity neighbours of a voxel.
 *
 * @todo
 * @param x x coordinate of voxel
 * @param y y coordinate of voxel
 * @param z z coordinate of voxel
 * @param index morton code of voxel
 * @return vector of morton codes of all 26 neighbours
 */
static std::vector<uint_fast64_t> getNeighbours26_morton(int x, int y, int z, uint_fast64_t index)  {
    auto x_plus_1 = getNeighbour_morton_x(x, index, true);
    auto y_plus_1 = getNeighbour_morton_y(y, index, true);
    auto z_plus_1 = getNeighbour_morton_z(z, index, true);
    auto x_min_1 = getNeighbour_morton_x(x, index, false);
    auto y_min_1 = getNeighbour_morton_y(y, index, false);
    auto z_min_1 = getNeighbour_morton_z(z, index, false);

    auto x_plus_1_y_plus_1 = getNeighbour_morton_y(y, x_plus_1, true);
    auto x_plus_1_y_min_1 = getNeighbour_morton_y(y, x_plus_1, false);

    auto x_min_1_y_plus_1 = getNeighbour_morton_y(y, x_min_1, true);
    auto x_min_1_y_min_1 = getNeighbour_morton_y(y, x_min_1, false);

    auto y_plus_1_z_plus_1 = getNeighbour_morton_z(z, y_plus_1, true);
    auto y_plus_1_z_min_1 = getNeighbour_morton_z(z, y_plus_1, false);

    auto y_min_1_z_plus_1 = getNeighbour_morton_z(z, y_min_1, true);
    auto y_min_1_z_min_1 = getNeighbour_morton_z(z, y_min_1, false);

    auto z_plus_1_x_plus_1 = getNeighbour_morton_x(x, z_plus_1, true);
    auto z_plus_1_x_min_1 = getNeighbour_morton_x(x, z_plus_1, false);

    auto z_min_1_x_plus_1 = getNeighbour_morton_x(x, z_min_1, true);
    auto z_min_1_x_min_1 = getNeighbour_morton_x(x, z_min_1, false);

    auto z_min_1_x_plus_1_y_plus_1 = getNeighbour_morton_y(y,z_min_1_x_plus_1, true);
    auto z_min_1_x_plus_1_y_min_1 = getNeighbour_morton_y(y, z_min_1_x_plus_1, false);

    auto z_min_1_x_min_1_y_plus_1 = getNeighbour_morton_y(y,z_min_1_x_min_1, true);
    auto z_min_1_x_min_1_y_min_1 = getNeighbour_morton_y(y, z_min_1_x_min_1, false);

    auto z_plus_1_x_plus_1_y_plus_1 = getNeighbour_morton_y(y,z_plus_1_x_plus_1, true);
    auto z_plus_1_x_plus_1_y_min_1 = getNeighbour_morton_y(y, z_plus_1_x_plus_1, false);

    auto z_plus_1_x_min_1_y_plus_1 = getNeighbour_morton_y(y,z_plus_1_x_min_1, true);
    auto z_plus_1_x_min_1_y_min_1 = getNeighbour_morton_y(y, z_plus_1_x_min_1, false);

    std::vector<uint_fast64_t> neighbours = {
            x_plus_1,
            y_plus_1,
            z_plus_1,
            x_min_1,
            y_min_1,
            z_min_1,
            x_plus_1_y_plus_1,
            x_plus_1_y_min_1,
            x_min_1_y_plus_1,
            x_min_1_y_min_1,
            y_plus_1_z_plus_1,
            y_plus_1_z_min_1,
            y_min_1_z_plus_1,
            y_min_1_z_min_1,
            z_plus_1_x_plus_1,
            z_plus_1_x_min_1,
            z_min_1_x_plus_1,
            z_min_1_x_min_1,
            z_min_1_x_plus_1_y_plus_1,
            z_min_1_x_plus_1_y_min_1,
            z_min_1_x_min_1_y_plus_1,
            z_min_1_x_min_1_y_min_1,
            z_plus_1_x_plus_1_y_plus_1,
            z_plus_1_x_plus_1_y_min_1,
            z_plus_1_x_min_1_y_plus_1,
            z_plus_1_x_min_1_y_min_1
    };
    return neighbours;
}


/**
 * Function to check if the path generated by pathfinding is an actual path, or not.
 *
 * @param path path generated by algorithm (std::vector<typename T>)
 * @return true if the path is longer than only the start and finish nodes
 */
static std::atomic_bool succesfulPath(std::vector<OctreeNode*> &path){
    if(path.size() <3 ){
        return false;
    } else {
        return true;
    }
}

static std::atomic_bool succesfulPath(std::deque<OctreeNode*> &path){
    if(path.size() <3 ){
        return false;
    } else {
        return true;
    }
}


/**
 * Function to check if x and y are within 10e-10.
 * @param x
 * @param y
 * @return true if within 10e-10, false otherwise
 */
static bool near(float x, float y) {

    if (isinf(x) && isinf(y)) return true;
    return (fabs(x-y) < eps);

}


struct byGreat{
    bool operator()(const Node &s1,const Node &s2)  {
        if (s1.k.first-eps > s2.k.first) return true;
        else if (s1.k.first < s2.k.first-eps) return false;
        return s1.k.second > s2.k.second;
    }
};


static bool Node_isLessEq(const Node &s1,const Node &s2)  {
    if (s1.k.first < s2.k.first) return true;
    else if (s1.k.first > s2.k.first) return false;
    return s1.k.second < s2.k.second + eps;
}

struct byLess{
    bool operator()(const Node *s1,const Node *s2)  {
        if (s1->k.first + eps < s2->k.first) return true;
        else if (s1->k.first - eps > s2->k.first) return false;
        return s1->k.second < s2->k.second;
    }
};



#endif /* util_h */