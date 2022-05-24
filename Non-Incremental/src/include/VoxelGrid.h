#ifndef VoxelGrid_h
#define VoxelGrid_h
#define eps (10e-10)
#include <assert.h>
#include <vector>
#include <iostream>

struct VoxelGrid {
    /**
     * This class represents a simple dense uniform voxelgrid. Fast access to the voxel data is provided
     * by using the indices of the voxel, which returns the value at that index. This is used in conjunction with the
     * classes defined in the Voxel.h and Node.h files, which provide access to the geometric aspect of the voxels.
     *
     * The original VoxelGrid.h file was provided by the GEO1004 course.
     */
  std::vector<int> voxels;
  int max_x, max_y, max_z;
  float mVoxelsize;
  bool tilted;

  VoxelGrid( int x,  int y,  int z) {
    max_x = x;
    max_y = y;
    max_z = z;

    int total_voxels = x*y*z;
    voxels.reserve(total_voxels);
    for (int i = 0; i < total_voxels; ++i) voxels.push_back(0);
  }

  int &operator()(const int &x, const int &y, const int &z) {
    assert(x >= 0 && x < max_x);
    assert(y >= 0 && y < max_y);
    assert(z >= 0 && z < max_z);
    return voxels[x + y*max_x + z*max_x*max_y];
  }

  int operator()(const int &x, const int &y, const int &z) const {
    assert(x >= 0 && x < max_x);
    assert(y >= 0 && y < max_y);
    assert(z >= 0 && z < max_z);
    return voxels[x + y*max_x + z*max_x*max_y];
  }

};





#endif /* VoxelGrid_h */
