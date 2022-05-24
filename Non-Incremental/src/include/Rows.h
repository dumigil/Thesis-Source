#ifndef Rows_h
#define Rows_h
#include <assert.h>

struct Rows {
    int x, y, z;
  
  Rows() {
    x = 0;
    y = 0;
    z = 0;
  }
  
  Rows(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  
  int& operator[](int coordinate) {
    if (coordinate == 0) return x;
    else if (coordinate == 1) return y;
    else if (coordinate == 2) return z;
    else assert(false);
  }
  
  int operator[](int coordinate) const {
    if (coordinate == 0) return x;
    else if (coordinate == 1) return y;
    else if (coordinate == 2) return z;
    else assert(false);
  }
  std::ostream& operator<<(std::ostream& os) {
        os << "(" << x << ", " << y << ", " << z << ")";
        return os;
  }
};



#endif /* Rows_h */
