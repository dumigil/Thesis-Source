#ifndef Voxel_h
#define Voxel_h
#include <cassert>
#include <vector>
struct Voxel {
    /**
     * This class represents a voxel, with integers x, y and z representing the index of the voxel in the voxel grid.
     * Furthermore, the class defines some useful functions for simple voxel arithmetic, and getting the neighbours of
     * a voxel (depending on which level of connectivity you require).
     */
    int x, y, z, id;

    Voxel() {
        x = 0;
        y = 0;
        z = 0;
        id = 0;
    }

    Voxel(const int &x, const int &y, const int &z) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->id = 0;
    }

    int &operator[](const int &coordinate) {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

    int operator[](const int &coordinate) const {
        if (coordinate == 0) return x;
        else if (coordinate == 1) return y;
        else if (coordinate == 2) return z;
        else
            assert(false);
    }

    Voxel operator+(const Voxel &other) const {
        return {x + other.x, y + other.y, z + other.z};
    }

    Voxel operator-(const Voxel &other) const {
        return {x - other.x, y - other.y, z - other.z};
    }

    Voxel operator*(const float &other) const {
        return {x * other, y * other, z * other};
    }

    Voxel operator*(const int &other) const {
        return {x * other, y * other, z * other};
    }


    Voxel operator/(const int &other) const {
        return {x / other, y / other, z / other};
    }

    bool operator<(const Voxel &other) const {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
        //return x < other.x || y < other.y || z < other.z;
    }

    bool operator>(const Voxel &other) const {
        return std::tie(x, y, z) > std::tie(other.x, other.y, other.z);
        //return x > other.x || y > other.y || z > other.z;
    }

    bool operator==(const Voxel &other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const Voxel &other) const {
        return x != other.x || y != other.y || z != other.z;
    }

    int dot(const Voxel &other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    /*
    std::ostream& operator<<(std::ostream& os) const {
        os << "(" << x << ", " << y << ", " << z << ")";
        return os;
    }
     */

    Voxel cross(const Voxel &other) const {
        return {y * other.z - z * other.y, -(x * other.z - z * other.x), x * other.y - y * other.x};
    }

    std::vector<Voxel> getNeighbours26() const {
        std::vector<Voxel> neighbours =
                {
                        //1
                        Voxel(x + 1, y + 1, z + 0),
                        //2
                        Voxel(x + 1, y - 1, z + 0),
                        //3
                        Voxel(x + 1, y + 1, z + 1),
                        //4
                        Voxel(x + 1, y + 0, z + 1),
                        //5
                        Voxel(x + 1, y - 1, z + 1),
                        //6
                        Voxel(x + 1, y + 1, z - 1),
                        //7
                        Voxel(x + 1, y + 0, z - 1),
                        //8
                        Voxel(x + 1, y - 1, z - 1),
                        //9
                        Voxel(x + 1, y + 0, z + 0),
                        //10
                        Voxel(x + 0, y + 1, z + 0),
                        //11
                        Voxel(x + 0, y - 1, z + 0),
                        //12
                        Voxel(x + 0, y + 1, z + 1),
                        //13
                        Voxel(x + 0, y + 0, z + 1),
                        //14
                        Voxel(x + 0, y - 1, z + 1),
                        //15
                        Voxel(x + 0, y + 1, z - 1),
                        //16
                        Voxel(x + 0, y + 0, z - 1),
                        //17
                        Voxel(x + 0, y - 1, z - 1),
                        //18
                        Voxel(x - 1, y + 1, z + 0),
                        //19
                        Voxel(x - 1, y - 1, z + 0),
                        //20
                        Voxel(x - 1, y + 1, z + 1),
                        //21
                        Voxel(x - 1, y + 0, z + 1),
                        //22
                        Voxel(x - 1, y - 1, z + 1),
                        //23
                        Voxel(x - 1, y + 1, z - 1),
                        //24
                        Voxel(x - 1, y + 0, z - 1),
                        //25
                        Voxel(x - 1, y - 1, z - 1),
                        //26
                        Voxel(x - 1, y + 0, z + 0)
                };
        return neighbours;
    }

    std::vector<Voxel> getNeighbours18() const {
        std::vector<Voxel> neighbours = {
                //1
                Voxel(x + 1, y + 1, z + 0),
                //2
                Voxel(x + 1, y - 1, z + 0),
                //3
                Voxel(x + 1, y + 0, z + 1),
                //4
                Voxel(x + 1, y + 0, z - 1),
                //5
                Voxel(x + 1, y + 0, z + 0),
                //6
                Voxel(x + 0, y + 1, z + 0),
                //7
                Voxel(x + 0, y - 1, z + 0),
                //8
                Voxel(x + 0, y + 1, z + 1),
                //9
                Voxel(x + 0, y + 0, z + 1),
                //10
                Voxel(x + 0, y - 1, z + 1),
                //11
                Voxel(x + 0, y + 1, z - 1),
                //12
                Voxel(x + 0, y + 0, z - 1),
                //13
                Voxel(x + 0, y - 1, z - 1),
                //14
                Voxel(x - 1, y + 1, z + 0),
                //15
                Voxel(x - 1, y - 1, z + 0),
                //16
                Voxel(x - 1, y + 0, z + 1),
                //17
                Voxel(x - 1, y + 0, z - 1),
                //18
                Voxel(x - 1, y + 0, z + 0)
        };
        return neighbours;
    }

    std::vector<Voxel> getNeighbours6() const {
        std::vector<Voxel> neighbours = {
                //1
                Voxel(x + 1, y + 0, z + 0),
                //2
                Voxel(x + 0, y + 1, z + 0),
                //3
                Voxel(x + 0, y - 1, z + 0),
                //4
                Voxel(x + 0, y + 0, z + 1),
                //5
                Voxel(x + 0, y + 0, z - 1),
                //6
                Voxel(x - 1, y + 0, z + 0)
        };
        return neighbours;
    }

};



#endif /* Voxel_h */
