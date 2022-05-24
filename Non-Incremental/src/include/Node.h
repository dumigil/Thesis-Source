#ifndef NODE_H
#define NODE_H
#define eps (10e-10)

struct Node: public Voxel{
    Node *parent, *child;
    int h;
    double  g, rhs;
    std::pair<float, float> k;
    using Voxel::Voxel;

    Node()
    {
        parent = nullptr;
        h = 0;
        g = INFINITY;
        rhs = INFINITY;
        k = {};
        x = 0;
        y = 0;
        z = 0;
    }

    ~Node() = default;

    Node(const int &x, const int &y, const int &z)
            : Voxel(x, y, z){
        parent = nullptr;
        h = 0;
        k = {};
        g = INFINITY;
        rhs = INFINITY;


    }

    explicit Node(Voxel &voxel)
            : Voxel(voxel.x, voxel.y, voxel.z)
    {
        parent = nullptr;
        h = 0;
        k = {};
        g = INFINITY;
        rhs = INFINITY;


    }
    bool operator > (const Node &s2) const {
        if (k.first-eps > s2.k.first) return true;
        else if (k.first < s2.k.first-eps) return false;
        return k.second > s2.k.second;
    }

    bool operator <= (const Node &s2) const {
        if (k.first < s2.k.first) return true;
        else if (k.first > s2.k.first) return false;
        return k.second < s2.k.second + eps;
    }


    bool operator < (const Node &s2) const {
        if (k.first + eps < s2.k.first) return true;
        else if (k.first - eps > s2.k.first) return false;
        return k.second < s2.k.second;
    }

    void addParent(Node *node) {
        this->parent = node;
    }

    Node* getParent() const{
        return this->parent;
    }

    void resetParent(){
        this->parent = nullptr;
    }

    std::vector<Node*> getChildren(){
        std::vector<Node*> kids;
        for(auto n: this->getNeighbours26()){
            auto tmp = new Node();
            tmp->x = n.x;tmp->y = n.y;tmp->z = n.z;
            kids.emplace_back(tmp);
        }
        return kids;
    }


};
struct DynamicGrid {
    std::vector<Node*> voxels;
    int max_x, max_y, max_z;
    float mVoxelsize;

    DynamicGrid( int x,  int y,  int z) {
        max_x = x;
        max_y = y;
        max_z = z;

        int total_voxels = x*y*z;
        voxels.reserve(total_voxels);
        for (int i = 0; i < total_voxels; ++i) {
            auto node = new Node();
            voxels.push_back(node);
        }
    }

    Node *operator()(const int &x, const int &y, const int &z) {
        assert(x >= 0 && x < max_x);
        assert(y >= 0 && y < max_y);
        assert(z >= 0 && z < max_z);
        return voxels[x + y*max_x + z*max_x*max_y];
    }

    Node *operator()(const int &x, const int &y, const int &z) const {
        assert(x >= 0 && x < max_x);
        assert(y >= 0 && y < max_y);
        assert(z >= 0 && z < max_z);
        return voxels[x + y*max_x + z*max_x*max_y];
    }



};







#endif /* node_h */