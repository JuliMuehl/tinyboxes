#include "Collision.hpp"
#include <array>
#include <cassert>
#include <tuple>
#include <unordered_set>
#include <set>


static float dp[4][4];
static float memo[16][4];
static bool visited[16][4];
static SupportPoint y[4];

static float det(int s,int j,int len = -1) noexcept{
    if(visited[s][j]){
        return memo[s][j];
    }
    if(len == -1){
        len = 0;
        for(int i = 1;i <= s;i <<= 1) if(s & i) len++;
    }
    if(len == 1){
        return 1;
    }
    int s_ = s & ~(1 << j);
    int k = -1;
    float sum = 0;
    for(int i = 0;i < 4;++i){
        if(s_ & 1<<i){
            if(k < 0) k = i;
            sum += det(s_,i,len-1) * (dp[k][i] - dp[j][i]);
        }
    }
    visited[s][j] = true;
    memo[s][j] = sum;
    return sum;
}

static bool valid(int simplex,int s) noexcept{
    for(int i = 0;i < 4;i++){
        int bit = 1 << i;
        if(simplex & bit){
            if(s & bit){
                if(det(s,i) <= 0) return false;
            }else if(det(s|bit,i) > 0) {
                return false;
            }
        }
    }
    return true;
}

static void closest_vector(int s,Vector3f& v) noexcept{
    float sum = 0;
    v = Vector3f();
    for(int i =0;i<s;i++){
        if(s & 1 << i) {
            v  = v + det(s,i) * y[i].x;
            sum += det(s,i);
        }
    }
    v = (1/sum) * v;
}

static int closest_simplex(int simplex,Vector3f& v) noexcept{
    for(int i =0;i < 4;i++){
        for(int j = 0;j < 4;j++){
            dp[i][j] = Dot(y[i].x,y[j].x);
        }
    }
    for(int i =0;i < 16;i++){
        for(int j = 0;j < 4;j++){
            visited[i][j] = false;
        }
    }
    for(int s = 1;s<=simplex;s++){
        if((simplex & s) == s){
            if(valid(simplex,s)){
                closest_vector(s,v);
                return s;
            }
        }
    }
    return -1;
}

static void expand_simplex(std::vector<SupportPoint>& simplex, const ConvexCollider& c1, const ConvexCollider& c2) {
    if (simplex.size() == 1) {
        auto w = SupportPoint(c1.Support(Vector3f(1, 0, 0)), c2.Support(Vector3f(-1, 0, 0)));
        simplex.push_back(w);
    }
    if (simplex.size() == 2) {
        auto v = simplex[1].x - simplex[0].x;
        v = Vector3f(-v.y, v.x, 0);
        v.Normalize();
        auto s = SupportPoint(c1.Support((-1) * v), c2.Support(v));
        simplex.push_back(s);
    }
    if (simplex.size() == 3) {
        auto a = simplex[1].x - simplex[0].x;
        auto b = simplex[2].x - simplex[1].x;
        auto v = Cross(a,b);
        v.Normalize();
        auto s = SupportPoint(c1.Support((-1) * v), c2.Support(v));
        simplex.push_back(s);
    }
}

bool gjk(std::vector<SupportPoint>& simplex,const ConvexCollider& c1,const ConvexCollider& c2,Vector3f v) noexcept{
    constexpr float TOLERANCE = 1e-6;
    constexpr unsigned int MAX_ITERATIONS = 1000;
    int simplex_bits = 0;
    unsigned int iteration = 0;
    do{
        int next = 0;
        while(simplex_bits & 1 << next) next++;
        Vector3f s1 = c1.Support((-1)*v);
        Vector3f s2 = c2.Support(v);
        y[next] = SupportPoint(s1,s2);
        if(Dot(y[next].x,v) >= 0) return false;
        simplex_bits |= 1 << next;
        simplex_bits = closest_simplex(simplex_bits,v);
        if(simplex_bits < 0) return false;
        if(iteration++ > MAX_ITERATIONS) return false;
    }while(TOLERANCE < v.Norm());
    simplex.clear();
    simplex.reserve(4);
    for (int i = 0; i < 4;i++) {
        if (simplex_bits & 1 << i) {
            simplex.push_back(y[i]);
        }
    }
    expand_simplex(simplex,c1,c2);
    return true;
}

using Edge = std::pair<size_t, size_t>;
struct Face : public std::tuple<size_t, size_t, size_t> {
    Vector3f normal;
    Face() {}
    Face(size_t i, size_t j, size_t k, Vector3f normal) : std::tuple<size_t, size_t, size_t>(i, j, k), normal(normal) {}
    inline std::array<Edge, 3> edges() const {
        size_t i = std::get<0>(*this), j = std::get<1>(*this), k = std::get<2>(*this);
        return { std::make_pair(i,j) ,std::make_pair(j,k),std::make_pair(k,i) };
    }
};

template<> struct std::hash<Face>{
    size_t operator() (const Face& face) const noexcept{
        size_t h0 = std::hash<size_t>{}(std::get<0>(face));
        size_t h1 = std::hash<size_t>{}(std::get<1>(face));
        size_t h2 = std::hash<size_t>{}(std::get<2>(face));
        return h0 ^ (h1 << 1) ^ (h2 << 2);
    }
};

template<> struct std::hash<Edge>{
    size_t operator() (const Edge& edge) const noexcept{
        size_t h0 = std::hash<size_t>{}(edge.first);
        size_t h1 = std::hash<size_t>{}(edge.second);
        return h0 ^ (h1 << 1);
    }
};

template<typename T>
struct ArraySet{
public:
    using iterator_type = typename std::vector<T>::iterator;
    ArraySet(std::vector<T> data):arr(data){
    }
    ArraySet(){}
    void insert(T elem){
        arr.push_back(elem);
    }
    iterator_type erase(iterator_type it){
        std::swap(*it,arr.back());
        arr.pop_back();
        return it == std::prev(arr.end()) ? arr.end():it;
    }
    iterator_type begin(){
        return arr.begin();
    }
    iterator_type end(){
        return arr.end();
    }
private:
    std::vector<T> arr;
};

struct ExpandingPolytope {
private:
    std::vector<SupportPoint> vertices;
    //std::unordered_set<Face> faces;
    using Set = ArraySet<Face>;
    Set faces;
    inline Vector3f Normal(size_t i, size_t j, size_t k) noexcept{
        Vector3f n = Cross((vertices[j].x - vertices[i].x),(vertices[k].x - vertices[i].x));
        n = (1 / n.Norm()) * n;
        float distance = Dot(n,vertices[i].x);
        if (distance < 0) n = (-1) * n;
        else if (distance == 0) {
            for (auto& f : faces) {
                if (Dot(f.normal,n) > 0) {
                    n = (-1) * n;
                }
            }
        }
        return n;
    }

    inline Face MakeFace(size_t i, size_t j, size_t k) noexcept{
        return { i,j,k,Normal(i,j,k) };
    }
public:
    void InsertVertex(SupportPoint point) noexcept{
        vertices.push_back(point);
        std::unordered_set<Edge> horizon;
        for (auto it = faces.begin(); it != faces.end();) {
            const Face& face = *it;
            if (Dot(face.normal,point.x - vertices[std::get<0>(face)].x) >= 0) {
                for (const Edge& edge : face.edges()) {
                    Edge reversed = std::make_pair(edge.second, edge.first);
                    auto search_it = horizon.find(reversed);
                    if (search_it != horizon.end()) {
                        horizon.erase(search_it);
                    }
                    else
                        horizon.insert(edge);
                }
                it = faces.erase(it);
            }
            else {
                ++it;
            }
        }
        for (const Edge& edge : horizon) {
            faces.insert(MakeFace(edge.first, edge.second, vertices.size() - 1));
        }
    }

    Face ClosestFace(float& min_distance) noexcept{
        min_distance = std::numeric_limits<float>::max();
        Face face_result;
        for (const Face& face : faces) {
            float d = Dot(face.normal,vertices[std::get<0>(face)].x);
            if (d < min_distance) {
                min_distance = d;
                face_result = face;
            }
        }
        return face_result;
    }

    SupportPoint ClosestPoint(const Face& face) noexcept{
        SupportPoint &v0 = vertices[std::get<0>(face)], &v1 = vertices[std::get<1>(face)], &v2 = vertices[std::get<2>(face)];
        Vector3f &x0 = v0.x, &x1 = v1.x, &x2 = v2.x;
        Vector3f p = Dot(face.normal,x0) * face.normal;
        Vector3f a = x1 - x0;
        Vector3f b = x2 - x0;
        Vector3f c = p  - x0;
        float daa = Dot(a,a);
        float dab = Dot(a,b);
        float dac = Dot(a,c);
        float dbb = Dot(b,b);
        float dbc = Dot(b,c);
        float dcc = Dot(c,c);
        float det = daa * dbb - dab * dab;
        float lambda1 = (dbb * dac - dab * dbc) / det;
        float lambda2 = (daa * dbc - dab * dac) / det;
        float lambda0 = 1 - lambda1 - lambda2;
        return lambda0 * v0 + lambda1 * v1 + lambda2 * v2;
    }

    ExpandingPolytope(const std::vector<SupportPoint>& simplex) : vertices(simplex) {
        assert(simplex.size() == 4);
        faces = Set({ MakeFace(1,2,0) ,MakeFace(3,2,1) ,MakeFace(0,3,1) ,MakeFace(2,3,0) });
    }
};

Contact EPA(const std::vector<SupportPoint>& simplex,const ConvexCollider& c1,const ConvexCollider& c2) noexcept{
    constexpr unsigned int MAX_ITERATIONS = 100;
    constexpr float TOLERANCE = 1e-6;
    auto polytope = ExpandingPolytope(simplex);
    Face face;
    unsigned int i = 0;
    float distance = 0;
    float d = 0;
    while(true) {
        float last_distance = distance;
        face = polytope.ClosestFace(distance);
        auto v = c1.Support(face.normal);
        auto w = c2.Support((-1) * face.normal);
        auto s = SupportPoint(v, w);
        d = Dot(face.normal,s.x);
        if (d - distance <= TOLERANCE) {
            break;
        }
        if (i > MAX_ITERATIONS) {
            break;
        }
        ++i;
        polytope.InsertVertex(s);
    }
    SupportPoint closest = polytope.ClosestPoint(face);
    Contact c;
    c.point = 0.5 * closest.a + 0.5 * closest.b;
    c.depth = closest.x.NormSquared();
    c.normal = closest.x.Normalize();
    if(c.normal.x != 0) c.u1 = Cross(c.normal,Vector3f(0,1,0)).Normalize();
    else c.u1 = Cross(c.normal,Vector3f(1,0,0)).Normalize();
    c.u2 = Cross(c.u1,c.normal).Normalize();
    return c;
}
