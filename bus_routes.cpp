#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <chrono>
#include <random>
#include <iomanip>
#include <string>
#include <algorithm>

using namespace std;

static const int COST_RIDE     = 1;
static const int COST_TRANSFER = 3;
static const int INF = numeric_limits<int>::max() / 4;

struct PQItem {
    int node;
    int dist;
};

//  РЕАЛИЗАЦИЯ A — приоритетная очередь на массиве
class PriorityQueueArray {
    vector<PQItem> data;
public:
    void push(PQItem x) { data.push_back(x); }

    bool empty() const  { return data.empty(); }

    PQItem pop_min() {
        size_t best = 0;
        for (size_t i = 1; i < data.size(); ++i)
            if (data[i].dist < data[best].dist) best = i;

        PQItem result = data[best];
        data[best] = data.back();
        data.pop_back();
        return result;
    }
};

//  РЕАЛИЗАЦИЯ B — приоритетная очередь на односвязном списке
class PriorityQueueLinkedList {
    struct Node {
        PQItem value;
        Node*  next;
    };
    Node* head = nullptr;
public:
    ~PriorityQueueLinkedList() {
        while (head) { Node* t = head; head = head->next; delete t; }
    }

    bool empty() const { return head == nullptr; }

    void push(PQItem x) {
        Node* fresh = new Node{ x, nullptr };
        if (!head || x.dist < head->value.dist) {
            fresh->next = head;
            head = fresh;
            return;
        }
        Node* cur = head;
        while (cur->next && cur->next->value.dist <= x.dist)
            cur = cur->next;
        fresh->next = cur->next;
        cur->next   = fresh;
    }

    PQItem pop_min() {
        Node*  top = head;
        PQItem v   = top->value;
        head = head->next;
        delete top;
        return v;
    }
};

//  РЕАЛИЗАЦИЯ C — приоритетная очередь средствами STL
class PriorityQueueSTL {
    struct Cmp { bool operator()(const PQItem& a, const PQItem& b) const {
        return a.dist > b.dist;
    } };
    priority_queue<PQItem, vector<PQItem>, Cmp> pq;
public:
    bool   empty() const { return pq.empty(); }
    void   push(PQItem x) { pq.push(x); }
    PQItem pop_min()      { PQItem v = pq.top(); pq.pop(); return v; }
};

//  Граф состояний
struct Graph {
    int V = 0;
    vector<vector<pair<int,int>>> adj;

    int addVertex() { adj.emplace_back(); return V++; }
    void addEdge(int u, int v, int w) {
        adj[u].push_back({v, w});
        adj[v].push_back({u, w});
    }
};

struct BuiltGraph {
    Graph g;
    vector<vector<int>> vertexOfStop;
};

BuiltGraph buildGraph(int n, const vector<vector<int>>& routes) {
    BuiltGraph bg;
    bg.vertexOfStop.assign(n + 1, {});

    int R = (int)routes.size();

    vector<vector<pair<int,int>>> routeVertices(R);

    for (int r = 0; r < R; ++r) {
        const auto& route = routes[r];
        routeVertices[r].reserve(route.size());
        for (int stop : route) {
            int vid = bg.g.addVertex();
            routeVertices[r].push_back({stop, vid});
            bg.vertexOfStop[stop].push_back(vid);
        }
        for (size_t k = 1; k < routeVertices[r].size(); ++k) {
            int u = routeVertices[r][k - 1].second;
            int v = routeVertices[r][k].second;
            bg.g.addEdge(u, v, COST_RIDE);
        }
    }

    for (int s = 1; s <= n; ++s) {
        const auto& verts = bg.vertexOfStop[s];
        for (size_t i = 0; i < verts.size(); ++i)
            for (size_t j = i + 1; j < verts.size(); ++j)
                bg.g.addEdge(verts[i], verts[j], COST_TRANSFER);
    }

    return bg;
}

//  Универсальная реализация алгоритма Дейкстры, параметризованная
template <typename PQ>
int dijkstra(const Graph& g, int source, int target,
             vector<int>* parentOut = nullptr)
{
    vector<int> dist(g.V, INF);
    vector<int> parent(g.V, -1);
    dist[source] = 0;

    PQ pq;
    pq.push({source, 0});

    while (!pq.empty()) {
        PQItem cur = pq.pop_min();
        if (cur.dist > dist[cur.node]) continue;
        if (cur.node == target) break;
        for (auto [nxt, w] : g.adj[cur.node]) {
            int nd = cur.dist + w;
            if (nd < dist[nxt]) {
                dist[nxt]  = nd;
                parent[nxt] = cur.node;
                pq.push({nxt, nd});
            }
        }
    }

    if (parentOut) *parentOut = parent;
    return dist[target];
}

struct SolveResult {
    int time;
    vector<int> stopsPath;
    vector<int> routeOfStep;
};

template <typename PQ>
SolveResult solveBusProblem(int n,
                            const vector<vector<int>>& routes,
                            int startStop, int endStop)
{
    BuiltGraph bg = buildGraph(n, routes);

    vector<int> vertexStop(bg.g.V, 0);
    vector<int> vertexRoute(bg.g.V, -1);

    int idx = 0;
    for (int r = 0; r < (int)routes.size(); ++r)
        for (int stop : routes[r]) {
            vertexStop[idx]  = stop;
            vertexRoute[idx] = r;
            ++idx;
        }

    int S = bg.g.addVertex();
    int T = bg.g.addVertex();
    vertexStop.push_back(startStop); vertexRoute.push_back(-1); // S
    vertexStop.push_back(endStop);   vertexRoute.push_back(-1); // T

    for (int v : bg.vertexOfStop[startStop]) bg.g.addEdge(S, v, 0);
    for (int v : bg.vertexOfStop[endStop])   bg.g.addEdge(T, v, 0);

    vector<int> parent;
    int answer = dijkstra<PQ>(bg.g, S, T, &parent);

    SolveResult res;
    res.time = answer;

    if (answer >= INF) return res;

    vector<int> rev;
    for (int v = T; v != -1; v = parent[v]) rev.push_back(v);
    reverse(rev.begin(), rev.end());

    for (size_t i = 0; i < rev.size(); ++i) {
        int v = rev[i];
        if (v == S || v == T) continue;
        int stop = vertexStop[v];
        if (res.stopsPath.empty() || res.stopsPath.back() != stop) {
            res.stopsPath.push_back(stop);
            res.routeOfStep.push_back(vertexRoute[v]);
        } else {
            res.routeOfStep.back() = vertexRoute[v];
        }
    }
    return res;
}

void demoExample() {
    cout << "\n=== Демонстрационный пример ===\n";
    int n = 7;
    vector<vector<int>> routes = {
        {1, 2, 3, 4},
        {3, 5, 6},
        {6, 7}
    };

    int I = 1, J = 7;
    cout << "Остановок: " << n << ", маршрутов: " << routes.size() << "\n";
    for (size_t r = 0; r < routes.size(); ++r) {
        cout << "  Маршрут " << (r + 1) << ":";
        for (int s : routes[r]) cout << " " << s;
        cout << "\n";
    }
    cout << "Найти путь из остановки " << I << " в " << J << "\n";

    auto res = solveBusProblem<PriorityQueueSTL>(n, routes, I, J);
    if (res.time >= INF) {
        cout << "Пути не существует.\n";
        return;
    }
    cout << "Минимальное время: " << res.time
         << " ед. (проезд = " << COST_RIDE
         << ", пересадка = " << COST_TRANSFER << ")\n";
    cout << "Маршрут движения:\n";
    int prevRoute = -2;
    for (size_t i = 0; i < res.stopsPath.size(); ++i) {
        int stop = res.stopsPath[i];
        int r    = res.routeOfStep[i];
        if (r != prevRoute && r >= 0) {
            cout << "  [маршрут " << (r + 1) << "] ";
            prevRoute = r;
        } else {
            cout << "                ";
        }
        cout << "остановка " << stop << "\n";
    }
}

//  Бенчмарк: одна и та же случайная задача решается тремя реализациями.
struct BenchInput {
    int n;
    vector<vector<int>> routes;
    int I, J;
};

BenchInput generateRandomCase(int n, int R, int routeLen, uint32_t seed) {
    mt19937 rng(seed);
    uniform_int_distribution<int> stop(1, n);

    BenchInput in;
    in.n = n;
    in.routes.resize(R);
    for (int r = 0; r < R; ++r) {
        in.routes[r].reserve(routeLen);
        for (int k = 0; k < routeLen; ++k)
            in.routes[r].push_back(stop(rng));
    }
    in.I = stop(rng);
    in.J = stop(rng);
    return in;
}

template <typename PQ>
pair<int, double> runWith(const BenchInput& in, int repeats = 5) {
    int answer = 0;
    double bestMs = 1e18;
    for (int it = 0; it < repeats; ++it) {
        auto t0 = chrono::high_resolution_clock::now();
        auto res = solveBusProblem<PQ>(in.n, in.routes, in.I, in.J);
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double, milli>(t1 - t0).count();
        answer = res.time;
        if (ms < bestMs) bestMs = ms;
    }
    return {answer, bestMs};
}

void benchmark() {
    cout << "\n=== Бенчмарк трёх реализаций приоритетной очереди ===\n";
    cout << "Метрика: время решения одной задачи Дейкстры, миллисекунды.\n";

    struct Case { int n, R, len; };
    vector<Case> cases = {
        { 20,    5,   8 },
        { 100,  20,  15 },
        { 300,  50,  20 },
        { 800, 120,  25 }
    };

    cout << "\n"
         << left  << setw(8)  << "n"
         << setw(8)  << "R"
         << setw(8)  << "len"
         << right << setw(14) << "Array, ms"
         << setw(14) << "List, ms"
         << setw(14) << "STL, ms"
         << setw(14) << "answer\n";
    cout << string(80, '-') << "\n";

    for (auto c : cases) {
        BenchInput in = generateRandomCase(c.n, c.R, c.len, 42);

        auto [t1, ms1] = runWith<PriorityQueueArray>(in);
        auto [t2, ms2] = runWith<PriorityQueueLinkedList>(in);
        auto [t3, ms3] = runWith<PriorityQueueSTL>(in);

        cout << left  << setw(8) << c.n
             << setw(8) << c.R
             << setw(8) << c.len
             << right << fixed << setprecision(3)
             << setw(14) << ms1
             << setw(14) << ms2
             << setw(14) << ms3
             << setw(14) << (t1 >= INF ? -1 : t1)
             << "\n";

        if (t1 != t2 || t2 != t3) {
            cout << "  ВНИМАНИЕ: ответы реализаций различаются ("
                 << t1 << ", " << t2 << ", " << t3 << ")\n";
        }
    }

    cout << "\nВыводы:\n"
         << "  - реализация на массиве:        вставка O(1), извлечение O(n);\n"
         << "  - реализация на связном списке: вставка O(n), извлечение O(1);\n"
         << "  - реализация на STL (binary heap): обе операции O(log n);\n"
         << "  На малых n все три работают сопоставимо; с ростом n\n"
         << "  STL-реализация выигрывает на порядок и более.\n";
}

int main() {
    cout << "=====================================================\n";
    cout << " Лабораторная работа\n";
    cout << " Автор:  Лукин Роман Вячеславович\n";
    cout << " Группа: 090304-РПИа-о25\n";
    cout << " Тема:   Поиск кратчайшего пути в сети автобусных\n";
    cout << "         маршрутов. Сравнение трёх реализаций\n";
    cout << "         приоритетной очереди.\n";
    cout << "=====================================================\n";

    demoExample();
    benchmark();

    return 0;
}
