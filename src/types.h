#ifndef TYPES
#define TYPES

#include <stdlib.h>
#include <tuple>
#include <vector>
#include <set>
#include <utility>

using namespace std;

// <arr[time]=vertex, arr[vertex]=time>
typedef pair<vector<int>*, vector<int>*> sortResult;

// <outNeighbors, inNeighbors>
typedef pair<set<int>*, set<int>*> subGraph;//<out边集合，in边集合>，如subGraph[5]表示从节点5出发的边

// <type1 Graph, non-switchable Type2 Graph, switchable Type2 Graph, num nodes>
//ADG的类，<type1的边的集合，type2的非交换边的集合，type2的交换边的集合，节点数>
typedef tuple<subGraph, subGraph, subGraph, int> Graph;

typedef pair<int, int> Location;
typedef vector<pair<Location, int>> Path;//轨迹，每个元素为<location, time>，每个元素代表位置和时刻(timestep)
typedef vector<Path> Paths;//每个agent的轨迹集合
typedef tuple<Graph, Paths, vector<int>> ADG;//<adg_graph, paths, ??>
typedef tuple<ADG, int, vector<int>*> Node;

typedef tuple<ADG, int, int, vector<int>> slow_Node;


#endif

