#include "ADG_utilities.h"
#include "../types.h"

// 通过adg的paths获取agent数量
int get_agentCnt(ADG &adg) {
  Paths &paths = get<1>(adg);
  return paths.size();
}

int get_stateCnt(ADG &adg, int agent) {
  if (agent == 0) return get<2>(adg)[0];
  return get<2>(adg)[agent] - get<2>(adg)[agent-1];
}

int get_totalStateCnt(ADG &adg) {
  return get<2>(adg).back();
}

// 根据agent的id和state，获取顶点的编号或id
// accum_stateCnts保存每个agent的累计状态数，如accum_stateCnts[3]表示从第0个到第3个agent的累计状态数
int compute_vertex(vector<int> &accum_stateCnts, int agent, int state) {
  if (agent == 0) { // Accumulated state cnt == 0
    assert(state < accum_stateCnts[0]);
    return state; 
  }
  assert(state < accum_stateCnts[agent] - accum_stateCnts[agent-1]);
  int accum_stateCnt = accum_stateCnts[agent - 1];//上一个agent的累计状态数
  return (state + accum_stateCnt);
}

// 通过adg的顶点v获取agent和state
// 返回的是<agent, state>的pair
pair<int, int> compute_agent_state(vector<int> &accum_stateCnts, int v) {
  int agent = 0;
  int prevStateCnt = 0;
  for (int stateCnt: accum_stateCnts) {
    if (v < stateCnt) return make_pair(agent, v - prevStateCnt);
    prevStateCnt = stateCnt;
    agent ++;
  }
  return make_pair(-1, -1);
}

int compute_start_vertex(vector<int> &accum_stateCnts, int v) {
  int prevStateCnt = 0;
  for (int stateCnt: accum_stateCnts) {
    if (v < stateCnt) return prevStateCnt;
    prevStateCnt = stateCnt;
  }
  return -1;
}

// 判断(agent1, state1)到(agent2, state2)是否是type2的边
bool is_type2_edge(ADG &adg, int agent1, int state1, int agent2, int state2) {
  Graph &graph = get<0>(adg);
  int v1 = compute_vertex(get<2>(adg), agent1, state1);
  int v2 = compute_vertex(get<2>(adg), agent2, state2);
  return (get_type2_switchable_edge(graph, v1, v2) || 
          get_type2_nonSwitchable_edge(graph, v1, v2));
}

// 判断(agent1, state1)到(agent2, state2)是否是type2的switchable边
bool is_type2_switchable_edge(ADG &adg, int agent1, int state1, int agent2, int state2) {
  Graph &graph = get<0>(adg);
  vector<int> &accum_stateCnts = get<2>(adg);
  int v1 = compute_vertex(accum_stateCnts, agent1, state1);
  int v2 = compute_vertex(accum_stateCnts, agent2, state2);
  return (get_type2_switchable_edge(graph, v1, v2));
}

// 
void fix_type2_edge(ADG &adg, int agent1, int state1, int agent2, int state2) {
  Graph &graph = get<0>(adg);
  int v1 = compute_vertex(get<2>(adg), agent1, state1);
  int v2 = compute_vertex(get<2>(adg), agent2, state2);

  rem_type2_switchable_edge(graph, v1, v2);
  set_type2_nonSwitchable_edge(graph, v1, v2);
}

// 
void fix_type2_edge_reversed(ADG &adg, int agent1, int state1, int agent2, int state2) {
  Graph &graph = get<0>(adg);
  int v1 = compute_vertex(get<2>(adg), agent1, state1);
  int v2 = compute_vertex(get<2>(adg), agent2, state2);

  rem_type2_switchable_edge(graph, v1, v2);
  set_type2_nonSwitchable_edge(graph, v2+1, v1-1);
}

vector<pair<int, int>> get_switchable_inNeibPair(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  set<int> &inNeighbors = get_switchable_inNeib(graph, v);

  vector<pair<int, int>> inNeighbors_pair;
  for (auto it = inNeighbors.begin(); it != inNeighbors.end(); it++) {
    inNeighbors_pair.push_back(compute_agent_state(get<2>(adg), *it));
  }
  return inNeighbors_pair;
}

vector<pair<int, int>> get_switchable_outNeibPair(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  set<int>& outNeighbors = get_switchable_outNeib(graph, v);

  vector<pair<int, int>> outNeighbors_pair;
  for (auto it = outNeighbors.begin(); it != outNeighbors.end(); it++) {
    outNeighbors_pair.push_back(compute_agent_state(get<2>(adg), *it));
  }
  return outNeighbors_pair;
}

// 获取不可交换的状态
// 返回的是一个<agent, state>的vector
vector<pair<int, int>> get_nonSwitchable_inNeibPair(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  set<int> inNeighbors = get_nonSwitchable_inNeib(graph, v);// 获取所有进入n的type1的节点和不可交换的type2的节点

  vector<pair<int, int>> inNeighbors_pair;//记录不可以交换节点中的agent和state
  for (auto it = inNeighbors.begin(); it != inNeighbors.end(); it++) {
    inNeighbors_pair.push_back(compute_agent_state(get<2>(adg), *it));
  }
  return inNeighbors_pair;
}

vector<pair<int, int>> get_nonSwitchable_outNeibPair(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  set<int> outNeighbors = get_nonSwitchable_outNeib(graph, v);

  vector<pair<int, int>> outNeighbors_pair;
  for (auto it = outNeighbors.begin(); it != outNeighbors.end(); it++) {
    outNeighbors_pair.push_back(compute_agent_state(get<2>(adg), *it));
  }
  return outNeighbors_pair;
}

vector<pair<int, int>> get_inNeibPair(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  set<int> inNeighbors = get_inNeighbors(graph, v);

  vector<pair<int, int>> inNeighbors_pair;
  for (auto it = inNeighbors.begin(); it != inNeighbors.end(); it++) {
    inNeighbors_pair.push_back(compute_agent_state(get<2>(adg), *it));
  }
  return inNeighbors_pair;
}

vector<pair<int, int>> get_outNeibPair(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  set<int> outNeighbors = get_outNeighbors(graph, v);

  vector<pair<int, int>> outNeighbors_pair;
  for (auto it = outNeighbors.begin(); it != outNeighbors.end(); it++) {
    outNeighbors_pair.push_back(compute_agent_state(get<2>(adg), *it));
  }
  return outNeighbors_pair;
}

// 获取当前的目标点的位置，从agent的路径中获取
Location get_state_target(ADG &adg, int agent, int state) {
  return get<0>(((get<1>(adg))[agent])[state]);
}

ADG copy_ADG(ADG &adg) {
  Graph &graph = get<0>(adg);
  return make_tuple(copy_graph(graph), get<1>(adg), get<2>(adg));
}

bool detectCycle(ADG &adg, int agent, int state) {
  Graph &graph = get<0>(adg);
  int v = compute_vertex(get<2>(adg), agent, state);
  return check_cycle_nonSwitchable(graph, v);
}

void free_underlying_graph(ADG &adg) {
  Graph &graph = get<0>(adg);
  free_graph(graph);
  return;
}

void free_shared_graph(ADG &adg) {
  Graph &graph = get<0>(adg);
  free_nonSwitchable(graph);
  return;
}