#include "simulator.h"

Simulator::Simulator(ADG input_adg) {
  adg = input_adg;
  vector<int> init_states(get_agentCnt(adg), 0);//初始化每个agent的状态为0，也就是每个机器都没有沿着路径移动，都在路径起点
  states = init_states;
}

Simulator::Simulator(ADG input_adg, vector<int> visited_states) {
  adg = input_adg;
  states = visited_states;
}

// 遍历每个agent，查看他们下一个状态的依赖是否都已经完成，如果完成则可以移动，否则不能移动
// 输出movable: 表示agent能否移动的列表，如果movable[i] == 1，则agent i 能移动，否则不能移动
// 返回timeSpent: 表示还有多少个机器人正在移动
int Simulator::checkMovable(vector<int>& movable) {
  int timeSpent = 0;
  int agentCnt = get_agentCnt(adg);
  for (int agent = 0; agent < agentCnt; agent++) {
    int state = states[agent];
    if (state >= get_stateCnt(adg, agent) - 1) {
      // 如果agent已经到达终点
      continue;
    }
    timeSpent += 1;
    int next_state = state + 1;

    // 获取agent进入到下一个状态next_state的依赖（也就是adg中不可以交换的节点）
    vector<pair<int, int>> dependencies = get_nonSwitchable_inNeibPair(adg, agent, next_state);
    movable[agent] = 1;
    for (pair<int, int> dependency: dependencies) {
      int dep_agent = get<0>(dependency);
      int dep_state = get<1>(dependency);
      
      if (dep_agent != agent) {
        if (dep_state > states[dep_agent]) {
          movable[agent] = 0;
          break;
        }
      }
    }
  }
  return timeSpent;
}

bool Simulator::incident_to_switchable(int *v_from, int *v_to) {
  int agentCnt = get_agentCnt(adg);
  Graph &graph = get<0>(adg);
  for (int agent = 0; agent < agentCnt; agent++) {
    int state = states[agent];
    if (state >= get_stateCnt(adg, agent) - 1) continue;

    state += 1;
    set<int>& inNeib = get_switchable_inNeib(graph, compute_vertex(get<2>(adg), agent, state));
    for (auto it = inNeib.begin(); it != inNeib.end(); it++) {
      int from = *it;
      *v_from = from;
      *v_to = compute_vertex(get<2>(adg), agent, state);
      return true;
    }

    if (state >= get_stateCnt(adg, agent) - 1) continue;
    state += 1;
    set<int>& outNeib = get_switchable_outNeib(graph, compute_vertex(get<2>(adg), agent, state));
    for (auto it = outNeib.begin(); it != outNeib.end(); it++) {
      int to = *it;
      *v_to = to;
      *v_from = compute_vertex(get<2>(adg), agent, state);
      return true;
    }
  }
  return false;
}

// 遍历每个agent，查看他们下一个状态的依赖是否都已经完成，如果完成则可以移动，否则不能移动
// 输出movable: 表示agent能否移动的列表，如果movable[i] == 1，则agent i 能移动，否则不能移动
// 输出haventStop: 表示agent是否已经停止，如果还没有完成路径，则设置为1，否则设置为0
// 返回timeSpent: 表示还有多少个机器人正在移动
int Simulator::checkMovable(vector<int>& movable, vector<int>& haventStop) {
  int timeSpent = 0;
  int agentCnt = get_agentCnt(adg);
  // 遍历每个agent，查看他们下一个状态的依赖是否都已经完成，如果完成则可以移动，否则不能移动
  for (int agent = 0; agent < agentCnt; agent++) {
    int state = states[agent];
    if (state >= get_stateCnt(adg, agent) - 1) {
      // 如果agent已经到达终点
      continue;
    }
    timeSpent += 1;
    int next_state = state + 1;

    // 获取agent进入到下一个状态next_state的依赖（也就是adg中不可以交换的节点）
    vector<pair<int, int>> dependencies = get_nonSwitchable_inNeibPair(adg, agent, next_state);
    movable[agent] = 1;
    haventStop[agent] = 1;
    for (pair<int, int> dependency: dependencies) {
      int dep_agent = get<0>(dependency);
      int dep_state = get<1>(dependency);
      
      if (dep_agent != agent) {
        if (dep_state > states[dep_agent]) {
          // 如果其他agent的当前状态states[dep_agent]小于节点所依赖的状态，则不能移动
          movable[agent] = 0;
          break;
        }
      }
    }
  }
  return timeSpent;
}

// 机器人单步仿真
// 输入switchCheck：没用到
// 返回timeSpent: 表示还有多少个机器人正在移动
int Simulator::step(bool switchCheck) {
  int agentCnt = get_agentCnt(adg);
  vector<int> movable(agentCnt, 0);
  int timeSpent = checkMovable(movable);
  int moveCnt = 0;

  for (int agent = 0; agent < agentCnt; agent++) {
    if (movable[agent] == 1) {
      states[agent] += 1;
      moveCnt += 1;
    }
  }
  if (moveCnt == 0 && timeSpent != 0) {
    std::cout << "err\n";
    exit(0);
  }
  return timeSpent;
}

void Simulator::print_location(ofstream &outFile, Location location) {
  int i = get<0>(location);
  int j = get<1>(location);
  outFile << "(" << i << "," << j << ")->";
}

int Simulator::print_soln(const char* outFileName) {
  ofstream outFile;
  outFile.open(outFileName);
  int totalSpend = 0;
  int stepSpend = 0;

  if (outFile.is_open()) {
    vector<vector<Location>> expanded_paths;
    int agentCnt = get_agentCnt(adg);
    Paths &paths = get<1>(adg);

    for (int agent = 0; agent < agentCnt; agent ++) {
      vector<Location> expanded_path;
      expanded_path.push_back(get<0>((paths[agent])[0]));
      expanded_paths.push_back(expanded_path);
    }

    stepSpend = step(false);
    while (stepSpend != 0) {
      outFile << "step=" << stepSpend << "\n";
      for (int agent = 0; agent < agentCnt; agent ++) {
        Location new_location = get<0>((paths[agent])[(states[agent])]);
        if (!((same_locations(new_location, (expanded_paths[agent]).back())) && 
            ((size_t)(states[agent]) == (paths[agent]).size() - 1))) {
          (expanded_paths[agent]).push_back(new_location);
        }
      }
      totalSpend += stepSpend;
      stepSpend = step(false);
    }

    for (int agent = 0; agent < agentCnt; agent ++) {
      outFile << "Agent " << agent << ": ";
      vector<Location> &expanded_path = expanded_paths[agent];
      for (Location location: expanded_path) {
        print_location(outFile, location);
      }
      outFile << std::endl;
    }
    outFile.close();
  }

  return totalSpend;
}

// 进行仿真，返回完成的需要的总移动次数
int Simulator::print_soln() {
  int totalSpend = 0;
  int stepSpend = 0;

  stepSpend = step(false);//agent单步移动，返回多少个机器人正在移动
  while (stepSpend != 0) {//循环，直到所有agent都停止移动
    totalSpend += stepSpend;
    stepSpend = step(false);//agent单步移动，返回多少个机器人正在移动
  }
  
  return totalSpend;
}