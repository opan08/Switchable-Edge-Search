#include "generate_ADG.h"

bool same_locations(Location location1, Location location2) {
  int i1 = location1.first;
  int j1 = location1.second;
  int i2 = location2.first;
  int j2 = location2.second;
  
  return (i1 == i2 && j1 == j2);
}

// Return path and stateCnt of an agent
// line表示文件中记录的一行，每一行表示一个agent的路径
// 返回路径（如果等待前后2个栅格相同，则只记录一个）以及变化了多少次栅格
tuple<Path, int> parse_path(string line) {
  int i, j, stateCnt = 0;
  int time = 0;//记录这段路径有多少个timestep
  size_t comma_pos, leftPar_pos, rightPar_pos;
  Path path;
  Location prev_location = make_pair(-1, -1);

  while ((leftPar_pos = line.find("(")) != string::npos) {
    // Process an index pair
    comma_pos = line.find(",");
    i = stoi(line.substr(leftPar_pos + 1, comma_pos));
    rightPar_pos = line.find(")");
    j = stoi(line.substr(comma_pos + 1, rightPar_pos));
    line.erase(0, rightPar_pos + 1);

    // Create a location tuple and add it to the path
    Location location = make_pair(i, j);
    if (!same_locations(location, prev_location)) {
      stateCnt ++;
      path.push_back(make_pair(location, time));
      prev_location = location;
    }
    time++;
  }
  return make_tuple(path, stateCnt);
}

// Return all paths, accumulated counts of states, and States
// 返回所有的路径以及累计的状态数
tuple<Paths, vector<int>> parse_soln(char* fileName) {
  Paths paths;
  vector<int> accum_stateCnts;
  int sumStates = 0;

  string fileName_string = fileName;
  ifstream file(fileName_string);
  if (file.is_open()) {
    string line;
    while (getline(file, line)) {
      // Sanity check that the line is a path 路径文件中，如果是路径的话，第一个字符是A，Agent
      if (line[0] == 'A') {
        tuple<Path, int> parse_result = parse_path(line);
        Path path = get<0>(parse_result);
        int stateCnt = get<1>(parse_result);

        // Done with the agent
        paths.push_back(path);
        sumStates += stateCnt;
        accum_stateCnts.push_back(sumStates);
      }
    }
    file.close();
  } else {
    std::cout << "exit\n";
    exit(0);
  }
  return make_tuple(paths, accum_stateCnts);
}

// 遍历每个agent规划的路径，添加ADG中type1的边
// graph<0>保存了ADG的依赖关系
void add_type1_edges(Graph &graph, Paths &paths, vector<int> &accum_stateCnts) {
  int agentCnt = paths.size();
  for (int agent = 0; agent < agentCnt; agent++) {
    Path path = paths[agent];
    int stateCnt = path.size();
    int prev_vertex = -1;
    
    for (int state = 0; state < stateCnt; state++) {
      int curr_vertex = compute_vertex(accum_stateCnts, agent, state);
      if (prev_vertex >= 0) {
        set_type1_edge(graph, prev_vertex, curr_vertex);
      }
      prev_vertex = curr_vertex;
    }
  }
}

// 通过路径信息paths，添加ADG中type2的边（包括可交换的边和不可交换的边）
void add_type2_edges(Graph &graph, Paths &paths, vector<int> &accum_stateCnts) {
  int agentCnt = paths.size();
  // Looping through agents
  for (int agent1 = 0; agent1 < agentCnt; agent1++) {
    Path path1 = paths[agent1];
    int stateCnt1 = path1.size();
    for (int agent2 = agent1 + 1; agent2 < agentCnt; agent2 ++) {
      Path path2 = paths[agent2];
      int stateCnt2 = path2.size();

      // Looping through states
      for (int state1 = 0; state1 < stateCnt1; state1++) {
        pair<Location, int> pair1 = path1[state1];
        Location location1 = get<0>(pair1);
        for (int state2 = 0; state2 < stateCnt2; state2++) {
          pair<Location, int> pair2 = path2[state2];
          Location location2 = get<0>(pair2);

          int time1 = get<1>(pair1);
          int time2 = get<1>(pair2);

          if (same_locations(location1, location2) && (time1 >= 0) && (time2 >= 0)) {
            // Add a type2 edge
            int vertex1 = compute_vertex(accum_stateCnts, agent1, state1);
            int vertex2 = compute_vertex(accum_stateCnts, agent2, state2);

            // Set edges -- fix the starting out-edges and ending in-edges
            if (time1 < time2) {
              if ((state1 == 0) || (state2 == stateCnt2 - 1)) {
                set_type2_nonSwitchable_edge(graph, vertex1+1, vertex2);
              } else {
                set_type2_switchable_edge(graph, vertex1+1, vertex2);
              }
            }
            else {
              if ((state2 == 0) || (state1 == stateCnt1 - 1)) {
                set_type2_nonSwitchable_edge(graph, vertex2+1, vertex1);
              } else {
                set_type2_switchable_edge(graph, vertex2+1, vertex1);
              }
            }

          }
        }
      }
    }
  }
}

/**
 * @brief 通过路径信息paths添加类型2的边，并返回可交换边的数量
 * 
 * @param graph 输出的adg
 * @param paths 路径
 * @param accum_stateCnts 新的总共的状态数
 * @return int 添加的可交换边的数量
 */
int add_type2_edges_cnt(Graph &graph, Paths &paths, vector<int> &accum_stateCnts) {
  int cnt = 0;
  int agentCnt = paths.size();
  // Looping through agents
  for (int agent1 = 0; agent1 < agentCnt; agent1++) {
    Path path1 = paths[agent1];
    int stateCnt1 = path1.size();
    for (int agent2 = agent1 + 1; agent2 < agentCnt; agent2 ++) {
      Path path2 = paths[agent2];
      int stateCnt2 = path2.size();

      // Looping through states
      for (int state1 = 0; state1 < stateCnt1; state1++) {
        pair<Location, int> pair1 = path1[state1];
        Location location1 = get<0>(pair1);
        for (int state2 = 0; state2 < stateCnt2; state2++) {
          pair<Location, int> pair2 = path2[state2];
          Location location2 = get<0>(pair2);

          int time1 = get<1>(pair1);
          int time2 = get<1>(pair2);

          if (same_locations(location1, location2) && (time1 >= 0) && (time2 >= 0)) {
            // Add a type2 edge
            int vertex1 = compute_vertex(accum_stateCnts, agent1, state1);
            int vertex2 = compute_vertex(accum_stateCnts, agent2, state2);

            // Set edges -- fix the starting out-edges and ending in-edges
            if (time1 < time2) {
              if ((state1 == 0) || (state2 == stateCnt2 - 1)) {
                set_type2_nonSwitchable_edge(graph, vertex1+1, vertex2);
              } else {
                set_type2_switchable_edge(graph, vertex1+1, vertex2);
                cnt += 1;
              }
            }
            else {
              if ((state2 == 0) || (state1 == stateCnt1 - 1)) {
                set_type2_nonSwitchable_edge(graph, vertex2+1, vertex1);
              } else {
                set_type2_switchable_edge(graph, vertex2+1, vertex1);
                cnt += 1;
              }
            }

          }
        }
      }
    }
  }
  return cnt;
}

/**
 * @brief 读取路径信息，构造ADG
 * 
 * @param fileName 路径的文件名
 * @return ADG 
 */
ADG construct_ADG(char* fileName) {
  Paths paths;
  vector<int> accum_stateCnts;
  tie(paths, accum_stateCnts) = parse_soln(fileName);//解析文件中的路径信息（可以理解为MAPF中得到的路径）
  int sumStates = accum_stateCnts.back();//所有agent加起来总的状态数

  Graph graph = new_graph(sumStates);
  add_type1_edges(graph, paths, accum_stateCnts);// 添加type1的边
  add_type2_edges(graph, paths, accum_stateCnts);// 添加ADG中type2的边（包括可交换的边和不可交换的边）

  return make_tuple(graph, paths, accum_stateCnts);
}

/**
 * @brief 重新构造延时后的ADG
 * 
 * @param adg 输入的adg
 * @param dlow 延时的最小值，单位：步
 * @param dhigh 延时的最大值，单位：步
 * @param delayed_agents 记录agent是否应该延迟的列表
 * @param states 各个agent的当前状态（也就是当前路径运行到的序号）
 * @param input_sw_cnt 输出延时后，adg中可交换边的数量
 * @param outFile_setup 输出哪个agent延时了，已经延时多久的保存文件
 * @return ADG 延时后的ADG
 */
ADG construct_delayed_ADG(ADG &adg, int dlow, int dhigh, vector<int> &delayed_agents, vector<int> &states, int *input_sw_cnt, ofstream &outFile_setup) {
  int agentCnt = get_agentCnt(adg);
  Paths paths;
  vector<int> accum_stateCnts;
  
  random_device rd;  
  mt19937 gen(rd());
  uniform_int_distribution<> distrib(dlow, dhigh);

  int delay_sum = 0;

  for (int agent = 0; agent < agentCnt; agent ++) {
    if (delayed_agents[agent] == 0) {
      // 如果不需要延迟
      outFile_setup << agent << ": 0\n";
      paths.push_back((get<1>(adg))[agent]);// 直接复制原来的路径
    } else { // this is a delayed agent如果需要延迟
      Path &ori_path = (get<1>(adg))[agent];//原来的路径
      Path new_path;
      int delayed_state = states[agent];//获取当前agent的状态作为延时起点

      // 将已经走过的路径复制到新路径中
      for (int state = 0; state <= delayed_state; state ++) {
        new_path.push_back(ori_path[state]);
      }
      pair<Location, int> repeat = make_pair(get<0>(new_path.back()), -1);
      int delay = distrib(gen);//从延时最小值到最大值随机生成一个延时值
      outFile_setup << agent << ": "<< delay << "\n";
      delay_sum += delay;
      // 将延时的状态插入到新路径中
      for (int state = 0; state < delay; state ++) {
        new_path.push_back(repeat);
      }
      int ori_size = ori_path.size();
      // 将原来路径的剩下的路径复制到新路径中
      for (int state = delayed_state + 1; state < ori_size; state ++) {
        new_path.push_back(ori_path[state]);
      }
      paths.push_back(new_path);
    }

    accum_stateCnts.push_back((get<2>(adg))[agent] + delay_sum);
  }

  Graph graph = new_graph(accum_stateCnts.back());//新的adg

  // 重新添加type1的边和type2的边，并返回可交换边的数量
  add_type1_edges(graph, paths, accum_stateCnts);
  *input_sw_cnt = add_type2_edges_cnt(graph, paths, accum_stateCnts);

  // 遍历graph的所有节点，将已经走过的时刻，删除可交换边，并添加为不可交换边，TODO:不太懂原因？？？
  for (int v = 0; v < get<3>(graph); v ++) {
    int agent, state;
    tie(agent, state) = compute_agent_state(accum_stateCnts, v);
    if (state <= states[agent]) {
      set<int> outNeib = get_switchable_outNeib(graph, v);
      for (auto it: outNeib) {
        rem_type2_switchable_edge(graph, v, it);
        set_type2_nonSwitchable_edge(graph, v, it);
        *input_sw_cnt = *input_sw_cnt - 1;
      }
    }
  }

  return make_tuple(graph, paths, accum_stateCnts);
}