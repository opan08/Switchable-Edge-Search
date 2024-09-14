#include <fstream>

#include "Algorithm/Astar.h"

// 打印重新规划时候，agent的当前位置和目标位置
void print_for_replanning(ADG &adg, vector<int> states, ofstream &outFile_path) {
  outFile_path << "version 1\n";
  int agentCnt = get_agentCnt(adg);
  for (int agent = 0; agent < agentCnt; agent ++) {
    outFile_path << "4	random-32-32-10.map	32	32	";
    Location l = get_state_target(adg, agent, states[agent]);//获取当前的位置
    Location e = get_state_target(adg, agent, get_stateCnt(adg, agent)-1);//获取agent路径的终点
    outFile_path << get<1>(l) << "	" << get<0>(l) << "	" << get<1>(e) << "	" << get<0>(e) << "	18.72792206\n";
  }
}

// 仿真的单步函数
// p：决定随机延时的阈值，随机数从[1, 1000]，产生的随机数小于p，则agent需要延时
// delay_mark：是否产生延时的标志位
// delayed_agents：记录哪些agent需要延时
// 返回值：还有多少agent可以移动，如果没有agent可以移动，则返回-1，如果延时发生，则返回0
int Simulator::step_wdelay(int p, bool *delay_mark, vector<int> &delayed_agents) {
  int agentCnt = get_agentCnt(adg);

  random_device rd;  
  mt19937 gen(rd());
  uniform_int_distribution<> distrib(1, 1000);

  vector<int> movable(agentCnt, 0);//表示agent能否移动的列表
  vector<int> haventStop(agentCnt, 0);//表示agent是否还没有停止（是否还没有完成路径）的列表，1则还没有停止，0则已经停止
  int timeSpent = checkMovable(movable, haventStop);//timeSpent表示还有多少agent移动，检测agent能否移动，输出保存到movable，并初始化hanveStop为1
  int moveCnt = 0;//统计这个周期移动的agent数目

  for (int agent = 0; agent < agentCnt; agent++) {
    if (haventStop[agent] == 1) {
      // 如果agent还没有完成路径，则随机决定是否延时
      if (distrib(gen) <= p) {//distrib(gen)产生[1,1000]的随机数，如果随机数小于p，则agent需要延时
        std::cout << "agent " << agent << " is delayed\n";
        delayed_agents[agent] = 1;
        *delay_mark = true;
      }
    }
  }

  // 如果有agent需要延时，则返回0，所有agent不进行移动
  if (*delay_mark) {
    return 0;
  }

  for (int agent = 0; agent < agentCnt; agent++) {
    if (movable[agent] == 1) {
      // 如果agent可以移动，则移动一步
      states[agent] += 1;
      moveCnt += 1;
    }
  }
  if (moveCnt == 0 && timeSpent != 0) {
    // 如果没有agent移动，说明死锁或者任务都已经完成，则返回-1
    return -1;
  }
  return timeSpent;
}

/**
 * @brief 仿真函数，带延时功能，程序里面使用了2种搜索方法，一种是基于graph的搜索方法，一种是基于执行的方法
 * 
 * @param p 延时的概率值，延时概率为(p/1000)
 * @param dlow 延时的最小值，单位：步
 * @param dhigh 延时的最大值，单位：步
 * @param outFile 使用基于graph的搜索方法的结果保存到这个文件
 * @param outFile_slow 使用基于执行的方法的结果保存到这个文件
 * @param outFile_path 输出重新规划时的状态保存文件
 * @param outFile_setup 输出哪个agent延时了，已经延时多久的保存文件
 * @param timeout 搜索的超时限制
 * @return int 
 */
int Simulator::simulate_wdelay(int p, int dlow, int dhigh, ofstream &outFile, ofstream &outFile_slow, ofstream &outFile_path, ofstream &outFile_setup, int timeout) {
  int agentCnt = get_agentCnt(adg);
  int stepSpend = 1;//当前周期还有多少agent可以移动
  bool delay_mark = false;
  vector<int> delayed_agents(agentCnt, 0);

  while (stepSpend != 0) {//如果还有agent可以移动，则继续循环
    stepSpend = step_wdelay(p, &delay_mark, delayed_agents);
    if (delay_mark) // a delay just happened 如果发生了延时
    {
      // 保存重新规划时的当前状态和目标状态到文件
      print_for_replanning(adg, states, outFile_path);
      outFile_path.close();
      // int delayed_state = states[delayed_agent];

      int input_sw_cnt;

      microseconds timer_constructADG(0);//构建延时后的adg消耗的时间
      auto start = high_resolution_clock::now();
      // 重构延时后的ADG
      ADG adg_delayed = construct_delayed_ADG(adg, dlow, dhigh, delayed_agents, states, &input_sw_cnt, outFile_setup);
      outFile_setup.close();
      auto stop = high_resolution_clock::now();
      timer_constructADG += duration_cast<microseconds>(stop - start);

      // 这部分仿真延时后的ADG(没有考虑走过的路径与 考虑走过的路径)，但是都没有重新触发重规划
      ADG adg_delayed_copy = copy_ADG(adg_delayed);
      set_switchable_nonSwitchable(get<0>(adg_delayed_copy));//将所有可交换的边加入不可交换的边中，并且清空可交换的边
      Simulator simulator_original(adg_delayed_copy);//没有考虑当前状态，只是考虑了延时后的ADG
      int originalTime = simulator_original.print_soln();
      Simulator simulator_ori_trunc(adg_delayed_copy, states);//将当前的状态作为起始状态，构造新的Simulator
      int oriTime_trunc = simulator_ori_trunc.print_soln();

      ADG adg_delayed_slow = copy_ADG(adg_delayed);

      // 使用graph的搜索方法进行ADG的重规划
      microseconds timer(0);
      start = high_resolution_clock::now();
      Astar search(timeout, true);//初始化为快速的A*搜索，使用graph的搜索方法
      ADG replanned_adg = search.startExplore(adg_delayed, input_sw_cnt);//重新规划
      stop = high_resolution_clock::now();
      timer += duration_cast<microseconds>(stop - start);

      // 如果规划超时
      if (duration_cast<seconds>(timer).count() >= timeout) {
        outFile << timer.count() << "," << 
        timer.count() + timer_constructADG.count() << ",,,,,";
        search.print_stats(outFile);
        exit(0);
      }
      
      // 仿真重规划的结果
      Simulator simulator_res(replanned_adg);//这个不考虑当前状态，初始状态为0
      int timeSum = simulator_res.print_soln("out.txt");
      Simulator simulator_res_trunc(replanned_adg, states);//这个考虑已经走过的路径，构造新的Simulator
      int timeSum_trunc = simulator_res_trunc.print_soln();

      // 输出搜索结果耗时
      outFile << "plan time:" << timer.count() << "us," << std::endl <<
      "plan+construct time:" << timer.count() + timer_constructADG.count() << "us," << std::endl <<
      "originalTime:" << originalTime << "," << std::endl <<
      "timeSum:" << timeSum << "," << std::endl <<
      "oriTime_trunc:" << oriTime_trunc << "," << std::endl <<
      "timeSum_trunc:" << timeSum_trunc << ",";
      
      search.print_stats(outFile);

      // 使用执行的搜索方法进行ADG的重规划
      microseconds timer_slow(0);
      start = high_resolution_clock::now();
      Astar search_slow(timeout, false);//初始化为慢速的A*搜索，使用执行的搜索方法
      ADG replanned_slow = search_slow.startExplore(adg_delayed_slow, input_sw_cnt);//重新规划
      stop = high_resolution_clock::now();
      timer_slow += duration_cast<microseconds>(stop - start);

      // 如果规划超时
      if (duration_cast<seconds>(timer_slow).count() >= timeout) {
        outFile_slow << timer_slow.count() << "," << 
        timer_slow.count() + timer_constructADG.count() << ",,,,,";
        search_slow.print_stats(outFile_slow);
        exit(0);
      }
      
      Simulator simulator_slow(replanned_slow);
      timeSum = simulator_slow.print_soln();
      Simulator simulator_slow_trunc(replanned_slow, states);
      timeSum_trunc = simulator_slow_trunc.print_soln();

      // 输出搜索结果耗时
      outFile_slow << "plan time:" << timer_slow.count() << "us," << std::endl <<
      "plan+construct time:" << timer_slow.count() + timer_constructADG.count() << "us," << std::endl <<
      "originalTime:" << originalTime << "," << std::endl <<
      "timeSum:" << timeSum << "," << std::endl <<
      "oriTime_trunc:" << oriTime_trunc << "," << std::endl <<
      "timeSum_trunc:" << timeSum_trunc << ",";
      
      search_slow.print_stats(outFile_slow);
      exit(0);
    }
    if (stepSpend < 0) return -1; // stuck
  }
  std::cout << "no delay happened\n";
  return 0;
}

int main(int argc, char** argv) {
  char* fileName = argv[1];//输入的mapf路径
  int p = atoi(argv[2]);//延时的阈值，控制延时的概率[0,1000]
  int dlow = atoi(argv[3]);//最小延时值
  int dhigh = atoi(argv[4]);//最大延时值
  const char* outFileName = argv[5];//使用基于graph的搜索方法的结果保存到这个文件
  const char* outFileName_slow = argv[6];//使用基于执行的方法的结果保存到这个文件
  const char* outFileName_path = argv[7];//重新规划的时候，agent的状态保存到这个文件
  const char* outFileName_setup = argv[8];//输出哪个agent延时了，已经延时多久的保存文件
  int timeout = 90;

  ADG adg = construct_ADG(fileName);//读取路径信息，构造ADG

  ofstream outFile;//使用基于graph的搜索方法的结果保存到这个文件
  outFile.open(outFileName, ios::app);
  ofstream outFile_slow;
  outFile_slow.open(outFileName_slow, ios::app);//使用基于执行的方法的结果保存到这个文件
  ofstream outFile_path;//重新规划的时候，agent的状态保存到这个文件
  outFile_path.open(outFileName_path, ios::app);
  ofstream outFile_setup; //输出哪个agent延时了，已经延时多久的保存文件
  outFile_setup.open(outFileName_setup, ios::app);
  if (outFile.is_open()) {
    // if (print_header) {
    //   outFile << "search time,search + construct time,original total cost,"	
    //   << "solution total cost,original remain cost,solution remain cost,"
    //   << "explored node,pruned node,added node,heuristic time,branch time,"
    //   << "sort time,priority queue time,copy&free graph time,dfs time" << endl;
    // }

    Simulator simulator(adg);
    simulator.simulate_wdelay(p, dlow, dhigh, outFile, outFile_slow, outFile_path, outFile_setup, timeout);
    outFile.close();
    outFile_slow.close();
  }
  return 0;
}