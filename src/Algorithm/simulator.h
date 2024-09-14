#ifndef SIMULATOR
#define SIMULATOR

#include "../ADG/generate_ADG.h"

class Simulator {
  public:         
    ADG adg;//当前的adg
    vector<int> states;//每个agent当前的状态，记录每个agent走到路径的哪个节点
    
    // 通过adg初始化当前机器人状态，会初始化states和adg
    Simulator(ADG adg);

    /**
     * @brief 初始化仿真器
     * 
     * @param adg 
     * @param visited_states agent已经走到哪个节点
     */
    Simulator(ADG adg, vector<int> visited_states);

    /**
     * @brief 仿真的单步运行
     * 
     * @param switchCheck 
     * @return int 
     */
    int step(bool switchCheck);

    // 遍历每个agent，查看他们下一个状态的依赖是否都已经完成，如果完成则可以移动，否则不能移动
    // 输出movable: 表示agent能否移动的列表，如果movable[i] == 1，则agent i 能移动，否则不能移动
    // 输出haventStop: 表示agent是否已经停止，如果还没有完成路径，则设置为1，否则设置为0
    // 返回timeSpent: 表示还有多少个机器人正在移动
    int checkMovable(vector<int>& movable, vector<int>& haventStop);

    // 遍历每个agent，查看他们下一个状态的依赖是否都已经完成，如果完成则可以移动，否则不能移动
    // 输出movable: 表示agent能否移动的列表，如果movable[i] == 1，则agent i 能移动，否则不能移动
    // 返回timeSpent: 表示还有多少个机器人正在移动
    int checkMovable(vector<int>& movable);
    void print_location(ofstream &outFile, Location location);
    int print_soln(const char* outFileName);
    int print_soln();

    bool incident_to_switchable(int *v_from, int *v_to);


    int step_wdelay(int p, bool *delay_mark, vector<int> &delayed_agents);
    // bool amove(vector<int>& moved, int agent, int *timeSpent, int *delayer, int p, int d);
    int simulate_wdelay(int p, int dlow, int dhigh, ofstream &outFile, ofstream &outFile_slow, ofstream &outFile_path, ofstream &outFile_setup, int timeout);
};
#endif