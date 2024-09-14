# Switchable-Edge-Search

This repository is the implementation of **A Real-Time Rescheduling Algorithm for Multi-robot Plan Execution** published in ICAPS 2024.

The source code for K-Robust CBS is available at https://github.com/nobodyczcz/Lazy-Train-and-K-CBS. The "wait-on-start" branch supports the delayed situation that we considered.

**这个代码里面有ADG的实现，可以参考**

Build:

```bash
g++ -Wall -Werror -Ofast src/types.h src/graph/*.h src/graph/*.cpp src/ADG/*.h src/ADG/*.cpp src/Algorithm/*.h src/Algorithm/*.cpp src/simulate.cpp

# 增加了cmakelists
mkdir build
cd build
cmake ..
make -j10
```

Run:
```bash
./a.out {path input file} {delay probability -- an integer in [0, 1000]} {lower bound of the delay length -- an integer} {upper bound of the delay length -- an integer} {output file (stats) for the graph-based module} {output file (stats) for the execution-based module} {output file for the start and goal locations when a delay happens} {output file for the index of the delayed agents and the length of the delay}
```

Example:
```bash
./a.out example/path.txt 10 10 20 stats_graph.csv stats_exec.csv locations.txt delay_setup.txt

# build文件夹下
./a.out ../example/path.txt 10 10 20 stats_graph.csv stats_exec.csv locations.txt delay_setup.txt
```

The above command reads an example path, uses a delay probability = 1 percent and a delay length range = [10, 20], and creates two .csv files to record the output stats and two .txt files to record the information about the delay.

上述命令读取一个示例路径，使用延迟概率为1%，延迟时长范围为[10, 20]，并创建两个.csv文件记录输出统计信息，以及两个.txt文件记录延迟的相关信息。

The stats columns are (from left to right) 状态文件中每一列的含义:

runtime || runtime + the time for constructing the TPG || original total cost of the TPG || replanned total cost || original remaining cost of the TPG (the parts after the delay) || replanned remaining cost

The rest columns are about runtime breakdown and the number of search nodes.
