# iterative-average-consensus
Iterative average consensus algorithms simulation on NS3

## Building

Install anaconda and create a new environment.
https://www.continuum.io/downloads

```
apt-get install bzip2 g++ git parallel screen
wget https://repo.continuum.io/archive/Anaconda2-4.4.0-Linux-x86_64.sh
./Anaconda2-4.4.0-Linux-x86_64.sh -b -p /opt/anaconda

conda create --name ns3 python=2.7
source activate ns3
conda install -y notebook pandas matplotlib networkx
conda install -y -c conda-forge jupyter_contrib_nbextensions jupyter_nbextensions_configurator
conda install -y -c conda-forge boost cmake
```

Build the project

```
source activate ns3
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Running simulations

```
$ ./iterative_avg_consensus --help
iterative_avg_consensus [Program Arguments] [General Arguments]

Program Arguments:
    --nNodes:            Number of nodes in the simulation [2]
    --secsToRun:         Number of seconds to simulate [10]
    --makhoul:           Use the example of makhoul thesis [0]
    --path:              Output path [/tmp]
    --self_stab:         Self-stabilization mode [0]
    --positionRng:       Define the random generator for the position allocator [ns3::UniformRandomVariable[Min=0.0|Max=100.0]]
    --distance:          Define the maximum signal range [30]
    --async:             Set async mode for diffusion algorithm [0]
    --seed:              Set seed value (simulation reproducibility) [42]
    --run_id:            Set run id (simulation reproducibility) [0]
    --graph_correction:  Run the graph correction algorithm for non-connected components [1]

General Arguments:
    --PrintGlobals:              Print the list of globals.
    --PrintGroups:               Print the list of groups.
    --PrintGroup=[group]:        Print all TypeIds of group.
    --PrintTypeIds:              Print all TypeIds.
    --PrintAttributes=[typeid]:  Print all attributes of typeid.
    --PrintHelp:                 Print this help message.
```

