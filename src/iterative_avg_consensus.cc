/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (C) 2017 - Matthieu Lagacherie
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "ns3/log.h"
#include "iterative_avg_consensus.h"
#include "sensor_app.h"

using namespace ns3;
using namespace std;
using namespace boost;

ns3::LogComponent g_log = ns3::LogComponent("IterativeAvgConsensus", "iterative_avg_consensus.h");

/**
 * Initialize the graph object with simulated signal range
 * @param nNodes number of nodes in the simulation
 * @param use_mk use a static graph configuration for debugging
 * @param distance the simulated signal range
 * @param nodes ns3 nodes objects
 * @param G the boost graph
 * @param mobility mobility model used to compute the simulated distances between nodes
 */
void initializeGraph(uint32_t nNodes, uint32_t use_mk, double distance, const NodeContainer &nodes, Graph &G,
                     const MobilityHelper &mobility) {
    if (use_mk > 0) {
        add_edge(0, 3, G);
        add_edge(1, 3, G);
        add_edge(2, 3, G);
        add_edge(1, 2, G);
    } else { /// Connectivity depends on signal range (distance parameter)
        for (uint32_t i = 0; i < nNodes; i++) {
            for (uint32_t j = i + 1; j < nNodes; j++) {
                Ptr<Node> node_i = nodes.Get(i);
                Ptr<Node> node_j = nodes.Get(j);
                double dist_node = sqrt(mobility.GetDistanceSquaredBetween(node_i, node_j));

                if (dist_node < distance) {
                    NS_LOG_DEBUG("extra connectivity: connecting " << i << " to " << j << " " << dist_node);
                    add_edge(i, j, G);
                }
            }
        }
    }
}

/**
 * A post-processing step is applied to the generated graph to ensure that the graph is fully connected.
 * This step iterates on all the nodes and extracts the connected components. If the number of connected components
 * is greater than 1, then a connection is formed to connect the graph components between them.
 * @param nNodes the current graph number of nodes
 * @param G the boost graph object
 */
void graphCorrection(uint32_t nNodes, Graph &G) {
    /// Computes the connected components to ensure a connected graph
    int num = 0;
    std::vector<int> component = get_connected_components(G, &num, nNodes);
    size_t i;

    NS_LOG_INFO("Total number of components: " << num << " vec " << component.size() << " # vertices "
                                               << num_vertices(G));

    /// If the number of components is greater than 1, then we connect the other components to the first
    if (num > 1) {
        set<int> processed;
        int node_id = -1;

        for (i = 0; i < component.size(); ++i) {
            if (component[i] == 0) {
                node_id = i;
                break;
            }
        }
        for (i = 0; i < component.size(); ++i) {
            cerr << "[" << i << " " << component[i] << "]" << endl;
            int cmp_id = component[i];

            if (cmp_id > 0 && processed.find(cmp_id) == processed.end()) {
                NS_LOG_INFO("Connecting node " << i << " in component " << cmp_id << " to " << node_id);
                add_edge(i, node_id, G);
                processed.insert(cmp_id);
            }
        }
        cerr << endl;
        component = get_connected_components(G, &num, nNodes);
        NS_LOG_INFO("Total number of components after processing: " << num);
    }
}

/**
 * This function writes the current graph for networkx plotting
 */
void writeGraph(uint32_t nNodes, const string &output_path, const NodeContainer &nodes, const Graph &G) {
    /// Writing graph informations for networkx plotting
    ofstream graph;
    graph.open(output_path + "/graph.tsv");

    /// Node positions
    for (uint32_t i = 0; i < nNodes; i++) {
        Ptr<Node> node = nodes.Get(i);
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        Vector current = mobility->GetPosition();

        graph << i << "\t" << current.x << "\t" << current.y << endl;
    }

    for (uint32_t i = 0; i < nNodes; i++) {
        for (uint32_t j = i + 1; j < nNodes; j++) {
            if (edge(i, j, G).second) {
                graph << i << "\t" << j << endl;
            }
        }
    }
    graph.close();
}

/// Initializing point to point connectivity
void initializePoint2Point(uint32_t nNodes, const NodeContainer &nodes, const Graph &G,
                           NetDeviceContainer &devices) {
    uint32_t num_networks = 1;

    for (uint32_t i = 0; i < nNodes; i++) {
        for (uint32_t j = i + 1; j < nNodes; j++) {
            if (edge(i, j, G).second) {
                PointToPointHelper pointToPoint;
                pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
                pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

                NodeContainer endpoints;
                endpoints.Add(nodes.Get(i));
                endpoints.Add(nodes.Get(j));

                NetDeviceContainer ndc = pointToPoint.Install(endpoints);

                devices.Add(ndc);
                Ipv4AddressHelper address;
                Ipv4Address network((num_networks << 3) | 0x0a000000);

                Ipv4Mask mask(0xfffffff8);

                address.SetBase(network, mask);

                address.Assign(ndc);
                ++num_networks;
            }
        }
    }
}

/**
 * Main simulation program
 */
int
main(int argc, char *argv[]) {
    uint32_t nNodes = 2;
    uint32_t secs_to_run = 10;
    uint32_t use_mk = 0;
    std::string output_path = "/tmp";
    uint32_t self_stab = 0;
    std::string random_gen_pos("ns3::UniformRandomVariable[Min=0.0|Max=100.0]");
    double distance = 30;
    int is_async = 0;
    int seed = 42;
    int run_id = 0;
    int graph_correction = 1;

    CommandLine cmd;

    cmd.AddValue("nNodes", "Number of nodes in the simulation", nNodes);
    cmd.AddValue("secsToRun", "Number of seconds to simulate", secs_to_run);
    cmd.AddValue("makhoul", "Use the example of makhoul thesis", use_mk);
    cmd.AddValue("path", "Output path", output_path);
    cmd.AddValue("self_stab", "Self-stabilization mode", self_stab);
    cmd.AddValue("positionRng", "Define the random generator for the position allocator", random_gen_pos);
    cmd.AddValue("distance", "Define the maximum signal range", distance);
    cmd.AddValue("async", "Set async mode for diffusion algorithm", is_async);
    cmd.AddValue("seed", "Set seed value (simulation reproducibility)", seed);
    cmd.AddValue("run_id", "Set run id (simulation reproducibility)", run_id);
    cmd.AddValue("graph_correction", "Run the graph correction algorithm for non-connected components",
                 graph_correction);

    cmd.Parse(argc, argv);

    Time::SetResolution(Time::MS);
    SeedManager::SetSeed(seed);
    SeedManager::SetRun(run_id);

    if (use_mk > 0)
        nNodes = 4;

    NodeContainer nodes;
    nodes.Create(nNodes);
    Graph G(nNodes);

    /// Position Allocator: allocate the nodes of the simulated map using random_gen_pos
    AsciiTraceHelper ascii;
    MobilityHelper mobility;

    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                  "X", StringValue(random_gen_pos),
                                  "Y", StringValue(random_gen_pos));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.EnableAsciiAll(ascii.CreateFileStream("positions.tr"));
    mobility.Install(nodes);


    /// Initializing graph corresponding to network nodes
    /// Static graph generation
    initializeGraph(nNodes, use_mk, distance, nodes, G, mobility);

    if (graph_correction > 0) {
        graphCorrection(nNodes, G);
    }

    writeGraph(nNodes, output_path, nodes, G);

    InternetStackHelper stack;
    stack.Install(nodes);

    NetDeviceContainer devices;
    initializePoint2Point(nNodes, nodes, G, devices);

    /// Initializing device energy model
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10));
    EnergySourceContainer nrj_sources = basicSourceHelper.Install(nodes);
    DeviceEnergyModelContainer nrj_models;

    for (uint32_t i = 0; i < nNodes; i++) {
        Ptr<Node> node = nodes.Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ptr<NetDevice> dev = ipv4->GetNetDevice(0);

        Ptr<EnergySource> nrj_source = node->GetObject<EnergySourceContainer>()->Get(0);
        Ptr<WifiRadioEnergyModel> nrj_model = CreateObject<WifiRadioEnergyModel>();

        nrj_model->SetEnergySource(nrj_source);
        nrj_source->AppendDeviceEnergyModel(nrj_model);

        nrj_models.Add(nrj_model);
    }

    /// Simulation run
    SensorDataFactory *factory = new SensorDataFactory();
    SensorAppHelper sensor_app(4242, factory, use_mk, self_stab, is_async);
    ApplicationContainer apps;

    apps.Add(sensor_app.Install(nodes));

    apps.Start(Seconds(0.0));
    apps.Stop(Seconds((secs_to_run + 1) * 1.0));

    Simulator::Stop(Seconds(secs_to_run));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}

