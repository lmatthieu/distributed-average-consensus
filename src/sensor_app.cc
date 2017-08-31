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

#include "iterative_avg_consensus.h"
#include "sensor_app.h"

namespace ns3 {

    SensorApp::SensorApp() {
        m_sent = 0;
        m_nit = 0;
        m_send_event = EventId();
        m_print_event = EventId();

        /// generating initial state of the node
        m_z = generate_random_sensor_data();

        /// self-stab initial parameters
        m_epsilon = 0.0001;
        m_is_self_stab = false;
        m_broadcast = true;
        m_async = false;
    }

    SensorApp::~SensorApp() {
        uint32_t i;
        for (i = 0; i < m_num_peers; i++) {
            m_send_sockets[i] = 0;
        }
        delete[] m_send_sockets;
        delete[] m_peer_addresses;
        m_recv_socket = 0;
    }

    void SensorApp::StartApplication(void) {
        m_sensor_data->SetNodeId(GetNode()->GetId());
        m_sensor_data->Update(SENSOR_INIT, m_z);
        m_x[GetNode()->GetId()] = m_z;
        m_update_state = false;
        /// Self-stab variables
        m_invitation[GetNode()->GetId()] = SELF_STAB_NULL;
        m_acceptance[GetNode()->GetId()] = SELF_STAB_NULL;
        m_future[GetNode()->GetId()] = m_z;
        m_current[GetNode()->GetId()] = m_z;
        /// Initializing broadcast states
        m_broadcast = true;
        m_invitation_updated = false;
        m_acceptance_updated = false;

        /// Recv socket initialization
        if (m_recv_socket == 0) {
            TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
            m_recv_socket = Socket::CreateSocket(GetNode(), tid);
            InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
            m_recv_socket->Bind(local);
            NS_LOG_DEBUG("Setting up receiving socket " << local);
        }
        m_recv_socket->SetRecvCallback(MakeCallback(&SensorApp::HandleMessage, this));

        /// Initializing sockets to neighbors
        m_num_peers = GetNumNeighbors();
        m_peer_addresses = new Ipv4Address[m_num_peers];
        m_send_sockets = new Ptr<Socket>[m_num_peers];
        m_peer_nodes = new Ptr<Node>[m_num_peers];
        NS_LOG_DEBUG("setting up " << m_num_peers << " peers");

        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        uint32_t peers = 0;
        for (uint32_t iface = 0; iface < ipv4->GetNInterfaces(); iface++) {
            if (!IsLoopback(iface)) {
                Ipv4Address peerAddr = GetNeighborAddress(iface);
                Ptr<Socket> ssock = m_send_sockets[peers];

                if (ssock == 0) {
                    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
                    ssock = Socket::CreateSocket(GetNode(), tid);
                    m_peer_addresses[peers] = peerAddr;
                    m_peer_nodes[peers] = GetNeighborNode(iface);
                    ssock->Bind();
                    ssock->Connect(InetSocketAddress(peerAddr, m_port));

                    NS_LOG_INFO("established socket to peer node "
                                        << GetNeighborNode(iface)->GetId()
                                        << " at " << peerAddr << ":" << m_port
                                        << " ["
                                        << GetNode()->GetId()
                                        << " -> "
                                        << m_peer_nodes[peers]->GetId() << "]");
                    m_send_sockets[peers] = ssock;
                }
                peers++;
            }
        }

        m_print_event = Simulator::Schedule(MilliSeconds(50), &SensorApp::PrintState, this);
        if (m_is_self_stab == false) {
            m_send_event = Simulator::Schedule(MilliSeconds(10), &SensorApp::SendState, this);
        } else {
            m_send_event = Simulator::Schedule(MilliSeconds(10), &SensorApp::SelfStabSendState, this);
        }
    }

    void SensorApp::StopApplication(void) {
        m_sensor_data->LogMemory();

        if (m_recv_socket != 0) {
            m_recv_socket->Close();
            m_recv_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
            m_recv_socket = 0;
        }
        uint32_t i;
        for (i = 0; i < m_num_peers; i++) {
            Ptr<Socket> sock = m_send_sockets[i];
            if (sock != 0) {
                sock->Close();
                sock->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> >());
            }
            m_send_sockets[i] = sock = 0;
        }

        Simulator::Cancel(m_send_event);
        Simulator::Cancel(m_iter_event);
        Simulator::Cancel(m_update_event);
        Simulator::Cancel(m_print_event);
    }

    bool SensorApp::IsLoopback(uint32_t iface) {
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();

        for (uint32_t i = 0; i < ipv4->GetNAddresses(iface); i++) {
            if (ipv4->GetAddress(iface, i).GetLocal() == Ipv4Address::GetLoopback())
                return true;
        }
        return false;
    }

    uint32_t SensorApp::GetNumNeighbors() {
        uint32_t out = 0;
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();

        for (uint32_t itf = 0; itf < ipv4->GetNInterfaces(); itf++) {
            if (!IsLoopback(itf))
                ++out;
        }
        return out;
    }

    Ipv4Address SensorApp::GetNeighborAddress(uint32_t itf) {
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<NetDevice> dev = ipv4->GetNetDevice(itf);
        Ptr<Channel> chan = dev->GetChannel();
        Ptr<NetDevice> otherDev;

        for (uint32_t i = 0; i < chan->GetNDevices(); i++) {
            otherDev = chan->GetDevice(i);
            if (otherDev->GetAddress() != dev->GetAddress())
                break;
        }
        Ptr<Ipv4> otherIpv4 = otherDev->GetNode()->GetObject<Ipv4>();
        uint32_t otherIface = otherIpv4->GetInterfaceForDevice(otherDev);
        Ipv4InterfaceAddress iaddr = otherIpv4->GetAddress(otherIface, 0);

        return iaddr.GetLocal();
    }

    Ptr<Node> SensorApp::GetNeighborNode(uint32_t itf) {
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<NetDevice> dev = ipv4->GetNetDevice(itf);
        Ptr<Channel> chan = dev->GetChannel();
        Ptr<NetDevice> otherDev;

        for (uint32_t i = 0; i < chan->GetNDevices(); i++) {
            otherDev = chan->GetDevice(i);
            if (otherDev->GetAddress() != dev->GetAddress())
                break;
        }
        return otherDev->GetNode();
    }

    void SensorApp::SendState(void) {
        //NS_LOG_INFO("NODE " << GetNode()->GetId() << " SEND STATE TO PEERS");
        std::stringstream addr_str;

        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        for (uint32_t iface = 0; iface < ipv4->GetNInterfaces(); iface++) {
            for (uint32_t addr_idx = 0; addr_idx < ipv4->GetNAddresses(iface);
                 addr_idx++) {
                Ipv4InterfaceAddress iaddr = ipv4->GetAddress(iface, addr_idx);
                Ipv4Address addr = iaddr.GetLocal();
                if (addr == Ipv4Address::GetLoopback())
                    continue;
                addr_str << "|" << addr;
            }
        }
        //NS_LOG_INFO("\tCurrent Identity " << addr_str.str());

        m_sensor_data->SetNodeId(GetNode()->GetId());

        // On envoie l'état courant à chacun des voisins connectés
        for (size_t v_idx = 0; v_idx < GetNumNeighbors(); ++v_idx) {
            // Pour chaque peer nous mettons à jour l'état courant du noeud
            m_sensor_data->Update(SENSOR_INIT, m_x[GetNode()->GetId()]);

            ChangeState(WifiPhy::TX);

            // Initialisation du buffer d'envoi
            uint32_t bufsz = m_sensor_data->GetMarshalledSize();
            uint8_t *buf = new uint8_t[bufsz];
            m_sensor_data->MarshalTo(buf);
            Ptr<Packet> p = Create<Packet>(buf, bufsz);

            // Envoi du paquet réseau
            m_send_sockets[v_idx]->Send(p);

            delete[] buf;
            ++m_sent;
            /*Ipv4Address peerAddress = m_peer_addresses[v_idx];
            NS_LOG_INFO("\tSending current state to "
                                << m_peer_nodes[v_idx]->GetId()
                                << "[" << peerAddress << "] "
                                << m_sensor_data->ToString());*/
        }
    }

    void SensorApp::SendUpdate(void) {
        NS_LOG_FUNCTION(this);
        std::stringstream addr_str;

        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        for (uint32_t iface = 0; iface < ipv4->GetNInterfaces(); iface++) {
            for (uint32_t addr_idx = 0; addr_idx < ipv4->GetNAddresses(iface);
                 addr_idx++) {
                Ipv4InterfaceAddress iaddr = ipv4->GetAddress(iface, addr_idx);
                Ipv4Address addr = iaddr.GetLocal();
                if (addr == Ipv4Address::GetLoopback())
                    continue;
                addr_str << "|" << addr;
            }
        }
        NS_LOG_INFO("NODE " << GetNode()->GetId()
                            << " SENDING UPDATES");

        m_sensor_data->SetNodeId(GetNode()->GetId());

        // On envoie le message de maj chacun des voisins connectés
        //if (m_s.size() > 0)
        {
            for (size_t v_idx = 0; v_idx < GetNumNeighbors(); ++v_idx) {
                uint32_t node_id = m_peer_nodes[v_idx]->GetId();

                if (m_s.find(node_id) != m_s.end()) {
                    m_sensor_data->Update(SENSOR_RUN, m_s[node_id]);
                } else {
                    m_sensor_data->Update(SENSOR_RUN, 0);
                    //continue;
                }

                ChangeState(WifiPhy::TX);

                uint32_t bufsz = m_sensor_data->GetMarshalledSize();
                uint8_t *buf = new uint8_t[bufsz];
                m_sensor_data->MarshalTo(buf);
                Ptr<Packet> p = Create<Packet>(buf, bufsz);

                m_send_sockets[v_idx]->Send(p);

                delete[] buf;
                ++m_sent;
                //Ipv4Address peerAddress = m_peer_addresses[v_idx];

                NS_LOG_INFO("\tSending message from " << GetNode()->GetId()
                                                      << " to "
                                                      << m_peer_nodes[v_idx]->GetId()
                                                      << " ["
                                                      << m_sensor_data->ToString()
                                                      << "]");

            }
        }
    }

    void SensorApp::UpdateState() {
        uint32_t node_id = GetNode()->GetId();
        double sum_s_ij = 0;
        double sum_r_ij = 0;
        double new_state = 0;

        NS_LOG_INFO("NODE " << GetNode()->GetId()
                            << " UPDATING STATES "
                            << m_r.size()
                            << " " << m_x.size());

        for (auto it : m_s) {
            if (it.first != node_id)
                sum_s_ij += it.second;
        }
        for (auto it : m_r) {
            if (it.first != node_id)
                sum_r_ij += it.second;
        }
        new_state = m_x[node_id] - sum_s_ij + sum_r_ij;
        m_x[node_id] = new_state;

        m_send_event = Simulator::Schedule(MilliSeconds(10), &SensorApp::SendState, this);
        m_r.clear();
        //m_x.clear();
        //m_x[node_id] = new_state;

        NS_LOG_INFO("\tCURRENT STATE ["
                            << node_id << "] "
                            << m_z << " "
                            << m_x[node_id]
                            << " - " << sum_s_ij << " + " << sum_r_ij);


    }

    void SensorApp::HandleMessage(Ptr<Socket> socket) {
        std::stringstream addr_str;

        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        for (uint32_t iface = 0; iface < ipv4->GetNInterfaces(); iface++) {
            for (uint32_t addr_idx = 0; addr_idx < ipv4->GetNAddresses(iface);
                 addr_idx++) {
                Ipv4InterfaceAddress iaddr = ipv4->GetAddress(iface, addr_idx);
                Ipv4Address addr = iaddr.GetLocal();
                if (addr == Ipv4Address::GetLoopback())
                    continue;
                addr_str << "|" << addr;
            }
        }
        NS_LOG_INFO("NODE " << GetNode()->GetId()
                            << " RECEIVING DATA FROM PEERS");
        NS_LOG_INFO("\tCurrent identity" << addr_str.str());

        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            ChangeState(WifiPhy::RX);

            Ipv4Address src = InetSocketAddress::ConvertFrom(from).GetIpv4();
            uint32_t bufsz = packet->GetSize();
            uint8_t *buf = new uint8_t[bufsz];
            packet->CopyData(buf, bufsz);

            SensorData rcv_msg;
            rcv_msg.MarshalFrom(buf);
            /*NS_LOG_INFO("\tReceiving message from [" << src << "] "
                                                     << rcv_msg.ToString());*/

            m_ip_dic[src.Get()] = rcv_msg.GetNodeId();
            if (m_is_self_stab == false) {
                if (rcv_msg.GetType() == SENSOR_INIT)
                    m_x[rcv_msg.GetNodeId()] = rcv_msg.GetValue();
                else
                    m_r[rcv_msg.GetNodeId()] = rcv_msg.GetValue();
            } else {
                m_invitation[rcv_msg.GetNodeId()] = rcv_msg.GetInvitation();
                m_acceptance[rcv_msg.GetNodeId()] = rcv_msg.GetAcceptance();
                m_current[rcv_msg.GetNodeId()] = rcv_msg.GetValue();
                m_future[rcv_msg.GetNodeId()] = rcv_msg.GetFuture();
            }
            delete[] buf;
        }
        // Starting iteration in diffusion mode
        if (m_is_self_stab == false) {
            NS_LOG_INFO("Diffusion mode " << m_async << " -- Checking states -- "
                                          << m_r.size() << " "
                                          << m_x.size() << " == " << GetNumNeighbors() + 1);

            if ((m_async && m_r.size() > 0 && m_update_state)
                || (m_async == false && m_r.size() == GetNumNeighbors() && m_update_state)) {
                NS_LOG_INFO("Diffusion mode " << m_async << " -- Updating states");
                UpdateState();
                m_update_state = false;
            }
            if ((m_async && m_x.size() > 1 && m_update_state == false)
                || (m_async == false && m_x.size() == GetNumNeighbors() + 1
                    && m_update_state == false)) {
                NS_LOG_INFO("Diffusion mode " << m_async << " -- Iterating");
                Iterate();
                m_update_state = true;
            }
        } else {
            if (m_current.size() == GetNumNeighbors() + 1)
                SelfStabIterate();
        }
    }

    double SensorApp::node_delta(double v) {
        double current = m_x[GetNode()->GetId()];

        if (current > v)
            return current - v;
        else
            return 0;
    }

    void SensorApp::Iterate() {
        NS_LOG_INFO("NODE " << GetNode()->GetId() << " ITERATING");

        std::map<uint32_t, double> s_ij, alpha_ij;
        std::vector<uint32_t> delta_id;
        std::vector<double> delta_value;

        NS_LOG_INFO("\tPeer current states " << m_x[GetNode()->GetId()]);
        for (auto it : m_x) {
            NS_LOG_INFO("\t- " << it.first << " -> " << it.second);
            double delta_v = node_delta(it.second);

            if (delta_v > 0) {
                s_ij[it.first] = 0;
                alpha_ij[it.first] = 0;
                delta_id.push_back(it.first);
                delta_value.push_back(delta_v);
            }
        }

        // Tri du vecteur delta
        auto p = sort_permutation(delta_value, std::greater<double>());
        delta_value = apply_permutation(delta_value, p);
        delta_id = apply_permutation(delta_id, p);

        // debug only
        {
            std::stringstream dbg;

            for (size_t i = 0; i < delta_id.size(); ++i)
                dbg << "[" << delta_id[i] << " " << delta_value[i] << "]";
            NS_LOG_INFO("\tSorted delta values [" << dbg.str() << "]");
        }

        // on parcours la liste des deltas positifs
        double sum = 0;

        for (size_t i = 0; i < delta_id.size(); ++i) {
            uint32_t id_il = delta_id[i];
            double delta_il = delta_value[i];
            double x_il = m_x[id_il];
            double alpha_il = 1. / (node_eta() + 1.);
            double s_il = alpha_il * delta_il;


            NS_LOG_INFO("\tChecking ping-pong condition "
                                << " with [" << id_il << " " << alpha_il << " * " << delta_il
                                << " = " << s_il
                                << "] "
                                << m_x[GetNode()->GetId()]
                                << " - "
                                << sum
                                << " >= "
                                << x_il << " + " << s_il);
            if (!(m_x[GetNode()->GetId()] - sum >= x_il + s_il))
                break;
            NS_LOG_INFO("\tChecking ping-pong -- OK");

            s_ij[id_il] = s_il;
            alpha_ij[id_il] = alpha_il;
            sum += s_il;
        }

        for (auto it : s_ij) {
            NS_LOG_INFO("\tResult for node " << it.first
                                             << "\tState " << m_x[it.first]
                                             << "\tS_ij " << s_ij[it.first]
                                             << "\tAlpha_ij " << alpha_ij[it.first]);
        }

        m_s = s_ij;
        m_r.clear();
        double current_state = m_x[GetNode()->GetId()];
        m_x.clear();
        m_x[GetNode()->GetId()] = current_state;

        m_sensor_data->Update(SENSOR_RUN, m_sensor_data->GetNodeId());

        m_send_event = Simulator::Schedule(MilliSeconds(10), &SensorApp::SendUpdate, this);
        //m_update_event = Simulator::Schedule(Seconds(0.5), &SensorApp::UpdateState, this);
        //m_iter_event = Simulator::Schedule(Seconds(1), &SensorApp::Iterate, this);

        m_nit++;
    }

    uint32_t SensorApp::GetInvitation(uint32_t id) {
        if (m_invitation.find(id) == m_invitation.end()) {
            m_invitation[id] = SELF_STAB_NULL;
        }
        return m_invitation[id];
    }

    uint32_t SensorApp::GetAcceptance(uint32_t id) {
        if (m_acceptance.find(id) == m_acceptance.end()) {
            m_acceptance[id] = SELF_STAB_NULL;
        }
        return m_acceptance[id];
    }

    double SensorApp::GetCurrent(uint32_t id) {
        if (m_current.find(id) == m_current.end()) {
            m_current[id] = std::numeric_limits<double>::quiet_NaN();
        }
        return m_current[id];
    }

    double SensorApp::GetFuture(uint32_t id) {
        if (m_future.find(id) == m_future.end()) {
            m_future[id] = std::numeric_limits<double>::quiet_NaN();
        }
        return m_future[id];
    }

    void SensorApp::SelfStabInvitation() {
        uint32_t i_id = GetNode()->GetId();

        if (GetInvitation(i_id) == SELF_STAB_NULL && GetAcceptance(i_id) == SELF_STAB_NULL) {
            double x_i = GetCurrent(i_id);
            std::vector<uint32_t> delta_id;
            std::vector<double> delta_value;

            for (auto it : m_current) {
                uint32_t j_id = it.first;
                double j_i = it.second;
                double delta_v = j_i - x_i;

                if (delta_v > m_epsilon) {
                    delta_value.push_back(delta_v);
                    delta_id.push_back(j_id);
                    //m_invitation[i_id] = j_id;
                    //NS_LOG_INFO("\t** INVITATION FROM " << i_id << " TO " << j_id);
                    //break;
                }
            }

            if (delta_value.size() > 0) {
                // Tri du vecteur delta
                auto p = sort_permutation(delta_value, std::greater<double>());
                delta_value = apply_permutation(delta_value, p);
                delta_id = apply_permutation(delta_id, p);


                // debug only
                {
                    std::stringstream dbg;

                    for (size_t i = 0; i < delta_id.size(); ++i)
                        dbg << "[" << delta_id[i] << " " << delta_value[i] << "]";
                    NS_LOG_INFO("\tSorted delta values [" << dbg.str() << "]");
                }
                random_shuffle(delta_id.begin(), delta_id.end());
                m_invitation[i_id] = delta_id.front();
                NS_LOG_INFO("\t** INVITATION FROM " << i_id << " TO " << delta_id.front());
                m_invitation_updated = true;
            }
        }
    }

    void SensorApp::SelfStabAccept() {
        uint32_t i_id = GetNode()->GetId();
        double i_x = GetCurrent(i_id);
        std::vector<uint32_t> delta_id;
        std::vector<double> delta_value;

        for (auto it_j : m_current) {
            uint32_t j_id = it_j.first;

            // TODO ajouter i != j
            if (j_id != i_id) {
                double j_x = it_j.second;
                double delta_v = i_x - j_x;

                NS_LOG_INFO("NODE " << i_id << " CHECKING ACCEPTANCE " << j_id << " "
                                    << delta_v << ">" << m_epsilon
                                    << " && " << GetInvitation(j_id) << "==" << i_id
                                    << " && " << GetAcceptance(i_id) << "==" << SELF_STAB_NULL
                                    << " && " << GetInvitation(i_id) << "==" << SELF_STAB_NULL);
                if (delta_v > m_epsilon && GetInvitation(j_id) == i_id
                    && GetAcceptance(i_id) == SELF_STAB_NULL && GetInvitation(i_id) == SELF_STAB_NULL) {
                    delta_id.push_back(j_id);
                    delta_value.push_back(delta_v);

                }
            }
        }

        if (delta_value.size() > 0) {
            // Tri du vecteur delta
            auto p = sort_permutation(delta_value, std::greater<double>());
            delta_value = apply_permutation(delta_value, p);
            delta_id = apply_permutation(delta_id, p);
            uint32_t j_id = 0;

            // debug only
            {
                std::stringstream dbg;

                for (size_t i = 0; i < delta_id.size(); ++i)
                    dbg << "[" << delta_id[i] << " " << delta_value[i] << "]";
                NS_LOG_INFO("\tAcceptations sorted delta values [" << dbg.str() << "]");
            }
            //random_shuffle(delta_id.begin(), delta_id.end());

            j_id = delta_id.front();
            NS_LOG_INFO("\t** ACCEPTANCE FROM " << i_id << " TO " << j_id);
            m_acceptance[i_id] = j_id;
            m_future[i_id] = (GetCurrent(i_id) + GetCurrent(j_id)) / 2.0;
            m_acceptance_updated = true;
        }
    }

    void SensorApp::SelfStabConfirm() {
        uint32_t i_id = GetNode()->GetId();

        for (auto it_j : m_current) {
            uint32_t j_id = it_j.first;

            if (j_id != i_id) {
                if (GetAcceptance(j_id) == i_id && GetInvitation(i_id) == j_id
                    && GetFuture(i_id) != (GetCurrent(i_id) + GetCurrent(j_id)) / 2.0
                    && GetFuture(i_id) != (GetCurrent(i_id) + GetFuture(j_id)) / 2.0// correction boucle confirm
                        ) {
                    m_future[i_id] = (GetCurrent(i_id) + GetCurrent(j_id)) / 2.0;
                    NS_LOG_INFO("\t** CONFIRMATION FROM " << i_id << " TO " << j_id);
                    m_acceptance_updated = true;
                    break;
                }
            }
        }
    }

    void SensorApp::SelfStabCommit() {
        uint32_t i_id = GetNode()->GetId();

        for (auto it_j : m_current) {
            uint32_t j_id = it_j.first;

            if (j_id != i_id) {
                if (((GetAcceptance(j_id) == i_id && GetInvitation(i_id) == j_id)
                     || (GetInvitation(j_id) == i_id && GetAcceptance(i_id) == j_id))
                    && GetFuture(i_id) == GetFuture(j_id)
                    && GetCurrent(i_id) != GetFuture(j_id)) {
                    NS_LOG_INFO("\t** COMMIT FROM " << i_id << " TO " << j_id);
                    m_current[i_id] = m_future[i_id];
                    glb_shared_values[i_id] = m_current[i_id];
                    m_broadcast = true;
                }
            }
        }
    }

    void SensorApp::SelfStabRelease() {
        uint32_t i_id = GetNode()->GetId();

        for (auto it_j : m_current) {
            uint32_t j_id = it_j.first;

            if (j_id != i_id) {
                if (((GetAcceptance(j_id) == i_id && GetInvitation(i_id) == j_id)
                     || (GetInvitation(j_id) == i_id && GetAcceptance(i_id) == j_id))
                    && GetCurrent(i_id) == GetCurrent(j_id)) {
                    NS_LOG_INFO("\t** RELEASING FROM " << i_id << " TO " << j_id);
                    m_invitation[i_id] = SELF_STAB_NULL;
                    m_acceptance[i_id] = SELF_STAB_NULL;
                    m_invitation_updated = true;
                    m_acceptance_updated = true;
                    m_to_update.insert(j_id);
                }
            }
        }
    }

    void SensorApp::SelfStabCorrect() {
        uint32_t i_id = GetNode()->GetId();
        double i_x = GetCurrent(i_id);

        for (auto it_j : m_current) {
            uint32_t j_id = it_j.first;
            double j_x = GetCurrent(j_id);
            double delta_v = j_x - i_x;

            if (j_id != i_id) {
                // AJout d'une condition sinon tout les opérateur sont bloqués par des invitations
                //NS_LOG_INFO("CHECKING INVITATION " << delta_v);
                if (GetInvitation(i_id) == j_id
                    && !(GetAcceptance(j_id) == i_id &&
                         GetCurrent(j_id) != GetFuture(j_id)) // on efface pas avant le commit
                    && ((//0 <= delta_v && suppression de ce cas pour nettoyer les invitations avec delta negatifs
                                delta_v <= m_epsilon)
                        || (GetAcceptance(j_id) != SELF_STAB_NULL
                            && GetAcceptance(j_id) != i_id))) {
                    NS_LOG_INFO("\t** CORRECTION INVITATION FROM " << i_id << " TO " << j_id);
                    m_invitation[i_id] = SELF_STAB_NULL;
                    m_invitation_updated = true;
                    m_to_update.insert(j_id);
                }
                if (GetAcceptance(i_id) == j_id && (GetInvitation(j_id) != i_id
                                                    || (0 <= i_x - j_x && i_x - j_x <= m_epsilon))) {
                    NS_LOG_INFO("\t** CORRECTION ACCEPTANCE FROM " << i_id << " TO " << j_id);
                    m_acceptance[i_id] = SELF_STAB_NULL;
                    m_acceptance_updated = true;
                    m_to_update.insert(j_id);
                }
            }
        }
    }

    void SensorApp::SelfStabIterate() {
        uint32_t node_id = GetNode()->GetId();
        double sum = 0;
        std::stringstream str;

        NS_LOG_INFO("============= NODE " << node_id << " ITERATING =================");
        str << "\n\tVALUES ";
        for (auto it : glb_shared_values) {
            str << "\t[" << it.first << " -> " << it.second << "]";
            sum += it.second;
        }
        str << " *********** SUM " << sum << std::endl;

        SelfStabInvitation();
        SelfStabAccept();
        SelfStabConfirm();
        SelfStabCommit();
        SelfStabRelease();
        SelfStabCorrect();

        str << "\tInvitation: ";
        for (auto it : m_invitation) {
            if (it.second != SELF_STAB_NULL && (it.first == node_id || it.second == node_id))
                str << "\t[" << it.first << " -> " << it.second << "]";
        }
        str << "\n\tAcceptance: ";
        for (auto it : m_acceptance) {
            if (it.second != SELF_STAB_NULL && (it.first == node_id || it.second == node_id))
                str << "\t[" << it.first << " -> " << it.second << "]";
        }
        str << "\n\tCurrent: ";
        for (auto it : m_current) {
            //if (it.first == node_id || it.first == GetAcceptance(node_id) || it.first == GetInvitation(node_id))
            str << "\t[" << it.first << " -> " << it.second << "]";
        }
        str << "\n\tFuture: ";
        for (auto it : m_future) {
            //if (it.first == node_id || it.first == GetAcceptance(node_id) || it.first == GetInvitation(node_id))
            str << "\t[" << it.first << " -> " << it.second << "]";
        }
        NS_LOG_INFO("NODE " << node_id << " " << str.str());


        m_send_event = Simulator::Schedule(MilliSeconds(10), &SensorApp::SelfStabSendState, this);
        m_iter_event = Simulator::Schedule(MilliSeconds(50), &SensorApp::SelfStabIterate, this);

        m_nit++;
    }

    void SensorApp::ChangeState(int newstate) {
        Ptr<EnergySource> nrj_source = GetNode()->GetObject<EnergySourceContainer>()->Get(0);
        DeviceEnergyModelContainer c = nrj_source->FindDeviceEnergyModels(WifiRadioEnergyModel::GetTypeId());

        Ptr<DeviceEnergyModel> nrj_model = c.Get(0);

        nrj_model->ChangeState(newstate);
    }

    void SensorApp::PrintState() {
        uint32_t node_id = GetNode()->GetId();
        Ptr<EnergySource> nrj_source = GetNode()->GetObject<EnergySourceContainer>()->Get(0);
        //Ptr <BasicEnergySource> nrj_source = GetNode()->GetObject<BasicEnergySource>();
        //Ptr <WifiRadioEnergyModel> nrj_model = GetNode()->GetObject<WifiRadioEnergyModel>();

        NS_ASSERT(nrj_source != NULL);
        //NS_ASSERT (nrj_model != NULL);

        if (m_is_self_stab == false) {
            std::cerr << "TR\t" << node_id
                      << "\t" << Simulator::Now().GetMilliSeconds()
                      << "\t" << m_nit
                      << "\t" << m_sent
                      << std::setprecision(3)
                      << "\t" << m_z
                      << "\t" << m_x[node_id]
                      << "\t" << nrj_source->GetEnergyFraction()
                      << std::endl;
        } else {
            SelfStabIterate();
            std::cerr << "TR\t" << node_id
                      << "\t" << Simulator::Now().GetMilliSeconds()
                      << "\t" << m_nit
                      << "\t" << m_sent
                      << std::setprecision(3)
                      << "\t" << m_z
                      << "\t" << m_current[node_id]
                      << "\t" << nrj_source->GetEnergyFraction()
                      << std::endl;
            //m_iter_event = Simulator::Schedule(MilliSeconds(10), &SensorApp::SelfStabIterate, this);
        }
        ChangeState(WifiPhy::SLEEP);
        m_print_event = Simulator::Schedule(MilliSeconds(50), &SensorApp::PrintState, this);
    }

    void SensorApp::SelfStabSendState(void) {
        uint32_t i_id = GetNode()->GetId();
        //NS_LOG_INFO("NODE " << GetNode()->GetId() << " SEND STATE TO PEERS");
        std::stringstream addr_str;

        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        for (uint32_t iface = 0; iface < ipv4->GetNInterfaces(); iface++) {
            for (uint32_t addr_idx = 0; addr_idx < ipv4->GetNAddresses(iface);
                 addr_idx++) {
                Ipv4InterfaceAddress iaddr = ipv4->GetAddress(iface, addr_idx);
                Ipv4Address addr = iaddr.GetLocal();
                if (addr == Ipv4Address::GetLoopback())
                    continue;
                addr_str << "|" << addr;
            }
        }
        //NS_LOG_INFO("\tCurrent Identity " << addr_str.str());

        m_sensor_data->SetNodeId(GetNode()->GetId());

        // On envoie l'état courant à chacun des voisins connectés
        for (size_t v_idx = 0; v_idx < GetNumNeighbors(); ++v_idx) {
            Ipv4Address peerAddress = m_peer_addresses[v_idx];
            uint32_t j_id = m_ip_dic[peerAddress.Get()];

            // Pour chaque peer nous mettons à jour l'état courant du noeud
            if (m_broadcast == false) {
                auto s = m_to_update.find(j_id);

                if (j_id != GetInvitation(i_id) && j_id != GetAcceptance(i_id) && s == m_to_update.end())
                    continue;
            }
            if (!m_broadcast && !m_acceptance_updated && !m_invitation_updated)
                continue;

            ChangeState(WifiPhy::TX);

            m_sensor_data->SelfStabUpdate(SENSOR_INIT,
                                          GetInvitation(i_id),
                                          GetAcceptance(i_id),
                                          GetCurrent(i_id),
                                          GetFuture(i_id));

            // Initialisation du buffer d'envoi
            uint32_t bufsz = m_sensor_data->GetMarshalledSize();
            uint8_t *buf = new uint8_t[bufsz];
            m_sensor_data->MarshalTo(buf);
            Ptr<Packet> p = Create<Packet>(buf, bufsz);

            // Envoi du paquet réseau
            m_send_sockets[v_idx]->Send(p);

            delete[] buf;
            ++m_sent;

            NS_LOG_INFO("\tNODE " << i_id << " sending current state to "
                                  << j_id
                                  << "[" << peerAddress << "] "
                                  << m_sensor_data->ToString());
            //*/
        }

        m_broadcast = false;
        m_invitation_updated = false;
        m_acceptance_updated = false;
        m_to_update.clear();
    }

    TypeId SensorApp::GetTypeId(void) {
        static TypeId type_id = TypeId("ns3::SensorApp")
                .SetParent<Application>()
                .AddConstructor<SensorApp>()
                .AddAttribute("Port",
                              "The destination port",
                              UintegerValue(0),
                              MakeUintegerAccessor(&SensorApp::m_port),
                              MakeUintegerChecker<uint16_t>());
        return type_id;
    }

    void SensorApp::SetAsyncMode(bool is_async) {
        m_async = is_async;
    }

    void SensorApp::SetSensorData(SensorData *dd) {
        m_sensor_data = dd;
    }

    void SensorApp::SetIsSelfStab(bool is_self_stab) {
        m_is_self_stab = is_self_stab;
    }

    void SensorApp::SetInitialState(double value) {
        m_z = value;
    }

    void SensorApp::DoDispose(void) {
        Application::DoDispose();
    }
}
