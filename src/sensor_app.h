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

#ifndef PROJECT_SENSOR_APP_H
#define PROJECT_SENSOR_APP_H

#include "iterative_avg_consensus.h"
#include "utils.h"
#include "sensor_data.h"

static std::map<uint32_t, double> glb_shared_values;

namespace ns3 {

    /**
     * SensorApp: main class for average consensus computing
     */
    class SensorApp : public Application {
    public:
        SensorApp();
        virtual ~SensorApp();

        static TypeId GetTypeId(void);
        void SetAsyncMode(bool is_async);
        void SetSensorData(SensorData *dd);
        void SetIsSelfStab(bool is_self_stab);
        void SetInitialState(double value);

    protected:
        virtual void DoDispose(void);

    private:
        /// Algorithm initialization
        virtual void StartApplication(void);
        virtual void StopApplication(void);


        /**
         * Network operators are inspired from the project dmc-sim project
         * https://github.com/Comcast/dmc-sim
         */

        /**
         * Returns true if this interface is the loopback interface
         */
        bool IsLoopback(uint32_t iface);
        /**
         * Returns the number of connected neighbors
         */
        uint32_t GetNumNeighbors();
        Ipv4Address GetNeighborAddress(uint32_t itf);
        Ptr <Node> GetNeighborNode(uint32_t itf);

        /**
         * Send the current state to node neighbors
         */
        void SendState(void);

        /**
         * Send updates to neighbors
         */
        void SendUpdate(void);

        /**
         * Current state update
         */
        void UpdateState();

        /**
         * Message reception
         */
        void HandleMessage(Ptr<Socket> socket);

        /**
         * Compute the difference ∆ji (t) between node i and its neighbor j
         */
        double node_delta(double v);

        /**
         * ηi(t) is the number of neighbors of node i at time t
         */
        double node_eta() {
            return (double) GetNumNeighbors();
        }

        /**
         * Diffusion algorithm iterate function
         */
        void Iterate();

        /**
         * Self-stabilizing algorithm, current status functions
         */
        inline uint32_t GetInvitation(uint32_t id);
        inline uint32_t GetAcceptance(uint32_t id);
        inline double GetCurrent(uint32_t id);
        inline double GetFuture(uint32_t id);

        /**
         * Self-stab invitation
         * (i.I = null ∧ i.A = null ∧ ∃j ∈ N(i) : j.x − i.x > ε) ⇒ (i.I = j)
         */
        void SelfStabInvitation();

        /**
         * Accept invitation and begin transaction
         */
        void SelfStabAccept();

        /**
         * Confirm transaction
         */
        void SelfStabConfirm();

        /**
         * Commit transaction
         */
        void SelfStabCommit();

        /**
         * Release pointers
         */
        void SelfStabRelease();

        /**
         * Correct pointers
         */
        bool SelfStabCorrect();

        /**
         * Self-stab consensus algorithm iteration
         */
        void SelfStabIterate();

        /// Send the current node state
        void SelfStabSendState(void);

        /// Update the current state for energy model
        void ChangeState(int newstate);

        /// Print the current simulation state
        void PrintState();

        /// Network attributes
        uint32_t m_sent;
        uint16_t m_port;
        uint32_t m_num_peers;
        Ipv4Address *m_peer_addresses;
        Ptr <Node> *m_peer_nodes;
        Ptr <Socket> *m_send_sockets;
        Ptr <Socket> m_recv_socket;

        /// Event objects
        EventId m_send_event;
        EventId m_iter_event;
        EventId m_update_event;
        EventId m_print_event;

        /// Storing message data
        SensorData *m_sensor_data;

        double m_z; /// node initial state
        std::map<uint32_t, double> m_x; /// state list
        std::map<uint32_t, double> m_s; /// sent scalars
        std::map<uint32_t, double> m_r; /// received scalars

        std::map<uint32_t, uint32_t> m_ip_dic; /// Ip address dictionary

        /// Self-stabilization attributes
        std::map<uint32_t, uint32_t> m_invitation; // Invitation pointer
        std::map<uint32_t, uint32_t> m_acceptance; // Acceptance pointer
        std::map<uint32_t, double> m_current; // Current values
        std::map<uint32_t, double> m_future; // Future values
        double m_epsilon;
        bool m_is_self_stab;

        /// Self-stabilization status variables
        bool m_broadcast;
        bool m_invitation_updated;
        bool m_acceptance_updated;
        std::set<uint32_t> m_to_update;
        int m_sstab_step;
        bool is_move;

        uint32_t m_nit;
        uint32_t m_nmove, m_prv_nmove;
        bool m_async;
        bool m_update_state;

        static std::map<size_t, size_t> m_iterations_move;
    };


    class SensorAppHelper {
    public:
        SensorAppHelper(uint16_t port, SensorDataFactory *factory, uint32_t use_mk = 0,
                        uint32_t self_stab = 0, uint32_t is_async = 0) {
            m_factory.SetTypeId(SensorApp::GetTypeId());
            SetAttribute("Port", UintegerValue(port));
            m_sensor_factory = factory;
            m_use_mk = use_mk;
            m_is_self_stab = self_stab == 0 ? false : true;
            m_is_async = is_async == 0 ? false : true;
        }

        void SetAttribute(std::string name, const AttributeValue &value) {
            m_factory.Set(name, value);
        }

        ApplicationContainer Install(Ptr <Node> node) const {
            return ApplicationContainer(InstallPriv(node));
        }

        ApplicationContainer Install(std::string nodeName) const {
            Ptr <Node> node = Names::Find<Node>(nodeName);
            return ApplicationContainer(InstallPriv(node));
        }

        ApplicationContainer Install(NodeContainer nodes) const {
            ApplicationContainer out;
            std::vector<double> initial_states;
            double sum = 0;

            if (m_use_mk) {
                initial_states.push_back(0.7);
                initial_states.push_back(0.5);
                initial_states.push_back(0.2);
                initial_states.push_back(0.9);
                sum = 1;
            } else {
                for (uint32_t i = 0; i < nodes.GetN(); i++) {
                    initial_states.push_back(generate_random_sensor_data());
                    sum += initial_states.back();
                }
            }
            for (uint32_t i = 0; i < nodes.GetN(); i++) {
                if (m_use_mk)
                    out.Add(InstallPriv(nodes.Get(i), initial_states[i]));
                else
                    out.Add(InstallPriv(nodes.Get(i), initial_states[i]));// 5 * nodes.GetN() * (initial_states[i] / sum)));
            }

            return out;
        }

    private:
        Ptr <Application> InstallPriv(Ptr <Node> node, double value = 0) const {
            Ptr <SensorApp> sensor_app = (Ptr <SensorApp>) m_factory.Create<SensorApp>();
            sensor_app->SetSensorData(m_sensor_factory->Create());
            sensor_app->SetInitialState(value);
            sensor_app->SetIsSelfStab(m_is_self_stab);
            sensor_app->SetAsyncMode(m_is_async);
            Ptr <Application> app = (Ptr <Application>) sensor_app;

            node->AddApplication(app);

            return app;
        }

        ObjectFactory m_factory; //!< Object factory.
        SensorDataFactory *m_sensor_factory;
        uint32_t m_use_mk;
        bool m_is_self_stab;
        bool m_is_async;
    };
}

#endif //PROJECT_SENSOR_APP_H
