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

#ifndef ITERATIVEAVGCONSENSUS_SENSOR_DATA_H
#define ITERATIVEAVGCONSENSUS_SENSOR_DATA_H

#include "iterative_avg_consensus.h"

namespace ns3 {


    /**
     * Types de messages échangés entre les noeuds.
     * SENSOR_INIT : message de mise à jour d'état
     * SENSOR_RUN : message d'envoi des updates
     */
    enum SensorMsgType {
        SENSOR_INIT = 0,
        SENSOR_RUN = 1
    };


    /**
     * Cette classe gère les messages échangés entre les noeuds.
     * Basiquement trois données sont échangées :
     * - m_value qui correspond à la valeur
     * - m_type qui correspond au type de message
     * - m_node_id qui stocke le noeud qui a émit le message
     *
     * Dans le cas de l'algorithme self-stab nous ajoutons
     * - m_future correspond à la valeur future
     * - m_acceptance pointeur d'acceptation
     * - m_invitation pointeur d'invitation
     */
    class SensorData {
    public:

        SensorData();

        virtual ~SensorData();

        /**
         * Mise à jour des données du message
         */
        void Update(int tp, double value);

        void SelfStabUpdate(int tp, uint32_t invitation,
                            uint32_t acceptance, double value, double future);

        uint32_t GetMarshalledSize();

        /**
         * Sérialisation dans un buffer
         */
        void MarshalTo(uint8_t const *buf);

        /**
         * Désérialisation dans un buffer
         */
        void MarshalFrom(uint8_t const *buf);

        /**
         * Log de l'état courant
         */
        void LogMemory();

        /**
         * Sérialisation en string
         */
        std::string ToString();

        /**
         * Mise à jour de l'ID du noeud qui emet le message
         */
        void SetNodeId(uint32_t me);

        uint32_t GetNodeId() const;

        inline double GetValue() const { return m_value; }

        inline uint32_t GetType() const { return m_type; }

        inline uint32_t GetInvitation() const { return m_invitation; }

        inline uint32_t GetAcceptance() const { return m_acceptance; }

        inline double GetFuture() const { return m_future; }


    private:
        uint32_t m_node_id;
        uint32_t m_type;
        double m_value;

        // self stab values
        uint32_t m_invitation;
        uint32_t m_acceptance;
        double m_future;
    };

    /**
     * Classe d'instanciation de message SensorData
     */
    class SensorDataFactory {
    public:
        SensorData *Create() {
            return new SensorData();
        }
    };
}



#endif //ITERATIVEAVGCONSENSUS_SENSOR_DATA_H
