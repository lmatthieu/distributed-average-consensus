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

#include "sensor_data.h"

namespace ns3 {

    SensorData::SensorData() {
        m_value = 0;
        m_type = -1;

        // self-stab
        m_future = 0;
        m_acceptance = 0;
        m_invitation = 0;
    }

    SensorData::~SensorData() {
    }

    void SensorData::Update(int tp, double value) {
        m_type = tp;
        m_value = value;
    }

    void SensorData::SelfStabUpdate(int tp, uint32_t invitation,
                                    uint32_t acceptance, double value, double future) {
        m_type = tp;
        m_invitation = invitation;
        m_acceptance = acceptance;
        m_value = value;
        m_future = future;
    }

    uint32_t SensorData::GetMarshalledSize() {
        return 2 * sizeof(double) + 4 * sizeof(uint32_t);
    }

    void SensorData::MarshalTo(uint8_t const *buf) {
        uint8_t *dst = (uint8_t *) buf;
        memcpy(dst, &m_node_id, sizeof(uint32_t));
        dst += sizeof(uint32_t);
        memcpy(dst, &m_type, sizeof(uint32_t));
        dst += sizeof(uint32_t);
        memcpy(dst, &m_value, sizeof(double));
        dst += sizeof(double);

        // self-stab
        memcpy(dst, &m_future, sizeof(double));
        dst += sizeof(double);
        memcpy(dst, &m_invitation, sizeof(uint32_t));
        dst += sizeof(uint32_t);
        memcpy(dst, &m_acceptance, sizeof(uint32_t));
        dst += sizeof(uint32_t);
    }

    void SensorData::MarshalFrom(uint8_t const *buf) {
        uint8_t *src = (uint8_t *) buf;
        memcpy(&m_node_id, src, sizeof(uint32_t));
        src += sizeof(uint32_t);
        memcpy(&m_type, src, sizeof(uint32_t));
        src += sizeof(uint32_t);
        memcpy(&m_value, src, sizeof(m_value));
        src += sizeof(double);

        //self-stab
        memcpy(&m_future, src, sizeof(double));
        src += sizeof(double);
        memcpy(&m_invitation, src, sizeof(uint32_t));
        src += sizeof(uint32_t);
        memcpy(&m_acceptance, src, sizeof(uint32_t));
        src += sizeof(uint32_t);
    }

    void SensorData::LogMemory() {
        NS_LOG_INFO("memory: msg_sz=" << GetMarshalledSize() <<
                                      " m_node_id=" << m_node_id <<
                                      " m_type=" << m_type <<
                                      " m_value=" << m_value <<
                                      " m_future=" << m_future <<
                                      " m_invitation=" << m_invitation <<
                                      " m_acceptance=" << m_acceptance);
    }

    std::string SensorData::ToString() {
        std::stringstream stream;

        stream << "[" << this << "]" <<
               "memory: msg_sz=" << GetMarshalledSize() <<
               " m_node_id=" << m_node_id <<
               " m_type=" << m_type <<
               " m_value=" << m_value <<
               " m_future=" << m_future <<
               " m_invitation=" << m_invitation <<
               " m_acceptance=" << m_acceptance;
        return stream.str();
    }

    void SensorData::SetNodeId(uint32_t me) {
        m_node_id = me;
    }

    uint32_t SensorData::GetNodeId() const {
        return m_node_id;
    }
}

