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

#ifndef PROJECT_ITERATIVE_AVG_CONSENSUS_H
#define PROJECT_ITERATIVE_AVG_CONSENSUS_H

#include <numeric>
#include <iomanip>
#include <fstream>


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"

#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"

#define SELF_STAB_NULL 4242
#define SELF_STAB_EPSILON 0.1

extern ns3::LogComponent g_log;

#endif //PROJECT_ITERATIVE_AVG_CONSENSUS_H
