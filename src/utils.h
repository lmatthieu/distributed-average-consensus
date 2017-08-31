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

#ifndef PROJECT_UTILS_H
#define PROJECT_UTILS_H

#include <map>
#include <vector>
#include <algorithm>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

using namespace boost;
using namespace std;

typedef adjacency_list<vecS, vecS, undirectedS> Graph;

template<typename T, typename Compare>
inline std::vector<std::size_t> sort_permutation(
        const std::vector<T> &vec,
        Compare compare) {
    std::vector<std::size_t> p(vec.size());
    std::iota(p.begin(), p.end(), 0);
    std::sort(p.begin(), p.end(),
              [&](std::size_t i, std::size_t j) { return compare(vec[i], vec[j]); });
    return p;
}

template<typename T>
inline std::vector<T> apply_permutation(
        const std::vector<T> &vec,
        const std::vector<std::size_t> &p) {
    std::vector<T> sorted_vec(vec.size());
    std::transform(p.begin(), p.end(), sorted_vec.begin(),
                   [&](std::size_t i) { return vec[i]; });
    return sorted_vec;
}

/**
 * This function extracts the connected components of a Graph G
 * @param G the boost graph
 * @param num_cmp output the number of connected components
 * @param nnodes the number of vertices of G
 * @return A component vector indicating the component id of each vertice
 */
inline std::vector<int> get_connected_components(Graph &G, int *num_cmp, int nnodes) {
    std::vector<int> component(nnodes);
    *num_cmp = connected_components(G, &component[0]);

    return component;
}

/**
 * Generates a random measure in the range [low, high]
 */
inline double generate_random_sensor_data(double low = 0.0, double high = 1.0) {
    return low + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (high - low)));
}


#endif //PROJECT_UTILS_H
