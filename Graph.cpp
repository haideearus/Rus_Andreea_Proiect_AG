#include "Graph.hpp"
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <numbers>
#include <unordered_map>

constexpr double EARTH_RADIUS_KM = 6378.0;

double Graph::haversine(double lat1, double lon1, double lat2, double lon2) const {
    double toRad = std::numbers::pi / 180.0;
    double dLat = (lat2 - lat1) * toRad;
    double dLon = (lon2 - lon1) * toRad;
    lat1 *= toRad;
    lat2 *= toRad;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
        std::cos(lat1) * std::cos(lat2) *
        std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return EARTH_RADIUS_KM * c;
}

void Graph::addStation(const Station& station) {
    stations.push_back(station);
}

void Graph::buildGraph(double maxDistanceMeters) {
    double maxKm = maxDistanceMeters / 1000.0;
    adjList.clear();
    allEdges.clear();
    adjList.resize(stations.size());

    for (size_t i = 0; i < stations.size(); ++i) {
        for (size_t j = i + 1; j < stations.size(); ++j) {
            double dist = haversine(
                stations[i].latitude, stations[i].longitude,
                stations[j].latitude, stations[j].longitude);
            if (dist <= maxKm) {
                adjList[i].push_back({ (int)j, dist });
                adjList[j].push_back({ (int)i, dist });
                allEdges.emplace_back(i, j, dist);
            }
        }
    }
}

Graph Graph::buildSubgraphNear(double centerLat, double centerLon, double maxDistanceMeters) const {
    Graph subgraph;
    double maxKm = maxDistanceMeters / 1000.0;
    std::vector<int> indices;

    for (size_t i = 0; i < stations.size(); ++i) {
        if (haversine(centerLat, centerLon, stations[i].latitude, stations[i].longitude) <= maxKm) {
            indices.push_back(i);
            subgraph.stations.push_back(stations[i]);
        }
    }

    std::unordered_map<int, int> indexMap;
    for (size_t newIdx = 0; newIdx < indices.size(); ++newIdx) {
        indexMap[indices[newIdx]] = newIdx;
    }

    for (const auto& [i, j, dist] : allEdges) {
        if (indexMap.count(i) && indexMap.count(j)) {
            int newI = indexMap[i];
            int newJ = indexMap[j];
            subgraph.adjList.resize(subgraph.stations.size());
            subgraph.adjList[newI].push_back({ newJ, dist });
            subgraph.adjList[newJ].push_back({ newI, dist });
            subgraph.allEdges.emplace_back(newI, newJ, dist);
        }
    }
    return subgraph;
}

int Graph::findClosestStation(double lat, double lon) const {
    int best = -1;
    double minDist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < stations.size(); ++i) {
        double d = haversine(lat, lon, stations[i].latitude, stations[i].longitude);
        if (d < minDist) {
            minDist = d;
            best = (int)i;
        }
    }
    return best;
}

std::vector<int> Graph::findNearestStations(double lat, double lon, double maxDistanceKm) const {
    std::vector<int> result;
    for (size_t i = 0; i < stations.size(); ++i) {
        if (haversine(lat, lon, stations[i].latitude, stations[i].longitude) <= maxDistanceKm)
            result.push_back(i);
    }
    return result;
}

std::string Graph::formatStationDetails(int index, const std::string& fuelType) const {
    if (index < 0 || index >= (int)stations.size()) return "Invalid station index";
    const Station& s = stations[index];
    std::ostringstream out;
    out << "Name: " << s.name << "\nCounty: " << s.county << "\nCity: " << s.city;
    out << "\nCoords: (" << s.latitude << ", " << s.longitude << ")\n";
    out << "Price " << fuelType << ": " << s.getFuelPrice(fuelType) << " RON/L\n";
    return out.str();
}

std::vector<std::vector<int>> Graph::getConnectedComponents() const {
    std::vector<std::vector<int>> components;
    std::vector<bool> visited(stations.size(), false);

    for (size_t i = 0; i < stations.size(); ++i) {
        if (!visited[i]) {
            std::vector<int> component;
            std::queue<int> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty()) {
                int current = q.front(); q.pop();
                component.push_back(current);
                for (const auto& neighbor : adjList[current]) {
                    if (!visited[neighbor.to]) {
                        visited[neighbor.to] = true;
                        q.push(neighbor.to);
                    }
                }
            }

            components.push_back(component);
        }
    }

    return components;
}

std::vector<int> Graph::dijkstraPath(int start, int end) const {
    const int N = stations.size();
    std::vector<double> dist(N, std::numeric_limits<double>::infinity());
    std::vector<int> prev(N, -1);
    dist[start] = 0.0;

    using Pair = std::pair<double, int>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<>> pq;
    pq.push({ 0.0, start });

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (u == end) break;

        for (const auto& edge : adjList[u]) {
            if (dist[edge.to] > dist[u] + edge.distance) {
                dist[edge.to] = dist[u] + edge.distance;
                prev[edge.to] = u;
                pq.push({ dist[edge.to], edge.to });
            }
        }
    }

    std::vector<int> path;
    for (int at = end; at != -1; at = prev[at])
        path.push_back(at);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::vector<double>> Graph::runFloydWarshall() const {
    int N = stations.size();
    std::vector<std::vector<double>> dist(N, std::vector<double>(N, std::numeric_limits<double>::infinity()));

    for (int i = 0; i < N; ++i) dist[i][i] = 0.0;
    for (int i = 0; i < N; ++i) {
        for (const auto& edge : adjList[i]) {
            dist[i][edge.to] = edge.distance;
        }
    }

    for (int k = 0; k < N; ++k)
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (dist[i][k] + dist[k][j] < dist[i][j])
                    dist[i][j] = dist[i][k] + dist[k][j];

    return dist;
}
