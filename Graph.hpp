#pragma once
#include "Station.hpp"
#include <vector>
#include <tuple>

struct Edge {
    int to;
    double distance;
};

class Graph {
public:
    std::vector<Station> stations;
    std::vector<std::vector<Edge>> adjList;
    std::vector<std::tuple<int, int, double>> allEdges;

    void addStation(const Station& station);
    void buildGraph(double maxDistanceMeters);
    Graph buildSubgraphNear(double centerLat, double centerLon, double maxDistanceMeters) const;
    double haversine(double lat1, double lon1, double lat2, double lon2) const;

    int findClosestStation(double lat, double lon) const;
    std::vector<int> findNearestStations(double lat, double lon, double maxDistanceKm) const;
    std::string formatStationDetails(int index, const std::string& fuelType) const;
    std::vector<std::vector<int>> getConnectedComponents() const;
    std::vector<int> dijkstraPath(int start, int end) const;
    std::vector<std::vector<double>> runFloydWarshall() const;
};
