#include "httplib.h"
#include "CSVReader.hpp"
#include "Graph.hpp"
#include <queue>
#include <tuple>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <unordered_map>

using json = nlohmann::json;

Graph fullGraph;
std::string defaultFuel = "Motorina";
double radius = 2000;

void setupGraph(double newRadius) {
    static bool initialized = false;
    static double lastRadius = -1;

    if (!initialized || newRadius != lastRadius) {
        fullGraph.stations.clear();
        std::vector<Station> loaded = readCSV("statii_carburanti.csv");
        for (const auto& s : loaded)
            fullGraph.addStation(s);

        double maxDistance = 3; // km
        fullGraph.adjList.clear();
        fullGraph.adjList.resize(fullGraph.stations.size());
        fullGraph.allEdges.clear();

        for (size_t i = 0; i < fullGraph.stations.size(); ++i) {
            std::vector<std::pair<int, double>> neighbors;
            for (size_t j = 0; j < fullGraph.stations.size(); ++j) {
                if (i == j) continue;
                double dist = fullGraph.haversine(
                    fullGraph.stations[i].latitude, fullGraph.stations[i].longitude,
                    fullGraph.stations[j].latitude, fullGraph.stations[j].longitude);
                if (dist <= maxDistance) {
                    neighbors.emplace_back(j, dist);
                }
            }
            std::sort(neighbors.begin(), neighbors.end(), [](auto& a, auto& b) {
                return a.second < b.second;
                });
            for (int k = 0; k < std::min(3, (int)neighbors.size()); ++k) {
                int j = neighbors[k].first;
                double dist = neighbors[k].second;
                fullGraph.adjList[i].push_back({ j, dist });
                fullGraph.adjList[j].push_back({ (int)i, dist });
                fullGraph.allEdges.emplace_back(i, j, dist);
            }
        }

        initialized = true;
        lastRadius = newRadius;
    }
}

json buildJsonResponse(const Station& s, const std::string& fuel, const std::string& type) {
    return {
        {"type", type},
        {"name", s.name},
        {"county", s.county},
        {"city", s.city},
        {"latitude", s.latitude},
        {"longitude", s.longitude},
        {"price", s.getFuelPrice(fuel)}
    };
}

Station getSmartStation(const Graph& graph, double lat, double lon, const std::string& fuelType) {
    int closestIndex = graph.findClosestStation(lat, lon);
    if (closestIndex == -1) return Station("Invalid", "", "", 0, 0, {});

    int bestIndex = -1;
    double bestPrice = std::numeric_limits<double>::infinity();
    std::vector<double> dist(graph.stations.size(), std::numeric_limits<double>::infinity());
    std::priority_queue<std::tuple<double, double, int>, std::vector<std::tuple<double, double, int>>, std::greater<>> pq;

    dist[closestIndex] = 0.0;
    pq.push({ 0.0, 0.0, closestIndex });

    while (!pq.empty()) {
        auto [d, price, u] = pq.top(); pq.pop();
        double p = graph.stations[u].getFuelPrice(fuelType);
        if (p > 0 && p < bestPrice) {
            bestPrice = p;
            bestIndex = u;
        }
        for (const auto& edge : graph.adjList[u]) {
            double newDist = d + edge.distance;
            if (newDist < dist[edge.to]) {
                dist[edge.to] = newDist;
                pq.push({ newDist, graph.stations[edge.to].getFuelPrice(fuelType), edge.to });
            }
        }
    }
    if (bestIndex == -1) return Station("Invalid", "", "", 0, 0, {});
    return graph.stations[bestIndex];
}

int findStationIndexByNameAndCoords(const Graph& graph, const std::string& name, double lat, double lon) {
    for (size_t i = 0; i < graph.stations.size(); ++i) {
        const auto& s = graph.stations[i];
        if (s.name == name && std::abs(s.latitude - lat) < 0.0001 && std::abs(s.longitude - lon) < 0.0001)
            return static_cast<int>(i);
    }
    return -1;
}

int main() {
    httplib::Server svr;
    svr.set_mount_point("/", ".");

    svr.Get("/update", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Content-Type", "application/json");

        if (req.has_param("lat") && req.has_param("lon")) {
            double userLat = std::stod(req.get_param_value("lat"));
            double userLon = std::stod(req.get_param_value("lon"));
            std::string fuel = req.has_param("fuel") ? req.get_param_value("fuel") : "";
            std::string mode = req.has_param("mode") ? req.get_param_value("mode") : "both";
            double newRadius = req.has_param("radius") ? std::stod(req.get_param_value("radius")) * 1000.0 : 50000;
            radius = newRadius;

            setupGraph(radius);
            Graph localGraph = fullGraph.buildSubgraphNear(userLat, userLon, radius);

            bool anyWithFuel = false;
            for (const auto& s : localGraph.stations) {
                if (s.getFuelPrice(fuel) > 0) {
                    anyWithFuel = true;
                    break;
                }
            }

            int closestWithFuel = -1;
            double minDist = 1e9;
            for (size_t i = 0; i < localGraph.stations.size(); ++i) {
                double d = localGraph.haversine(userLat, userLon, localGraph.stations[i].latitude, localGraph.stations[i].longitude);
                double price = localGraph.stations[i].getFuelPrice(fuel);
                if (price > 0 && d < minDist) {
                    minDist = d;
                    closestWithFuel = static_cast<int>(i);
                }
            }

            Station cheapest = getSmartStation(localGraph, userLat, userLon, fuel);

            json response;
            response["user"] = { {"lat", userLat}, {"lon", userLon} };

            if (!anyWithFuel) {
                response["error"] = "Statiile nu au acest tip de carburant";
                res.set_content(response.dump(), "application/json");
                return;
            }

            if (mode == "nearest" || mode == "both") {
                if (closestWithFuel != -1)
                    response["nearest"] = buildJsonResponse(localGraph.stations[closestWithFuel], fuel, "nearest");
            }
            if (mode == "cheapest" || mode == "both") {
                if (cheapest.name != "Invalid")
                    response["cheapest"] = buildJsonResponse(cheapest, fuel, "cheapest");
            }

            res.set_content(response.dump(), "application/json");
        }
        else {
            res.set_content("{\"error\": \"Missing lat/lon\"}", "application/json");
        }
        });

    svr.Get("/stations", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Content-Type", "application/json");

        if (req.has_param("lat") && req.has_param("lon")) {
            double userLat = std::stod(req.get_param_value("lat"));
            double userLon = std::stod(req.get_param_value("lon"));

            Graph localGraph = fullGraph.buildSubgraphNear(userLat, userLon, radius);
            json stationsJson = json::array();
            for (const auto& s : localGraph.stations) {
                stationsJson.push_back(buildJsonResponse(s, defaultFuel, "station"));
            }
            res.set_content(stationsJson.dump(), "application/json");
        }
        else {
            res.set_content("{\"error\": \"Missing lat/lon\"}", "application/json");
        }
        });

    svr.Get("/graph", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Content-Type", "application/json");

        if (req.has_param("lat") && req.has_param("lon")) {
            double userLat = std::stod(req.get_param_value("lat"));
            double userLon = std::stod(req.get_param_value("lon"));
            Graph localGraph = fullGraph.buildSubgraphNear(userLat, userLon, radius);

            json edgeArray = json::array();
            for (const auto& [from, to, dist] : localGraph.allEdges) {
                const auto& s1 = localGraph.stations[from];
                const auto& s2 = localGraph.stations[to];
                edgeArray.push_back({
                    {"from", {s1.latitude, s1.longitude}},
                    {"to", {s2.latitude, s2.longitude}},
                    {"distance", dist}
                    });
            }
            std::vector<std::vector<int>> comps = localGraph.getConnectedComponents();
            json componentsJson = json::array();

            for (const auto& group : comps) {
                json groupJson = json::array();
                for (int idx : group) {
                    const auto& s = localGraph.stations[idx];
                    groupJson.push_back({
                        {"name", s.name},
                        {"city", s.city},
                        {"latitude", s.latitude},
                        {"longitude", s.longitude},
                        {"price", s.getFuelPrice(defaultFuel)}
                        });
                }
                componentsJson.push_back(groupJson);
            }

            json response;
            response["edges"] = edgeArray;
            response["components"] = componentsJson;
            res.set_content(response.dump(), "application/json");

        }
        else {
            res.set_content("{\"error\": \"Missing lat/lon\"}", "application/json");
        }
        });

    svr.Get("/path", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Content-Type", "application/json");

        if (req.has_param("from") && req.has_param("to")) {
            auto parseCoord = [](const std::string& str) {
                size_t sep = str.find(',');
                return std::make_pair(std::stod(str.substr(0, sep)), std::stod(str.substr(sep + 1)));
                };

            auto [fromLat, fromLon] = parseCoord(req.get_param_value("from"));
            auto [toLat, toLon] = parseCoord(req.get_param_value("to"));

            Graph localGraph = fullGraph.buildSubgraphNear(fromLat, fromLon, radius);

            int start = localGraph.findClosestStation(fromLat, fromLon);
            int end = localGraph.findClosestStation(toLat, toLon);

            if (start == -1 || end == -1 ||
                start >= localGraph.stations.size() || end >= localGraph.stations.size()) {
                res.set_content("{\"error\": \"Statiile selectate nu au fost gasite in graf.\"}", "application/json");
                return;
            }

            std::vector<std::vector<int>> components = localGraph.getConnectedComponents();
            bool inSameComponent = false;
            for (const auto& comp : components) {
                bool foundStart = false, foundEnd = false;
                for (int idx : comp) {
                    if (idx == start) foundStart = true;
                    if (idx == end) foundEnd = true;
                }
                if (foundStart && foundEnd) {
                    inSameComponent = true;
                    break;
                }
            }

            if (!inSameComponent) {
                res.set_content("{\"error\": \"Statiile selectate nu sunt in aceeasi componenta conexa.\"}", "application/json");
                return;
            }

            std::vector<int> path = localGraph.dijkstraPath(start, end);

            auto distMatrix = localGraph.runFloydWarshall();
            if (start >= distMatrix.size() || end >= distMatrix[start].size()) {
                res.set_content("{\"error\": \"Eroare la accesarea distantei in matricea Floyd-Warshall.\"}", "application/json");
                return;
            }

            double totalDist = distMatrix[start][end];

            json response;
            response["distance"] = totalDist;
            response["path"] = json::array();

            for (int id : path) {
                if (id >= 0 && id < localGraph.stations.size()) {
                    const auto& s = localGraph.stations[id];
                    response["path"].push_back({ s.latitude, s.longitude });
                }
            }

            res.set_content(response.dump(), "application/json");
        }
        else {
            res.set_content("{\"error\": \"Missing 'from' or 'to' parameter\"}", "application/json");
        }
        });

    std::cout << "\n Server JSON pornit la http://localhost:5000\n";
    return svr.listen("localhost", 5000);
}