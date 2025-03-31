#pragma once
#include <string>
#include <unordered_map>

class Station {
public:
    std::string name, county, city;
    double latitude, longitude;
    std::unordered_map<std::string, double> fuelPrices;

    Station(std::string n, std::string c, std::string ci, double lat, double lon, std::unordered_map<std::string, double> prices);
    double getFuelPrice(const std::string& fuelType) const;
};