#include "Station.hpp"

Station::Station(std::string n, std::string c, std::string ci, double lat, double lon, std::unordered_map<std::string, double> prices)
    : name(n), county(c), city(ci), latitude(lat), longitude(lon), fuelPrices(prices) {}

double Station::getFuelPrice(const std::string& fuelType) const {
    auto it = fuelPrices.find(fuelType);
    return (it != fuelPrices.end()) ? it->second : -1.0;
}
