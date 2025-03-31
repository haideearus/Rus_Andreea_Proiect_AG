#include "CSVReader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

std::string cleanString(std::string value) {
    value.erase(0, value.find_first_not_of(" \""));
    value.erase(value.find_last_not_of(" \"") + 1);
    return value;
}

void extractCounty(std::string& fullName, std::string& county) {
    size_t dashPos = fullName.find(" - ");
    if (dashPos != std::string::npos) {
        county = fullName.substr(dashPos + 3);
        fullName = fullName.substr(0, dashPos);
    }
    else {
        county = "Necunoscut";
    }
}

bool isValidNumber(const std::string& str) {
    if (str.empty()) return false;
    try {
        std::stod(str);
        return true;
    }
    catch (...) {
        return false;
    }
}

std::vector<Station> readCSV(const std::string& filename) {
    std::vector<Station> stations;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Nu s-a putut deschide fisierul " << filename << std::endl;
        return stations;
    }

    std::cout << "Fisier deschis cu succes!\n";

    std::getline(file, line);
    std::stringstream headerStream(line);
    std::vector<std::string> fuelTypes;
    std::string column;

    for (int i = 0; i < 5; ++i) {
        std::getline(headerStream, column, ',');
    }

    while (std::getline(headerStream, column, ',')) {
        fuelTypes.push_back(cleanString(column));
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;

        std::string name, county, city;
        std::getline(ss, name, ',');
        name = cleanString(name);

        extractCounty(name, county);

        std::getline(ss, city, ',');
        city = cleanString(city);

        std::string latStr, lonStr;
        std::getline(ss, latStr, ',');
        latStr = cleanString(latStr);

        std::getline(ss, lonStr, ',');
        lonStr = cleanString(lonStr);

        if (!isValidNumber(latStr) || !isValidNumber(lonStr)) {
            continue;
        }

        double latitude = std::stod(latStr);
        double longitude = std::stod(lonStr);

        std::unordered_map<std::string, double> fuelPrices;
        int index = 0;
        while (std::getline(ss, value, ',')) {
            value = cleanString(value);
            if (index < fuelTypes.size()) {
                double price = (value == "N/A" || value == "-" || value.empty()) ? -1.0 : std::stod(value);
                fuelPrices[fuelTypes[index]] = price;
            }
            index++;
        }

        stations.emplace_back(name, county, city, latitude, longitude, fuelPrices);
    }

    file.close();
    return stations;
}