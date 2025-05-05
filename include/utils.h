#ifndef UTILS_H
#define UTILS_H

#include <SDL3/SDL.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>

SDL_Color interpolateColor(const SDL_Color& a, const SDL_Color& b, float t) {
    SDL_Color result;
    result.r = (Uint8)(a.r + t * (b.r - a.r));
    result.g = (Uint8)(a.g + t * (b.g - a.g));
    result.b = (Uint8)(a.b + t * (b.b - a.b));
    result.a = 255;
    return result;
}

SDL_Color getSpeedColor(float speed, float maxSpeed) {
    float t = fminf(1.0f, speed / maxSpeed);  // Clamp 0–1

    SDL_Color c0 = {0, 0, 255, 255};      // Deep Blue
    SDL_Color c1 = {0, 255, 255, 255};    // Cyan
    SDL_Color c2 = {0, 255, 0, 255};      // Green
    SDL_Color c3 = {255, 255, 0, 255};    // Yellow
    SDL_Color c4 = {255, 0, 0, 255};      // Red

    if (t < 0.25f) return interpolateColor(c0, c1, t / 0.25f);
    else if (t < 0.5f) return interpolateColor(c1, c2, (t - 0.25f) / 0.25f);
    else if (t < 0.75f) return interpolateColor(c2, c3, (t - 0.5f) / 0.25f);
    else return interpolateColor(c3, c4, (t - 0.75f) / 0.25f);
}

void writeToFile(std::string filename, uint64_t frames, float runtime, const std::vector<float>& data) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "{\n\t\"frame_count\": " << frames << ",\n";
        file << "\t\"runtime\": " << runtime << ",\n";
        file << "\t\"fps\":[\n";
        for (size_t i = 0; i < data.size(); ++i) {
            file << " \t\t" << data[i];
            if (i < data.size() - 1) {
                file << ",";
            }
            file << "\n";
        }
        file << "\t]\n}";
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

#endif // UTILS_H