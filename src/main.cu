#include <SDL3/SDL.h>
#include <curand_kernel.h>
#include <vector>
#include <iostream>
#include "vector.h"
#include "boid.h"
#include "utils.h"

using namespace std;

#define NUM_BOIDS 7000
#define MAX_SPEED 300.0f
#define NEIGHBOUR_RANGE 35.0f
#define SEPARATION_THRESHOLD 20.0f
#define TARGET_ATTRACTION_STRENGTH 0.6f
#define EXPLODE_STRENGTH 1000000.0f
#define BLOCK_SIZE 256
#define MAX_NEIGHBOURS 128

__global__ void initBoidsKernel(Boid* boids, int screenWidth, int screenHeight, curandState* states) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= NUM_BOIDS) return;
    
    curandState localState = states[idx];
    
    float posX = curand_uniform(&localState) * screenWidth;
    float posY = curand_uniform(&localState) * screenHeight;
    
    float velX = (curand_uniform(&localState) * 2.0f - 1.0f) * MAX_SPEED;
    float velY = (curand_uniform(&localState) * 2.0f - 1.0f) * MAX_SPEED;
    
    boids[idx] = Boid(Vector2(posX, posY), Vector2(velX, velY));
    boids[idx].color = getSpeedColor(boids[idx].velocity.magnitude(), MAX_SPEED);
    
    states[idx] = localState;
}

void initBoidsGPU(std::vector<Boid>& boids, int screenWidth, int screenHeight, curandState* d_states) {
    
    Boid* d_boids;
    cudaMalloc(&d_boids, NUM_BOIDS * sizeof(Boid));
    
    int blockSize = BLOCK_SIZE;
    int gridSize = (NUM_BOIDS + blockSize - 1) / blockSize;
    
    initBoidsKernel<<<gridSize, blockSize>>>(d_boids, screenWidth, screenHeight, d_states);
    cudaDeviceSynchronize();

    cudaMemcpy(boids.data(), d_boids, NUM_BOIDS * sizeof(Boid), cudaMemcpyDeviceToHost);

    cudaFree(d_boids);
}

__global__ void setupKernel(curandState* states, unsigned long seed) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < NUM_BOIDS) {
        curand_init(seed, idx, 0, &states[idx]);
    }
}

__device__ int getNeighbours(Boid* boids, int self_idx, Boid* neighbours, float range) {
    int count = 0;
    const Boid& self = boids[self_idx];
    
    for (int i = 0; i < NUM_BOIDS && count < MAX_NEIGHBOURS; i++) {
        if (i == self_idx) continue;
        
        float dx = self.position.x - boids[i].position.x;
        float dy = self.position.y - boids[i].position.y;
        float distSquared = dx*dx + dy*dy;
        
        if (distSquared < range * range) {
            neighbours[count++] = boids[i];
        }
    }
    return count;
}

__device__ Vector2 calculateAlignment(Boid* boid, Boid* neighbours, int neighbour_count, float max_speed){
    Vector2 averageSpeed(0,0);
    for (int i = 0; i < neighbour_count; i++){
        averageSpeed += neighbours[i].velocity;
    }

    if(neighbour_count <= 0 || averageSpeed.magnitude() <= 0){
        return Vector2(0,0);
    }

    averageSpeed /= neighbour_count;
    return (averageSpeed - boid->velocity).normalize() * max_speed;
}

__device__ Vector2 calculateCohesion(Boid* boid, Boid* neighbours, int neighbour_count, float max_speed){
    Vector2 centerOfMass(0,0);
    for (int i = 0; i < neighbour_count; i++){
        centerOfMass += neighbours[i].position;
    }

    if(neighbour_count <= 0 || centerOfMass.magnitude() <= 0){
        return Vector2(0,0);
    }

    centerOfMass /= neighbour_count;
    return (centerOfMass - boid->position).normalize() * max_speed;
}

__device__ Vector2 calculateSeparation(Boid* boid, Boid* neighbours, int neighbour_count, float separationThreshold, float max_speed){
    Vector2 separationForce(0,0);
    
    float densityFactor = fminf(1.0f, 50.0f / (float)neighbour_count);
    float adjustedThreshold = separationThreshold * (1.0f + (1.0f - densityFactor) * 2.0f);
    
    for (int i = 0; i < neighbour_count; i++){
        float distance = boid->distance(neighbours[i]);
        if(distance < adjustedThreshold && distance > 0.0001f){
            float repulsionStrength = 1.0f / (distance * distance);
            separationForce += (boid->position - neighbours[i].position) * repulsionStrength;
        }
    }

    if(separationForce.magnitude() <= 0){
        return Vector2(0,0);
    }

    float priorityFactor = 1.0f + fminf(2.0f, (float)neighbour_count / 20.0f);
    return separationForce.normalize() * max_speed * priorityFactor;
}

__device__ Vector2 calculateAttraction(Boid* boid, const Vector2* target, float max_speed){
    if(target == nullptr){
        return Vector2(0,0);
    }

    Vector2 direction = *target - boid->position;
    
    if (direction.magnitude() < 5.0f) {
        return Vector2(0, 0);
    }

    return direction.normalize() * max_speed * TARGET_ATTRACTION_STRENGTH;
}

__device__ Vector2 calculateExplosion(Boid* boid, Boid* neighbours, int neighbour_count) {
    if (neighbour_count == 0) {
        return Vector2(0, 0);
    }
    
    Vector2 center(0, 0);
    for (int i = 0; i < neighbour_count; i++) {
        center += neighbours[i].position;
    }
    center += boid->position;
    center /= (neighbour_count + 1);
    
    Vector2 direction = boid->position - center;
    
    if (direction.magnitude() < 0.0001f) { // velocities can be reaaally small
        return Vector2(1.0f, 0.0f);
    }
    
    return direction.normalize() * EXPLODE_STRENGTH;
}

__global__ void SimulateBoidKernel(Boid* boids, Boid* updated_boids,
    const Vector2* target, int screenWidth, int screenHeight, float dt, bool explode, curandState* states)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= NUM_BOIDS) return;

    curandState localState = states[idx];
    Boid* boid = &boids[idx];
    
    Boid neighbour_list[MAX_NEIGHBOURS];  
    int neighbour_count = getNeighbours(boids, idx, neighbour_list, NEIGHBOUR_RANGE);
    
    if (neighbour_count > MAX_NEIGHBOURS) neighbour_count = MAX_NEIGHBOURS;

    float crowdFactor = fminf(1.0f, 40.0f / (float)(neighbour_count + 1));
    
    Vector2 alignment = calculateAlignment(boid, neighbour_list, neighbour_count, MAX_SPEED) * crowdFactor;
    Vector2 cohesion = calculateCohesion(boid, neighbour_list, neighbour_count, MAX_SPEED) * crowdFactor;
    Vector2 separation = calculateSeparation(boid, neighbour_list, neighbour_count, SEPARATION_THRESHOLD, MAX_SPEED);
    Vector2 attraction = calculateAttraction(boid, target, MAX_SPEED) * crowdFactor;
    
    Vector2 totalForce = alignment + cohesion + separation * (2.0f - crowdFactor) + attraction;

    if (explode) {
        Vector2 explosionForce = calculateExplosion(boid, neighbour_list, neighbour_count);
        Vector2 randomDirection(
            curand_uniform(&localState) * 2.0f - 1.0f,
            curand_uniform(&localState) * 2.0f - 1.0f
        );
        totalForce += explosionForce + randomDirection.normalize() * EXPLODE_STRENGTH;
    }

    states[idx] = localState;

    Vector2 new_velocity = boid->velocity + (totalForce * dt);

    if(new_velocity.magnitude() > MAX_SPEED){
        new_velocity = new_velocity.normalize() * MAX_SPEED;
    }

    Vector2 new_position = boid->position + (new_velocity * dt);
    
    if (new_position.x < 0) new_position.x = screenWidth;
    if (new_position.x > screenWidth) new_position.x = 0;
    if (new_position.y < 0) new_position.y = screenHeight;
    if (new_position.y > screenHeight) new_position.y = 0;

    updated_boids[idx] = Boid(new_position, new_velocity);
    updated_boids[idx].color = getSpeedColor(new_velocity.magnitude(), MAX_SPEED);
}

void SimulateBoids(std::vector<Boid>& boids, std::vector<Boid>& new_boids,
                   const Vector2* target, int screenWidth, int screenHeight, float dt, bool& explode, curandState* d_states) 
{
    Boid* d_boids, *d_new_boids;
    Vector2* d_target = nullptr;
    bool* d_explode;

    cudaMalloc(&d_boids, NUM_BOIDS * sizeof(Boid));
    cudaMalloc(&d_new_boids, NUM_BOIDS * sizeof(Boid));
    cudaMalloc(&d_explode, sizeof(bool));
    cudaMemcpy(d_boids, boids.data(), NUM_BOIDS * sizeof(Boid), cudaMemcpyHostToDevice);
    cudaMemcpy(d_explode, &explode, sizeof(bool), cudaMemcpyHostToDevice);

    if(target != nullptr) {
        cudaMalloc(&d_target, sizeof(Vector2));
        cudaMemcpy(d_target, target, sizeof(Vector2), cudaMemcpyHostToDevice);
    }

    int blockSize = BLOCK_SIZE;
    int gridSize = (NUM_BOIDS + blockSize - 1) / blockSize;
    
    SimulateBoidKernel<<<gridSize, blockSize>>>(d_boids, d_new_boids, d_target, screenWidth, screenHeight, dt, d_explode, d_states);
    cudaDeviceSynchronize();
    
    cudaMemcpy(new_boids.data(), d_new_boids, NUM_BOIDS * sizeof(Boid), cudaMemcpyDeviceToHost);

    cudaFree(d_boids);
    cudaFree(d_new_boids);
    cudaFree(d_explode);
    if(d_target != nullptr) {
        cudaFree(d_target);
    }

    explode = false;
}

void RenderScreen(SDL_Renderer* renderer, std::vector<Boid>& boids, std::vector<Boid>& new_boids, Vector2* target) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    for (size_t i = 0; i < new_boids.size(); i++) {
        boids[i] = new_boids[i];
        Vector2 pos = new_boids[i].position;
        SDL_Color color = new_boids[i].color;

        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

        const int radius = 2;
        for (int x = -radius; x <= radius; x++) {
            for (int y = -radius; y <= radius; y++) {
                if (x * x + y * y <= radius * radius) {
                    SDL_RenderPoint(renderer, (int)pos.x + x, (int)pos.y + y);
                }
            }
        }
    }

    if (target != nullptr) {
        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        const int targetRadius = 8;
        for (int x = -targetRadius; x <= targetRadius; x++) {
            for (int y = -targetRadius; y <= targetRadius; y++) {
                if (x * x + y * y <= targetRadius * targetRadius) {
                    SDL_RenderPoint(renderer, (int)target->x + x, (int)target->y + y);
                }
            }
        }
    }

    SDL_RenderPresent(renderer);
}


int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Boids Simulation", 1920, 1080, SDL_WINDOW_FULLSCREEN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, "software");

    int screenWidth, screenHeight;
    SDL_GetWindowSize(window, &screenWidth, &screenHeight);
    cout << "Screen dimensions: " << screenWidth << "x" << screenHeight << endl;

    std::vector<Boid> boids(NUM_BOIDS);
    std::vector<Boid> new_boids(NUM_BOIDS);

    int frameCount = 0;
    float fpsTimer = 0.0f;
    float fps = 0.0f;
    uint64_t totalFrames = 0;
    std::vector<float> fpsValues;

    bool running = true;
    bool explode = false;
    SDL_Event event;
    Vector2* target = nullptr;
    curandState* d_states;

    cudaMalloc(&d_states, sizeof(curandState) * NUM_BOIDS);

    setupKernel<<<(NUM_BOIDS + BLOCK_SIZE - 1) / BLOCK_SIZE, BLOCK_SIZE>>>(d_states, time(NULL));
    cudaDeviceSynchronize();

    initBoidsGPU(boids, screenWidth, screenHeight, d_states);

    uint64_t previousTime = SDL_GetTicks();
    uint64_t fpsStartTime = SDL_GetTicks();
    while (running  && totalFrames < 10000) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT || 
                (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                running = false;
            } 
            else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT) {
                float mouseX, mouseY;
                SDL_GetMouseState(&mouseX, &mouseY);
                if (target) delete target;
                target = new Vector2(mouseX, mouseY);
            }
            else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_RIGHT) {
                if (target) {
                    delete target;
                    target = nullptr;
                }
            }
            else if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_SPACE) {
                explode = true;
            }
        }

        uint64_t currentTime = SDL_GetTicks();
        float dt = (currentTime - previousTime) / 1000.0f;
        if (dt > 0.1f) dt = 0.1f;
        // fps
        fpsTimer += dt;
        frameCount++;
        totalFrames++;
        if(fpsTimer >= 1.0f){
            fps = frameCount / fpsTimer;
            fpsValues.push_back(fps);
            fpsTimer = 0.0f;
            frameCount = 0;
        }

        previousTime = currentTime;

        SimulateBoids(boids, new_boids, target, screenWidth, screenHeight, dt, explode, d_states);
        RenderScreen(renderer, boids, new_boids, target);
        
        // Cap framerate
        // SDL_Delay(16);
    }
    uint64_t fpsEndTime = SDL_GetTicks();
    float runtime = (fpsEndTime - fpsStartTime) / 1000.0f;

    float averageFps = totalFrames / runtime;
    float minFps = *std::min_element(fpsValues.begin(), fpsValues.end());
    float maxFps = *std::max_element(fpsValues.begin(), fpsValues.end());

    std::cout << "\n--- FPS Statistics (GPU) ---" << std::endl;
    std::cout << "Total frames: " << totalFrames << std::endl;
    std::cout << "Runtime: " << runtime << " seconds" << std::endl;
    std::cout << "Average FPS: " << averageFps << std::endl;
    std::cout << "Min FPS: " << minFps << std::endl;
    std::cout << "Max FPS: " << maxFps << std::endl;

    char buf[100];
    sprintf(buf, "output_GPU_%d.json", NUM_BOIDS);
    writeToFile(buf, totalFrames, runtime, fpsValues);

    if (target) delete target;
    cudaFree(d_states);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}