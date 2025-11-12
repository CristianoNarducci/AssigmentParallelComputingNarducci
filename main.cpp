#include <cmath>
#include <iostream>
#include <omp.h>
#include <vector>
#include <chrono>
#include <random>
#include <SFML/Graphics.hpp>

//AoS
struct Boid {
    float x,y;
    float vx,vy;
    int thread_id = 0;
};

//SoA
struct Boids {
    std::vector<float> x,y;
    std::vector<float> vx,vy;
};

const int NUM_BOIDS = 1000;
const int NUM_STEPS = 1000;
const int WIDTH = 1000;
const int HEIGHT = 1600;

// Boids algorithm parameters
const float VISUAL_RANGE = 40.0f;
const float PROTECTED_RANGE = 8.0f;
const float CENTERING_FACTOR = 0.0005f;  // Cohesion
const float AVOID_FACTOR = 0.05f;        // Separation
const float MATCHING_FACTOR = 0.05f;     // Alignment
const float TURN_FACTOR = 0.2f;          // Bordo schermo
const float MAX_SPEED = 6.0f;
const float MIN_SPEED = 3.0f;

float random_float(float min, float max) {
    static std::mt19937 gen(42); // seed fisso
    std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
}
// Inizializzazione boid casuali
void init_boids(std::vector<Boid>& boids) {

    for(auto &b : boids){
        b.x = random_float(0.0f,WIDTH);
        b.y = random_float(0.0f,HEIGHT);
        b.vx = random_float(1.0f,3.0f);
        b.vy = random_float(1.0f,3.0f);
    }
}

void init_boids_soa(Boids& boids) {
    boids.x.reserve(NUM_BOIDS);
    boids.y.reserve(NUM_BOIDS);
    boids.vx.reserve(NUM_BOIDS);
    boids.vy.reserve(NUM_BOIDS);
    for (int i = 0; i < NUM_BOIDS; i++) {
        boids.x.push_back(static_cast<float>(rand())/RAND_MAX * WIDTH);
        boids.y.push_back(static_cast<float>(rand())/RAND_MAX * HEIGHT);
        boids.vx.push_back(static_cast<float>(rand())/RAND_MAX - 0.5f);
        boids.vy.push_back(static_cast<float>(rand())/RAND_MAX - 0.5f);

    }
}

void update_position(Boid& boid) {
    boid.x += boid.vx;
    boid.y += boid.vy;

}
float getDistance(const Boid& a, const Boid& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;

    return std::sqrt((dx*dx) + (dy * dy));

}


void boids_rules(Boid& a, const std::vector<Boid>& boids) {
    float close_dx = 0;
    float close_dy = 0;

    float xvel_avg = 0;
    float yvel_avg = 0;
    int neighboring_boids = 0;
    float xpos_avg = 0;
    float ypos_avg = 0;

    for (const auto& other : boids) {
        if (&a == &other) continue;
        float distance = getDistance(a,other);
        //Separation
        if (distance < PROTECTED_RANGE) {
            //printf("Boids T: %d in separation with boids T: %d\n",a.thread_id,other.thread_id);
            close_dx += a.x - other.x;
            close_dy += a.y - other.y;
        }
        //Alignment e Cohesion
        if (distance < VISUAL_RANGE) {
            xvel_avg += other.vx;
            yvel_avg += other.vy;

            xpos_avg += other.x;
            ypos_avg += other.y;

            neighboring_boids++;
        }

    }

    if (neighboring_boids > 0) {
        xvel_avg = xvel_avg / static_cast<float>(neighboring_boids);
        yvel_avg = yvel_avg / static_cast<float>(neighboring_boids);

        xpos_avg = xpos_avg / static_cast<float>(neighboring_boids);
        ypos_avg = ypos_avg / static_cast<float>(neighboring_boids);

        a.vx += (xvel_avg - a.vx) * MATCHING_FACTOR; //velocity update wrt alignment rule
        a.vy += (yvel_avg - a.vy) * MATCHING_FACTOR;

        a.vx += (xpos_avg - a.x) * CENTERING_FACTOR; //velocity update wrt cohesion rule
        a.vy += (ypos_avg - a.y) * CENTERING_FACTOR;
    }

    a.vx += close_dx * AVOID_FACTOR;    //velocity update wrt separation rule
    a.vy += close_dy * AVOID_FACTOR;

    //check boids margin
    if (a.x < 50) {
        a.vx += TURN_FACTOR;
    }
    if (a.x >= WIDTH) {
        a.vx -= TURN_FACTOR;
    }
    if (a.y < 50) {
        a.vy += TURN_FACTOR;
    }
    if (a.y > HEIGHT - 50) {
        a.vy -= TURN_FACTOR;
    }

    //check boids velocity
    float speed = std::sqrt(a.vx * a.vx + a.vy * a.vy);
    if (speed > MAX_SPEED) {
        a.vx = (a.vx / speed)*MAX_SPEED;
        a.vy = (a.vy / speed)*MAX_SPEED;
    }else if (speed < MIN_SPEED) {
        a.vx = (a.vx / speed)*MIN_SPEED;
        a.vy = (a.vy / speed)*MIN_SPEED;
    }
}

void boids_rules_soa(Boids& a, const Boids& old, int index) {
    float close_dx = 0;
    float close_dy = 0;

    float xvel_avg = 0;
    float yvel_avg = 0;
    int neighboring_boids = 0;
    float xpos_avg = 0;
    float ypos_avg = 0;

    for (int j = 0; j < NUM_BOIDS;j++) {
        if (j == index) continue;
        float dx = old.x[index] - old.x[j];
        float dy = old.y[index] - old.y[j];
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance < PROTECTED_RANGE) {
            close_dx += dx;
            close_dy += dy;
        }

        if (distance < VISUAL_RANGE) {
            xvel_avg += old.vx[j];
            yvel_avg += old.vy[j];

            xpos_avg += old.x[j];
            ypos_avg += old.y[j];

            neighboring_boids++;
        }
    }
    if (neighboring_boids > 0) {
        xvel_avg /= static_cast<float>(neighboring_boids);
        yvel_avg /= static_cast<float>(neighboring_boids);

        xpos_avg /= static_cast<float>(neighboring_boids);
        ypos_avg /= static_cast<float>(neighboring_boids);

        a.vx[index] += (xvel_avg - old.vx[index]) * MATCHING_FACTOR;
        a.vy[index] += (yvel_avg - old.vy[index]) * MATCHING_FACTOR;

        a.vx[index] += (xpos_avg - old.x[index]) * CENTERING_FACTOR;
        a.vy[index] += (ypos_avg - old.y[index]) * CENTERING_FACTOR;
    }
    a.vx[index] += close_dx * AVOID_FACTOR;
    a.vy[index] += close_dy * AVOID_FACTOR;

    if (a.x[index] < 50) a.vx[index] += TURN_FACTOR;
    if (a.x[index] > WIDTH - 50) a.vx[index] -= TURN_FACTOR;
    if (a.y[index] < 50) a.vy[index] += TURN_FACTOR;
    if (a.y[index] > HEIGHT - 50) a.vy[index] -= TURN_FACTOR;

    float speed = std::sqrt(a.vx[index]*a.vx[index] + a.vy[index]*a.vy[index]);
    if (speed > MAX_SPEED) {
        a.vx[index] = a.vx[index] / speed * MAX_SPEED;
        a.vy[index] = a.vy[index] / speed * MAX_SPEED;
    }else if (speed < MIN_SPEED) {
        a.vx[index] = a.vx[index] / speed * MIN_SPEED;
        a.vy[index] = a.vy[index] / speed * MIN_SPEED;
    }

}
void update_positions_soa(Boids& boids,int i) {
    boids.x[i] += boids.vx[i];
    boids.y[i] += boids.vy[i];

    if (boids.x[i] < 0)   boids.x[i] += WIDTH;
    if (boids.x[i] >= WIDTH)  boids.x[i] -= WIDTH;
    if (boids.y[i] < 0)       boids.y[i] += HEIGHT;
    if (boids.y[i] >= HEIGHT) boids.y[i] -= HEIGHT;

}
void boids_no_graphical() {
    std::vector<Boid> boids(NUM_BOIDS);
    std::vector<Boid> old_boids(NUM_BOIDS);
    init_boids(boids);
    double start,end;
    printf("START AOS VERSION\n");
    start = omp_get_wtime();
    for (int i = 0; i < NUM_STEPS; i++) {
        std::swap(boids,old_boids);

        #pragma omp parallel
        {
            #pragma omp for
            for (int i = 0; i < NUM_BOIDS; i++) {
                boids[i].thread_id = omp_get_thread_num();
                boids_rules(boids[i], old_boids);
            }
            #pragma omp for nowait
            for (int i = 0; i < NUM_BOIDS; i++) {
                update_position(boids[i]);
            }
        }
    }
    end = omp_get_wtime();
    printf("Time elapsed parallel mode AOS: %g s\n", end - start);

    printf("START SOA VERSION\n");
    //SoA
    Boids b,old;
    init_boids_soa(b);
    old.x.resize(NUM_BOIDS);
    old.y.resize(NUM_BOIDS);
    old.vx.resize(NUM_BOIDS);
    old.vy.resize(NUM_BOIDS);

    start = omp_get_wtime();
    for (int step = 0; step < NUM_STEPS; step++) {
        std::swap(b,old);//old = b;
        #pragma omp parallel
        {
            #pragma omp for
            for (int i =0; i < NUM_BOIDS; i++) {
                boids_rules_soa(b,old,i);
            }
            #pragma omp for nowait
            for (int i = 0; i < NUM_BOIDS; i++) {
                update_positions_soa(b,i);
            }
        }
    }
    end = omp_get_wtime();
    printf("Time elapsed parallel mode SOA: %g s\n", end - start);
}

int main() {
#ifdef _OPENMP
    printf("OPENMP VERSION: %d\n", _OPENMP);
#else
    printf("Error openmp not found!\n");
#endif
    printf("START BOIDS ALGORITHM\n");
    printf("START TIMES COMPARATIONS WITHOUT GRAPHICAL VISUALIZATION\n");
    boids_no_graphical();
    sf::RenderWindow window(sf::VideoMode({WIDTH + 300,HEIGHT + 300},32),"Boids Simulation",sf::Style::None,sf::State::Fullscreen);
    window.setFramerateLimit(60);
    std::vector<Boid> boids(NUM_BOIDS);
    std::vector<Boid> old_boids(NUM_BOIDS);
    init_boids(boids);
    sf::CircleShape shape(2);
    shape.setFillColor(sf::Color::White);
    printf("START AOS VERSION WITH GRAPHICAL VISUALIZATION\n");
    while (window.isOpen()){
        for (int i = 0; i < NUM_STEPS; i++) {
            std::swap(boids,old_boids);
            #pragma omp parallel
            {
                #pragma omp for
                for (int i = 0; i < NUM_BOIDS; i++) {
                    boids[i].thread_id = omp_get_thread_num();
                    boids_rules(boids[i], old_boids);
                }

                #pragma omp for nowait
                for (int i = 0; i < NUM_BOIDS; i++) {
                    update_position(boids[i]);
                }
            }
            window.clear(sf::Color::Black);
            for (auto& b: boids) {
                shape.setPosition({b.x,b.y});
                window.draw(shape);
            }
            window.display();
        }
        window.close();
    }
    return 0;
}