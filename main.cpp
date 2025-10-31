#include <iostream>
#include <omp.h>
#include <vector>
#include <cmath>

struct Boid {
    float x,y;
    float vx,vy;
    int thread_id;
};

const int NUM_BOIDS = 1000;
const int NUM_STEPS = 1000;
const float WIDTH = 800;
const float HEIGHT = 600;

// Boids algorithm parameters
const float VISUAL_RANGE = 40.0f;
const float PROTECTED_RANGE = 8.0f;
const float CENTERING_FACTOR = 0.0005f;  // Cohesion
const float AVOID_FACTOR = 0.05f;        // Separation
const float MATCHING_FACTOR = 0.05f;     // Alignment
const float TURN_FACTOR = 0.2f;          // Bordo schermo
const float MAX_SPEED = 6.0f;
const float MIN_SPEED = 3.0f;

// Inizializzazione boid casuali
void init_boids(std::vector<Boid>& boids) {
    for(auto &b : boids){
        b.x = static_cast<float>(rand())/RAND_MAX * WIDTH;
        b.y = static_cast<float>(rand())/RAND_MAX * HEIGHT;
        b.vx = (static_cast<float>(rand())/RAND_MAX - 0.5f) * 2;
        b.vy = (static_cast<float>(rand())/RAND_MAX - 0.5f) * 2;
    }
}

float getDistance(const Boid& a, const Boid& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;

    return sqrt((dx*dx) + (dy * dy));

}


void boids_rules(Boid& a, const std::vector<Boid>& boids) {
    float close_dx = 0;
    float close_dy = 0;

    float xvel_avg = 0;
    float yvel_avg = 0;
    int neighboring_boids = 0;
    float xpos_avg = 0;
    float ypos_avg = 0 ;

    for (const auto& other : boids) {
        if (&a == &other) continue;
        float distance = getDistance(a,other);
        //Separation
        if (distance < PROTECTED_RANGE) {
            close_dx += a.x - other.x;
            close_dy += a.y - other.y;
        }
        //Alignment e Cohesion
        if (distance < VISUAL_RANGE) {
            xvel_avg += other.vx;
            yvel_avg += other.vy;

            xpos_avg += other.x;
            ypos_avg += other.y;

            neighboring_boids += 1;
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
        //printf("Boid di T: %d fuori! Rientra\n",a.thread_id);
        a.vx += TURN_FACTOR;
    }
    if (a.x > WIDTH - 50) {
        a.vx -= TURN_FACTOR;
    }
    if (a.y < 50) {
        a.vy += TURN_FACTOR;
    }
    if (a.y > HEIGHT - 50) {
        a.vy -= TURN_FACTOR;
    }

    //check boids velocity
    float speed = sqrt(a.vx * a.vx + a.vy * a.vy);
    if (speed > MAX_SPEED) {
        a.vx = (a.vx / speed)*MAX_SPEED;
        a.vy = (a.vy / speed)*MAX_SPEED;
    }else if (speed < MIN_SPEED) {
        a.vx = (a.vx / speed)*MIN_SPEED;
        a.vy = (a.vy / speed)*MIN_SPEED;
    }
    //a.x += a.vx;
    //a.y += a.vy;

}
void update_position(Boid& boid) {
    boid.x += boid.vx;
    boid.y += boid.vy;

    if (boid.x < 0)       boid.x += WIDTH;
    if (boid.x >= WIDTH)  boid.x -= WIDTH;
    if (boid.y < 0)       boid.y += HEIGHT;
    if (boid.y >= HEIGHT) boid.y -= HEIGHT;
}

int main() {
    std::vector<Boid> boids(NUM_BOIDS);
    std::vector<Boid> old_boids(NUM_BOIDS);
    init_boids(boids);

    for (int i = 0; i < NUM_STEPS; i++) {
        old_boids = boids;
        #pragma omp parallel
        {
        #pragma omp for
            for (int i = 0; i < NUM_BOIDS; i++) {
                boids[i].thread_id = omp_get_thread_num();
                boids_rules(boids[i], old_boids);
            }

            #pragma omp for
            for (int i = 0; i < NUM_BOIDS; i++) {
                update_position(boids[i]);
            }
        }
        if(i % 100 == 0){
        #pragma omp critical
            printf("step: %d\n",i);
            for (int j = 0; j< 3;j++) {
                printf("Boid[%d] Thread: %d\n",j,boids[j].thread_id);
                printf("X: %f , Y: %f\n",boids[j].x,boids[j].y);
                printf("Vx: %f, Vy: %f\n",boids[j].vx,boids[j].vy);
            }
        }
    }
    return 0;

}