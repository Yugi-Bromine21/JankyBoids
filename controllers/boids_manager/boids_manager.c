/*
 * File:          boids_manager.c
 * Description:   Boids flocking behavior for 15 Mavic 2 Pro drones
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

 // Parameters for the boids algorithm
#define DRONE_COUNT 15
#define SPEED 0.5
#define TIME_STEP 64
#define SEPARATION_WEIGHT 2.0
#define ALIGNMENT_WEIGHT 1.0
#define COHESION_WEIGHT 0.5
#define D_SEPARATION 2.0
#define D_ALIGNMENT 5.0
#define D_COHESION 10.0
#define MAX_FORCE 1.0

// Calculate distance between two 3D points
double distance(const double pos1[3], const double pos2[3]) {
    return sqrt(pow(pos1[0] - pos2[0], 2) +
        pow(pos1[1] - pos2[1], 2) +
        pow(pos1[2] - pos2[2], 2));
}

// Limit a vector's magnitude
void limit_vector(double vec[3], double max_magnitude) {
    double mag = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (mag > max_magnitude && mag > 0) {
        vec[0] = (vec[0] / mag) * max_magnitude;
        vec[1] = (vec[1] / mag) * max_magnitude;
        vec[2] = (vec[2] / mag) * max_magnitude;
    }
}

// Calculate the Separation force - steer away from nearby neighbors
void calculate_separation(const double pos[3], const double neighbors[][3], int neighbor_count, double result[3]) {
    result[0] = 0.0;
    result[1] = 0.0;
    result[2] = 0.0;

    int count = 0;
    for (int i = 0; i < neighbor_count; i++) {
        double d = distance(pos, neighbors[i]);
        if (d < D_SEPARATION && d > 0.001) {
            // Vector pointing away from neighbor, weighted by distance
            result[0] += (pos[0] - neighbors[i][0]) / (d * d);
            result[1] += (pos[1] - neighbors[i][1]) / (d * d);
            result[2] += (pos[2] - neighbors[i][2]) / (d * d);
            count++;
        }
    }

    if (count > 0) {
        result[0] /= count;
        result[1] /= count;
        result[2] /= count;
    }
}

// Calculate the Alignment force - steer towards average velocity of neighbors
void calculate_alignment(const double velocities[][3], const double distances[], int neighbor_count, double result[3]) {
    result[0] = 0.0;
    result[1] = 0.0;
    result[2] = 0.0;

    int count = 0;
    for (int i = 0; i < neighbor_count; i++) {
        if (distances[i] < D_ALIGNMENT && distances[i] > 0.001) {
            result[0] += velocities[i][0];
            result[1] += velocities[i][1];
            result[2] += velocities[i][2];
            count++;
        }
    }

    if (count > 0) {
        result[0] /= count;
        result[1] /= count;
        result[2] /= count;
    }
}

// Calculate the Cohesion force - steer towards center of mass of neighbors
void calculate_cohesion(const double pos[3], const double neighbors[][3], const double distances[], int neighbor_count, double result[3]) {
    double center_of_mass[3] = { 0.0, 0.0, 0.0 };
    int count = 0;

    for (int i = 0; i < neighbor_count; i++) {
        if (distances[i] < D_COHESION && distances[i] > 0.001) {
            center_of_mass[0] += neighbors[i][0];
            center_of_mass[1] += neighbors[i][1];
            center_of_mass[2] += neighbors[i][2];
            count++;
        }
    }

    if (count > 0) {
        center_of_mass[0] /= count;
        center_of_mass[1] /= count;
        center_of_mass[2] /= count;

        // Vector pointing towards center of mass
        result[0] = center_of_mass[0] - pos[0];
        result[1] = center_of_mass[1] - pos[1];
        result[2] = center_of_mass[2] - pos[2];
    }
    else {
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.0;
    }
}

int main(int argc, char** argv) {
    wb_robot_init();

    WbNodeRef drones[DRONE_COUNT];
    WbFieldRef drones_translation_fields[DRONE_COUNT];

    // Store velocities for each drone (needed for alignment)
    double velocities[DRONE_COUNT][3];

    // Initialize drones and their translation fields
    for (int i = 0; i < DRONE_COUNT; i++) {
        char def_name[16];
        sprintf(def_name, "DRONE_%d", i);

        drones[i] = wb_supervisor_node_get_from_def(def_name);

        if (drones[i] == NULL) {
            printf("Error: Could not find node with DEF name %s\n", def_name);
            continue;
        }

        drones_translation_fields[i] = wb_supervisor_node_get_field(drones[i], "translation");

        // Initialize velocities with small random values
        velocities[i][0] = ((double)rand() / RAND_MAX - 0.5) * 0.1;
        velocities[i][1] = 0.0;  // Keep Y (height) velocity minimal
        velocities[i][2] = ((double)rand() / RAND_MAX - 0.5) * 0.1;
    }

    // Main simulation loop
    while (wb_robot_step(TIME_STEP) != -1) {
        // First, gather all current positions
        double all_positions[DRONE_COUNT][3];

        for (int i = 0; i < DRONE_COUNT; i++) {
            if (drones[i] == NULL) continue;

            const double* pos = wb_supervisor_field_get_sf_vec3f(drones_translation_fields[i]);
            if (pos) {
                all_positions[i][0] = pos[0];
                all_positions[i][1] = pos[1];
                all_positions[i][2] = pos[2];
            }
        }

        // Update each drone
        for (int i = 0; i < DRONE_COUNT; i++) {
            if (drones[i] == NULL) continue;

            double current_pos[3] = { all_positions[i][0], all_positions[i][1], all_positions[i][2] };

            // Build neighbor lists (all other drones)
            double neighbor_positions[DRONE_COUNT - 1][3];
            double neighbor_velocities[DRONE_COUNT - 1][3];
            double neighbor_distances[DRONE_COUNT - 1];
            int neighbor_count = 0;

            for (int j = 0; j < DRONE_COUNT; j++) {
                if (i == j || drones[j] == NULL) continue;

                neighbor_positions[neighbor_count][0] = all_positions[j][0];
                neighbor_positions[neighbor_count][1] = all_positions[j][1];
                neighbor_positions[neighbor_count][2] = all_positions[j][2];

                neighbor_velocities[neighbor_count][0] = velocities[j][0];
                neighbor_velocities[neighbor_count][1] = velocities[j][1];
                neighbor_velocities[neighbor_count][2] = velocities[j][2];

                neighbor_distances[neighbor_count] = distance(current_pos, all_positions[j]);
                neighbor_count++;
            }

            // Calculate boids forces
            double separation_force[3], alignment_force[3], cohesion_force[3];

            calculate_separation(current_pos, neighbor_positions, neighbor_count, separation_force);
            calculate_alignment(neighbor_velocities, neighbor_distances, neighbor_count, alignment_force);
            calculate_cohesion(current_pos, neighbor_positions, neighbor_distances, neighbor_count, cohesion_force);

            // Apply weights to forces
            double acceleration[3];
            acceleration[0] = SEPARATION_WEIGHT * separation_force[0] +
                ALIGNMENT_WEIGHT * alignment_force[0] +
                COHESION_WEIGHT * cohesion_force[0];
            acceleration[1] = SEPARATION_WEIGHT * separation_force[1] +
                ALIGNMENT_WEIGHT * alignment_force[1] +
                COHESION_WEIGHT * cohesion_force[1];
            acceleration[2] = SEPARATION_WEIGHT * separation_force[2] +
                ALIGNMENT_WEIGHT * alignment_force[2] +
                COHESION_WEIGHT * cohesion_force[2];

            // Limit acceleration
            limit_vector(acceleration, MAX_FORCE);

            // Update velocity
            velocities[i][0] += acceleration[0];
            velocities[i][1] += acceleration[1];
            velocities[i][2] += acceleration[2];

            // Limit velocity (speed)
            limit_vector(velocities[i], SPEED);

            // Keep drones at roughly the same altitude (optional - comment out for full 3D)
            velocities[i][1] *= 0.1;  // Dampen vertical movement

            // Calculate new position
            double new_pos[3];
            double dt = TIME_STEP / 1000.0;  // Convert to seconds
            new_pos[0] = current_pos[0] + velocities[i][0] * dt;
            new_pos[1] = current_pos[1] + velocities[i][1] * dt;
            new_pos[2] = current_pos[2] + velocities[i][2] * dt;

            // Optional: Keep drones within bounds
            double bounds = 10.0;
            double min_height = 1.0;
            double max_height = 5.0;

            // Soft boundary - steer back if near edge
            if (new_pos[0] > bounds) velocities[i][0] -= 0.5;
            if (new_pos[0] < -bounds) velocities[i][0] += 0.5;
            if (new_pos[2] > bounds) velocities[i][2] -= 0.5;
            if (new_pos[2] < -bounds) velocities[i][2] += 0.5;
            if (new_pos[1] < min_height) velocities[i][1] += 0.5;
            if (new_pos[1] > max_height) velocities[i][1] -= 0.5;

            // Clamp position within hard bounds
            if (new_pos[0] > bounds * 1.5) new_pos[0] = bounds * 1.5;
            if (new_pos[0] < -bounds * 1.5) new_pos[0] = -bounds * 1.5;
            if (new_pos[2] > bounds * 1.5) new_pos[2] = bounds * 1.5;
            if (new_pos[2] < -bounds * 1.5) new_pos[2] = -bounds * 1.5;
            if (new_pos[1] < min_height) new_pos[1] = min_height;
            if (new_pos[1] > max_height) new_pos[1] = max_height;

            // Update drone position
            wb_supervisor_field_set_sf_vec3f(drones_translation_fields[i], new_pos);
        }
    }

    wb_robot_cleanup();
    return 0;
}