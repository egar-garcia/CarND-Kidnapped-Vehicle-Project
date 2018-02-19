/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

#define NUM_PARTICLES 100


void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // Setting the number of particles. Initializing all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Adding random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    if (is_initialized) {
        return;
    }

    // Setting the number of particles
    num_particles = NUM_PARTICLES;

    // Creating normal (Gaussian) distributions for x, y and theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // Creating the random particles based in the normal distributions
    for (int i = 0; i < num_particles; ++i) {
        Particle part;
        part.id = i;
        part.x = dist_x(gen);
        part.y = dist_y(gen);
        part.theta = dist_theta(gen);
        part.weight = 1.0;
        particles.push_back(part);

        weights.push_back(1.0);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // Adding measurements to each particle and adding random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    // Creating normal (Gaussian) distributions to add noise to x, y and theta
    normal_distribution<double> noise_x(0, std_pos[0]);
    normal_distribution<double> noise_y(0, std_pos[1]);
    normal_distribution<double> noise_theta(0, std_pos[2]);

    for (int i = 0; i < num_particles; ++i) {
        // Performing prediction for each particle
        Particle& p = particles[i];
        if (abs(yaw_rate) > 0.0001) {
            double delta_theta = yaw_rate * delta_t;
            double new_theta = p.theta + delta_theta;
            p.x += velocity / yaw_rate * (sin(new_theta) - sin(p.theta));
            p.y += velocity / yaw_rate * (cos(p.theta) - cos(new_theta));
            p.theta = new_theta;
        } else { // If Yaw rate is 0 just considering theta to do the prediction
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        }

        // Adding random Gaussian noise
        p.x += noise_x(gen);
        p.y += noise_y(gen);
        p.theta += noise_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs>& transformed_observations,
    const std::vector<Map::single_landmark_s>& landmark_list) {
    // Finding the landmark that is closest to each observed (and transformed) measurement and assign the
    //   observed measurement to this particular landmark.

    for (int i = 0; i < transformed_observations.size(); ++i) {
        LandmarkObs& obs = transformed_observations[i];

        // Looking for the closest landmark to the current observed measurement
        double min_dist = numeric_limits<double>::max();
        for (int j = 0; j < landmark_list.size(); ++j) {
            Map::single_landmark_s landmark = landmark_list[j];
            double dist = sqrt(pow(obs.x - landmark.x_f, 2) + pow(obs.y - landmark.y_f, 2));

            if (dist < min_dist) {
                obs.id = landmark.id_i;
                min_dist = dist;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // Updating the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Particles are located
    //   according to the MAP'S coordinate system. It is needed to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html

    double std_landmark_x = std_landmark[0];
    double std_landmark_y = std_landmark[1];

    // Creating a vector to store the trasformed observations to MAP's coordinate system
    std::vector<LandmarkObs> transformed_observations = observations;

    for (int i = 0; i < num_particles; ++i) {
        Particle& part = particles[i];

        // Transforming observations to MAP's coordinate system
        for (int j = 0; j < observations.size(); ++j) {
            LandmarkObs obs = observations[j];
            LandmarkObs& trans_obs = transformed_observations[j];
            trans_obs.id = j;
            trans_obs.x = part.x + cos(part.theta) * obs.x - sin(part.theta) * obs.y;
            trans_obs.y = part.y + sin(part.theta) * obs.x + cos(part.theta) * obs.y;
        }

        // Assiciating (transformed) observations to their closest landmak
        dataAssociation(transformed_observations, map_landmarks.landmark_list);

        std::vector<int> associations;
        std::vector<double> sense_x;
        std::vector<double> sense_y;

        // Calculating weight of the current particle
        double weight = 1.0;
        for (int j = 0; j < transformed_observations.size(); ++j) {
            LandmarkObs& trans_obs = transformed_observations[j];

            // Getting landmark associated with observation point.
            // In this case the id can be used to retrieve the index,
            // since the landmarks' vector is sorted by id and consecutive
            Map::single_landmark_s landmark = map_landmarks.landmark_list[trans_obs.id-1];

            associations.push_back(trans_obs.id);
            sense_x.push_back(trans_obs.x);
            sense_y.push_back(trans_obs.y);

            // Calculating normalization term
            double gauss_norm = 1 / (2 * M_PI * std_landmark_x * std_landmark_y);
            // Calculating exponent
            double exponent = (pow(trans_obs.x - landmark.x_f, 2)) / (2 * pow(std_landmark_x, 2)) +
                              (pow(trans_obs.y - landmark.y_f, 2)) / (2 * pow(std_landmark_y, 2));
            // Calculating weight using normalization terms and exponent
            weight *= gauss_norm * exp(-exponent);
				}

        part.weight = weight;
        weights[i] = weight;

        SetAssociations(part, associations, sense_x, sense_y);
    }
}

void ParticleFilter::resample() {
    // Resampling particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // Vector to store the resampled particles
    vector<Particle> resampledParticles;

    // Discrete distribution that uses the weight to do the resample proportionaly
    discrete_distribution<> discrete_distribution(weights.begin(), weights.end());
    // Getting the particles to include in the resample using the discrete distribution
    for (int i = 0; i < num_particles; ++i) {
        Particle particle = particles[discrete_distribution(gen)];
        particle.id = i;
        resampledParticles.push_back(particle);
    }

    particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
    const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
