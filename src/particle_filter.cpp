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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    if (is_initialized) {
        return;
    }

    // Setting the number of particles
    num_particles = 100;

    // Creating normal (Gaussian) distributions for x, y and theta
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

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
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    normal_distribution<double> noise_x(0, std_pos[0]);
    normal_distribution<double> noise_y(0, std_pos[1]);
    normal_distribution<double> noise_theta(0, std_pos[2]);

    for (int i = 0; i < num_particles; ++i) {
        Particle& p = particles[i];
        if (abs(yaw_rate) > 0.0001) {
            double delta_theta = yaw_rate * delta_t;
            double new_theta = p.theta + delta_theta;
            p.x += velocity / yaw_rate * (sin(new_theta) - sin(p.theta));
            p.y += velocity / yaw_rate * (cos(p.theta) - cos(new_theta));
            p.theta = new_theta;
        } else {
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        }

        p.x += noise_x(gen);
        p.y += noise_y(gen);
        p.theta += noise_theta(gen);
    }
}

/*
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
    for (int i = 0; i < predicted.size(); ++i) {
        LandmarkObs p = predicted[i];
        cout << "predicted: (" << p.id << ", " << p.x << ", " << p.y << ")";
    }
    for (int i = 0; i < observations.size(); ++i) {
        LandmarkObs o = observations[i];
        cout << "observations: (" << o.id << ", " << o.x << ", " << o.y << ")";
    }
    //<< ", observations: (" << observations.x << ", " << observations.y << ", " << observations.theta << ")" << endl;
}
*/
void ParticleFilter::dataAssociation(
    std::vector<LandmarkObs>& transformed_observations,
    const std::vector<Map::single_landmark_s>& landmark_list) {

    for (int i = 0; i < transformed_observations.size(); ++i) {
        double min_dist = numeric_limits<double>::max();
        LandmarkObs& obs = transformed_observations[i];

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
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    double std_landmark_x = std_landmark[0];
    double std_landmark_y = std_landmark[1];

    std::vector<LandmarkObs> transformed_observations = observations;
    for (int i = 0; i < num_particles; ++i) {
        Particle& part = particles[i];

        for (int j = 0; j < observations.size(); ++j) {
            LandmarkObs obs = observations[j];
            LandmarkObs& trans_obs = transformed_observations[j];
            trans_obs.id = j;
            trans_obs.x = part.x + cos(part.theta) * obs.x - sin(part.theta) * obs.y;
            trans_obs.y = part.y + sin(part.theta) * obs.x + cos(part.theta) * obs.y;
        }

        dataAssociation(transformed_observations, map_landmarks.landmark_list);

        double weight = 1.0;
        for (int j = 0; j < transformed_observations.size(); ++j) {
            LandmarkObs& trans_obs = transformed_observations[j];

            // Getting landmark associated with observation point
            Map::single_landmark_s landmark = map_landmarks.landmark_list[trans_obs.id-1];

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
    }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    vector<Particle> resampledParticles;

    discrete_distribution<> discrete_distribution(weights.begin(), weights.end());

    for (int i = 0; i < num_particles; ++i) {
        resampledParticles.push_back(particles[discrete_distribution(gen)]);
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
