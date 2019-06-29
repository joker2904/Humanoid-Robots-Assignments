#include <particle_filter/ParticleFilter.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <algorithm>




namespace particle_filter {

/**
 * \brief Calculate the probability phi(d, stdev) of a measurement according to a Gaussian distribution.
 * \param[in] d The difference between the measurement and the mean
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return Probability of the measurement.
 */
double ParticleFilter::gaussianProbability(const double& d, const double& stdev) {
	double probability = 0.0;
	/*TODO: Calculate the probability of the measurement for a Gaussian distribution with
	  the given mean and standard deviation */
        double e = -(d*d)/(2*stdev*stdev);
        e = exp(e);
        e = e / sqrt(2*M_PI*stdev*stdev);
        //std::cout<<"\n"<<e;
	return e;
}

/**
 * \brief Draw a sample from a Gaussian distribution.
 * \param[in] mean The mean of the Gaussian.
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return A random sample drawn from the given Gaussian distribution.
 */
double ParticleFilter::sampleFromGaussian(const double& mean, const double& stdev) {
	double result = 0.0;
	//TODO: draw a sample from a 1D Gaussian
        //generate a random probability between 0 and 1
         
        double s = 0.0;
        for(int i=0;i<12;++i)
            s += -stdev + ((double)(rand())*(2.0*stdev)/(double)(2147483647));
        s /= 2.0;
       result = mean +s;
      
    return result;
}


/**
 * \brief Initializes the position and weights of the particles.
 * \param[in,out] particles The list of particles.
 *
 * The positions should be distributed uniformly in the interval [0, 10].
 * The weights should be equal and sum up to 1.
 */
void ParticleFilter::initParticles(std::vector<Particle>& particles) {
	//TODO: Distribute the particles randomly between [0, 10] with equal weights
    int n = particles.size();
    for( std::vector<Particle>::iterator itr = particles.begin() ; itr != particles.end(); ++itr)
    {
       itr->x = rand()%10;
       itr->weight = double(1.0/double(n));
       //std::cout<<std::endl<<itr->x<<"  "<<itr->weight;
       
    }
}

/**
 * \brief Normalizes the weights of the particle set so that they sum up to 1.
 * \param[in,out] particles The list of particles.
 */
void ParticleFilter::normalizeWeights(std::vector<Particle>& particles) {
	//TODO: normalize the particles' weights so that they sum up to 1.
    double sum = 0.0;
    for( std::vector<Particle>::iterator itr = particles.begin() ; itr != particles.end(); ++itr)
    {
       sum +=  itr->weight;
    }
    for( std::vector<Particle>::iterator itr = particles.begin() ; itr != particles.end(); ++itr)
    {
       itr->weight /= sum;
    }
}

/**
 * \brief Displace the particles according to the robot's movements.
 * \param[in,out] particles The list of particles.
 * \param[in] ux The odometry (displacement) of the robot along the x axis.
 * \param[in] stdev The standard deviation of the motion model.
 */
void ParticleFilter::integrateMotion(std::vector<Particle>& particles, const double& ux, const double& stdev) {
	//TODO: Prediction step: Update each sample by drawing the a pose from the motion model.
    for( std::vector<Particle>::iterator itr = particles.begin() ; itr != particles.end(); ++itr)
    {
       //std::cout<<std::endl<<" motion::"<<sampleFromGaussian( ux, stdev);
       itr->x = sampleFromGaussian( ux+itr->x, stdev);
       //std::cout<<"  "<<itr->x;
       
    }
}


/**
 * \brief Returns the distance between the given x position and the nearest light source.
 * \param[in] x The position on the x axis.
 * \return The distance to the nearest light source.
 */
double ParticleFilter::getDistanceToNearestLight(const double& x) {
	double dist = 0.0;
	//TODO Return the distance from the robot's position x to the nearest light source.
        std::vector<double> d;
        //d.resize(3);
        d.push_back( fabs(x-2.000) );
        d.push_back( fabs(x-6.000) );
        d.push_back( fabs(x-8.000) );
        std::sort(d.begin(),d.end());
        dist = d.at(0);
	return dist;
}

/**
 * \brief Updates the particle weights according to the measured distance to the nearest light source.
 * \param[in,out] particles The list of particles.
 * \param[in] measurement The measured distance between the robot and the nearest light source.
 * \param[in] stdev The standard deviation of the observation model.
 */

void ParticleFilter::integrateObservation(std::vector<Particle>& particles, const double measurement,
		const double& stdev) {
	//TODO: Correction step: weight the samples according to the observation model.

	// Normalize the weights after updating so that they sum up to 1 again:
       
        for( std::vector<Particle>::iterator itr = particles.begin() ; itr != particles.end(); ++itr)
        {
           itr->weight = gaussianProbability(getDistanceToNearestLight(itr->x) - measurement  , stdev);
        }
     
	normalizeWeights(particles);
}

/**
 * \brief Resamples the particle set by throwing out unlikely particles and duplicating more likely ones.
 * \param[in] particles The old list of particles.
 * \return The new list of particles after resampling.
 */
std::vector<ParticleFilter::Particle> ParticleFilter::resample(const std::vector<Particle>& particles) {
	std::vector<Particle> newParticles;
	/*TODO: Use stochastic universal resampling (also called low variance resampling)
	 * to draw a new set of particles according to the old particles' weights */
        double J = 1.0/double(particles.size());
        double r = double(rand())*(J/double(RAND_MAX));
	if (r > J)
		r = J;
       	double c = particles[0].weight;
	int i = 0;
	for(int j=0; j < particles.size(); j++) 
        {
	  double U = r + j*J;
          while(U > c)
          {
	      if (i < particles.size())
	          c += particles[++i].weight;
          }	
          if(i < particles.size())
	     newParticles.push_back(particles[i]);
	} 
	return newParticles;
}

}  // namespace particle_filter

