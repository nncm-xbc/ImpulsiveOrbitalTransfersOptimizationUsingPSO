#include "Swarm.hpp"

#include <cstddef>
#include <memory>
#include <omp.h>
#include <iomanip>
#include <vector>

template <typename T, typename Fun>
Swarm<T, Fun>::Swarm(const size_t &numParticles,
    const size_t &dimension,
    const Fun &objectiveFunction,
    const T inertiaWeight,
    const T cognitiveWeight,
    const T socialWeight,
    const std::vector<T> lowerBounds,
    const std::vector<T> upperBounds):
        _numParticles(numParticles),
        _dimension(dimension),
        _objectiveFunction(objectiveFunction),
        _inertiaWeight(inertiaWeight),
        _cognitiveWeight(cognitiveWeight),
        _socialWeight(socialWeight),
        _lowerBounds(lowerBounds),
        _upperBounds(upperBounds)
        {}

template <typename T, typename Fun> Swarm<T, Fun>::~Swarm()
{
    deallocateMemory();
}

template <typename T, typename Fun>
void Swarm<T, Fun>::init()
{
    std::random_device rd;
    _rng = std::mt19937(rd());
    _dis = std::vector<std::uniform_real_distribution<T>>(_dimension);
    _gBestVal = std::numeric_limits<T>::max();
    _gBestPos.resize(_dimension);

    double margin = 0.05;

    for (size_t i = 0; i < _dimension; ++i)
    {
        double range = _upperBounds[i] - _lowerBounds[i];
        double adjustedLower = _lowerBounds[i] + (margin * range);
        double adjustedUpper = _upperBounds[i] - (margin * range);
        _dis[i] = (std::uniform_real_distribution<T>(adjustedLower, adjustedUpper));
    }

    for(size_t i = 0; i < _numParticles; ++i) {
        particles.emplace_back(Particle<T, Fun>(_objectiveFunction, _dimension, _rng, _dis, _lowerBounds, _upperBounds));
    }

    for (size_t i = 0; i < _numParticles; i++)
    {
        std::cout << "Particle" << i << " x: " << particles[i].getPosition()[0] << ";  y: "<< particles[i].getPosition()[1] << ";  z: "<< particles[i].getPosition()[2] << ";  w: "<< particles[i].getPosition()[3] << ";   val : "<< particles[i].getValue() << std::endl;
    }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePosition(Particle<T, Fun> &particle)
{
    //std::cout << "Update POSITION called" << std::endl;
    std::vector<T> newPosition = particle.getPosition();
    const std::vector<T> &velocity = particle.getVelocity();

    for (size_t i = 0; i < _dimension; ++i)
    {

        newPosition[i] += velocity[i];

        // boundary checks
        if (newPosition[i] < _lowerBounds[i])
        {
            newPosition[i] = _lowerBounds[i];
            std::vector<T> newVelocity = particle.getVelocity();
            newVelocity[i] = 0.0;
            particle.setVelocity(newVelocity);
            //TODO: set cognitive weight to zero.
        }
        else if (newPosition[i] > _upperBounds[i])
        {
            newPosition[i] = _upperBounds[i];

            std::vector<T> newVelocity = particle.getVelocity();
            newVelocity[i] = 0.0;
            particle.setVelocity(newVelocity);
            //TODO: set cognitive weight to zero.
        }
    }
    particle.setPosition(newPosition, _objectiveFunction);
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateVelocity(Particle<T, Fun> &particle)
{
    //std::cout << "Update VELOCITY called" << std::endl;

    std::vector<T> newVelocity(particle.getVelocity());
    const std::vector<T> &position = particle.getPosition();
    const std::vector<T> &pBest = particle.getBestPosition();

    for (size_t i = 0; i < _dimension; ++i)
    {
        T r1 = _dis[i](_rng);
        T r2 = _dis[i](_rng);
        newVelocity[i] = _inertiaWeight * newVelocity[i] +
                        _cognitiveWeight * r1 * (pBest[i] - position[i]) +
                        _socialWeight * r2 * (_gBestPos[i] - position[i]);
    }
    particle.setVelocity(newVelocity);
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePBestPos(Particle<T, Fun> &particle)
{
    if (particle.getValue() < particle.getBestValue())
    {
        particle.setBestPosition(particle.getPosition());
        particle.setBestValue(particle.getValue());
    }
    //std::cout << "Update PERSONAL BEST called" << std::endl;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePBestVal(Particle<T, Fun> &particle)
{
    particle.setBestValue(_objectiveFunction(particle.getBestPosition().data()));
}

template <typename T, typename Fun> void Swarm<T, Fun>::updateGBestPos()
{
    for (size_t i = 0; i < _numParticles; ++i)
    {
        T newBestValue = particles[i].getBestValue();
        if (newBestValue < _gBestVal)
        {
            std::cout << "Update GLOBAL BEST !!!!" << std::endl;
            std::cout << "Global best value update: " << newBestValue << "  vs  " << _gBestVal << std::endl;

            _gBestVal = newBestValue;
            _gBestPos = particles[i].getBestPosition();
        }
    }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::info() const
{
    std::cout << "\n╔═════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              Swarm Information                                  ║" << std::endl;
    std::cout << "╠═════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Number of Particles: " << std::setw(40) << _numParticles << "   ║" << std::endl;
    std::cout << "║ Dimension:           " << std::setw(40) << _dimension << "   ║" << std::endl;
    std::cout << "╠═════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Hyper-parameters:                                               ║" << std::endl;
    std::cout << "║   Inertia weight:    " << std::setw(40) << std::fixed << std::setprecision(4) << _inertiaWeight << "   ║" << std::endl;
    std::cout << "║   Cognitive weight:  " << std::setw(40) << _cognitiveWeight << "   ║" << std::endl;
    std::cout << "║   Social weight:     " << std::setw(40) << _socialWeight << "   ║" << std::endl;
    std::cout << "╠═════════════════════════════════════════════════════════════════╣" << std::endl;
    std::cout << "║ Global Best Value(init):" << std::setw(40) << std::scientific << std::setprecision(6) << _gBestVal << "║" << std::endl;
    std::cout << "╚═════════════════════════════════════════════════════════════════╝" << std::endl;
}

template <typename T, typename Fun>
size_t Swarm<T, Fun>::getNumParticles() const
{
    return _numParticles;
}

template <typename T, typename Fun> size_t Swarm<T, Fun>::getDimension() const
{
    return _dimension;
}

template <typename T, typename Fun>
Fun Swarm<T, Fun>::getObjectiveFunction() const
{
    return _objectiveFunction;
}

template <typename T, typename Fun> T Swarm<T, Fun>::getInertiaWeight() const
{
    return _inertiaWeight;
}

template <typename T, typename Fun>
T Swarm<T, Fun>::getCognitiveWeight() const
{
    return _cognitiveWeight;
}

template <typename T, typename Fun> T Swarm<T, Fun>::getSocialWeight() const
{
    return _socialWeight;
}

template <typename T, typename Fun>
std::vector<T> Swarm<T, Fun>::getPosition(Particle<T, Fun> &particle) const
{
    return particle.getPosition();
}

template <typename T, typename Fun>
std::vector<T> Swarm<T, Fun>::getVelocity(Particle<T, Fun> &particle) const
{
    return particle.getVelocity();
}

template <typename T, typename Fun>
std::vector<T> Swarm<T, Fun>::getGlobalBestPosition() const
{
    return _gBestPos;
}

template <typename T, typename Fun>
double Swarm<T, Fun>::getGlobalBestValue() const
{
    return _gBestVal;
}

// Setters
template <typename T, typename Fun>
void Swarm<T, Fun>::setNumParticles(const size_t &numParticles)
{
    _numParticles = numParticles;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setDimension(const size_t &dimension)
{
    _dimension = dimension;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setObjectiveFunction(const Fun &objectiveFunction)
{
    _objectiveFunction = objectiveFunction;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setInertiaWeight(const T &inertiaWeight)
{
    _inertiaWeight = inertiaWeight;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setCognitiveWeight(const T &cognitiveWeight)
{
    _cognitiveWeight = cognitiveWeight;
}

template <typename T, typename Fun>
void Swarm<T, Fun>::setSocialWeight(const T &socialWeight)
{
    _socialWeight = socialWeight;
}

// Memory management
template <typename T, typename Fun> void Swarm<T, Fun>::deallocateMemory()
{
    particles.clear();
}

// Explicit instantiation
template class Swarm<double, std::function<double(double*)>>;
