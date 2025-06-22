#include "optimization/Swarm.hpp"
#include "core/Constants.hpp"

#include <cstddef>
#include <memory>
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
    _gBestPos.resize(_dimension, std::numeric_limits<double>::quiet_NaN());

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

}

template <typename T, typename Fun>
void Swarm<T, Fun>::updatePosition(Particle<T, Fun> &particle)
{
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
        }
        else if (newPosition[i] > _upperBounds[i])
        {
            newPosition[i] = _upperBounds[i];

            std::vector<T> newVelocity = particle.getVelocity();
            newVelocity[i] = 0.0;
            particle.setVelocity(newVelocity);
        }
        else if (std::isnan(newPosition[i]) || std::isinf(newPosition[i]))
        {
            // Reset to a random value within bounds
            newPosition[i] = _lowerBounds[i] + (_upperBounds[i] - _lowerBounds[i]) * ((double)rand() / RAND_MAX);
            std::vector<T> newVelocity = particle.getVelocity();
            newVelocity[i] = 0.0;
            particle.setVelocity(newVelocity);
        }
    }
    particle.setPosition(newPosition, _objectiveFunction);
}

template <typename T, typename Fun>
void Swarm<T, Fun>::updateVelocity(Particle<T, Fun> &particle)
{

    std::vector<T> newVelocity(particle.getVelocity());
    const std::vector<T> &position = particle.getPosition();
    const std::vector<T> &pBest = particle.getBestPosition();

    for (size_t i = 0; i < _dimension; ++i)
    {
        T r1 = _dis[i](_rng);
        T r2 = _dis[i](_rng);

        if (position[i] + newVelocity[i] < _lowerBounds[i] || position[i] + newVelocity[i] > _upperBounds[i])
        {
            newVelocity[i] = 0.0;
        } else {

            newVelocity[i] = _inertiaWeight * newVelocity[i] +
                    _cognitiveWeight * r1 * (pBest[i] - position[i]) +
                    _socialWeight * r2 * (_gBestPos[i] - position[i]);
        }
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
            _gBestVal = newBestValue;
            _gBestPos = particles[i].getBestPosition();
        }
    }
}

template <typename T, typename Fun>
void Swarm<T, Fun>::info() const
{
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                      PSO ALGORITHM CONFIGURATION                     ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;

    // Swarm configuration
    std::cout << "║ SWARM CONFIGURATION:                                                 ║" << std::endl;
    std::cout << "║   • Population Size:           " << std::setw(12) << _numParticles << " particles" << std::setw(19) << " ║" << std::endl;
    std::cout << "║   • Problem Dimension:         " << std::setw(12) << _dimension << " parameters" << std::setw(18) << " ║" << std::endl;
    std::cout << "║   • Search Space:              " << std::setw(20) << "6D continuous" << std::setw(21) << " ║" << std::endl;

    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;

    // Algorithm parameters
    std::cout << "║ ALGORITHM PARAMETERS:                                                ║" << std::endl;
    std::cout << "║   • Inertia Weight (w):        " << std::setw(12) << std::fixed << std::setprecision(12)
              << _inertiaWeight << std::setw(27) << " ║" << std::endl;
    std::cout << "║   • Cognitive Weight (c₁):     " << std::setw(12) << _cognitiveWeight << std::setw(27) << " ║" << std::endl;
    std::cout << "║   • Social Weight (c₂):        " << std::setw(12) << _socialWeight << std::setw(27) << " ║" << std::endl;
    std::cout << "║   • Parameter Update:          " << std::setw(20) << "Dynamic (linear)" << std::setw(21) << " ║" << std::endl;

    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;

    // Problem specifics
    std::cout << "║ ORBITAL TRANSFER PROBLEM:                                            ║" << std::endl;
    std::cout << "║   • Initial Orbit:             " << std::setw(20) << "R₁ = 1.000 DU" << std::setw(23) << " ║" << std::endl;
    std::cout << "║   • Target Orbit:              " << std::setw(20) << "R₂ = 1.500 DU" << std::setw(23) << " ║" << std::endl;
    std::cout << "║   • Inclination Change:        " << std::setw(12) << std::setprecision(1)
              << std::abs(constant::I1-constant::I2) << " rad" << std::setw(25) << " ║" << std::endl;
              std::cout << "║   • Transfer Type:             " << std::setw(20)
                        << (constant::I1 == constant::I2 ? "Coplanar" : "Non-Coplanar")
                        << std::setw(21) << " ║" << std::endl;
    std::cout << "╠══════════════════════════════════════════════════════════════════════╣" << std::endl;

    // Initial status
    std::cout << "║ INITIALIZATION STATUS:                                               ║" << std::endl;
    if (_gBestVal == std::numeric_limits<T>::max()) {
        std::cout << "║   • Initial Best Value:        " << std::setw(20) << "Not evaluated" << std::setw(21) << " ║" << std::endl;
    } else {
        std::cout << "║   • Initial Best Value:        " << std::setw(12) << std::scientific << std::setprecision(3)
                  << _gBestVal << " km/s" << std::setw(7) << " ║" << std::endl;
    }
    std::cout << "║   • Swarm Status:              " << std::setw(20) << "✓ READY" << std::setw(23) << " ║" << std::endl;

    std::cout << "╚══════════════════════════════════════════════════════════════════════╝" << std::endl;

    std::cout << "\n Starting PSO optimization for orbital transfer problem..." << std::endl;
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

template <typename T, typename Fun> void Swarm<T, Fun>::deallocateMemory()
{
    particles.clear();
}

template class Swarm<double, std::function<double(double*)>>;
