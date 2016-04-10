/**
 * Random number generator given a range.
 *
 * @author Ted Li
 */

#include <chrono>
#include <random>

typedef struct Rand {
private:
  std::mt19937_64 rng;
  uint64_t timeSeed;
  std::seed_seq *ss;
  std::uniform_real_distribution<double> unif;

public:
  Rand() {
    std::mt19937_64 rng;
    timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    ss = new std::seed_seq{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(*ss);
    unif = std::uniform_real_distribution<double>(-0.5, 0.5);
  }

  float rand(float min, float max) {
    min = std::min(min, max);
    max = std::max(min, max);
    float scaled = unif(rng) * (max - min);
    float shifted = scaled + min - (-0.5) * (max - min);
    return shifted;
  }
} Rand;
