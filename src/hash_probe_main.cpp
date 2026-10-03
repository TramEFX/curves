
#include <cstdio>
#include "string_sim.h"
int main(){
  for (unsigned s : {1u, 7u, 12345u})
    for (unsigned i : {0u, 1u, 5u, 64u, 511u})
      std::printf("%u %u %.17g\n", s, i, curves::StringSim::hash_pm1(s, i));
  return 0;
}
