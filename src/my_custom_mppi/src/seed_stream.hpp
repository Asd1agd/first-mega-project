#pragma once
#include <array>
#include <cstdint>

// Reproduce the CPU lab caller's default_rng(seed).integers(0, 2**63).
// The rollout kernel itself uses std::mt19937_64 and std::normal_distribution.
class SeedStream
{
  using Wide = __uint128_t;
  Wide state_{}, increment_{};
  static constexpr Wide multiplier = (Wide(2549297995355413924ULL) << 64) | 4865540595714422341ULL;
  void advance() {state_ = state_ * multiplier + increment_;}

public:
  void seed(uint64_t value)
  {
    std::array<uint32_t, 4> pool{};
    uint32_t hash = 0x43b0d7e5U;
    auto mix_hash = [&](uint32_t x) {
        x ^= hash;
        hash *= 0x931e8875U;
        x *= hash;
        return x ^ (x >> 16);
      };
    for (unsigned i = 0; i < 4; ++i) {
      pool[i] = mix_hash(i < 2 ? uint32_t(value >> (32 * i)) : 0);
    }
    for (unsigned src = 0; src < 4; ++src) {
      for (unsigned dst = 0; dst < 4; ++dst) {
        if (src == dst) {continue;}
        uint32_t z = 0xca01f9ddU * pool[dst] - 0x4973f715U * mix_hash(pool[src]);
        pool[dst] = z ^ (z >> 16);
      }
    }
    hash = 0x8b51f9ddU;
    std::array<uint64_t, 4> words{};
    for (unsigned i = 0; i < 8; ++i) {
      uint32_t z = pool[i % 4] ^ hash;
      hash *= 0x58f38dedU;
      z *= hash;
      words[i / 2] |= uint64_t(z ^ (z >> 16)) << (32 * (i % 2));
    }
    Wide initial = (Wide(words[0]) << 64) | words[1];
    increment_ = (((Wide(words[2]) << 64) | words[3]) << 1) | 1;
    state_ = increment_ + initial;
    advance();
  }
  uint64_t operator()()
  {
    advance();
    uint64_t x = uint64_t(state_ >> 64) ^ uint64_t(state_);
    unsigned rot = unsigned(state_ >> 122);
    return ((x >> rot) | (x << ((-rot) & 63))) >> 1;
  }
};
