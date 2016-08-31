#include <cstdint>
#include <vector>


#include "arithmetic_codec.h"
#include "gtest/gtest.h"

static std::vector<uint8_t> GenerateSymbols(const std::vector<uint32_t> &freqs, size_t num_symbols) {
  assert(freqs.size() < 256);
  std::vector<uint8_t> symbols;
  symbols.reserve(num_symbols);
  srand(time(NULL));
  for (size_t i = 0; i < num_symbols; ++i) {
    uint32_t M = std::accumulate(freqs.begin(), freqs.end(), 0);
    int r = rand() % M;
    uint8_t symbol = 0;
    int freq = 0;
    for (auto f : freqs) {
      freq += f;
      if (r < freq) {
        break;
      }
      symbol++;
    }
    assert(symbol < freqs.size());
    symbols.push_back(symbol);
  }

  return std::move(symbols);
}


TEST(arith_codec, EncodeBits) {

  std::vector<uint8_t> data;
  data.resize(4096);

  unsigned max_bytes = 4096;

  entropy::Arithmetic_Codec ace(max_bytes, data.data());
  entropy::Adaptive_Data_Model model(257);

  ace.start_encoder();
  std::vector<uint8_t> symbols;


}
