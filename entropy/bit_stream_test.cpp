#include <cstdint>
#include "bit_stream.h"
#include "gtest/gtest.h"

TEST(bit_stream, CanWriteBytes) {
  uint32_t x;
  entropy::BitWriter w(reinterpret_cast<uint8_t*>(&x));
  w.WriteBits(1, 8);
  w.WriteBits(0, 8);
  w.WriteBits(0, 8);
  w.WriteBits(0, 8);
  EXPECT_EQ(x,1);
  EXPECT_EQ(w.BytesWritten(), 4);
  EXPECT_EQ(w.BitsWritten(), 32);

  w = entropy::BitWriter(reinterpret_cast<unsigned char*>(&x));
  w.WriteBits(0xbeef, 16);
  w.WriteBits(0xdead, 16);
  EXPECT_EQ(x, 0xdeadbeef);
  EXPECT_EQ(w.BytesWritten(), 4);
  EXPECT_EQ(w.BitsWritten(), 32);

}

TEST(Bits, CanWriteBits) {
  int32_t x;
  entropy::BitWriter w(reinterpret_cast<unsigned char*>(&x));
  for (int i = 0; i < 32; ++i) {
    w.WriteBit(1);
  }

  EXPECT_EQ(x, -1);
  EXPECT_EQ(w.BytesWritten(), 4);
  EXPECT_EQ(w.BitsWritten(), 32);

  w = entropy::BitWriter(reinterpret_cast<unsigned char*>(&x));
  for (int i = 0; i < 32; ++i) {
    if (i % 2 == 0) {
      w.WriteBit(0);
    } else {
      w.WriteBit(1);
    }
  }

  EXPECT_EQ(0xAAAAAAAA, x);
  EXPECT_EQ(w.BytesWritten(), 4);
  EXPECT_EQ(w.BitsWritten(), 32);
}
