#include <iostream>
#include <fstream>
#include <bitset>
#include <chrono>
#include "decoder.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define BUFFER

static void LerpChannels(uint8_t a[3], uint8_t b[3], uint8_t out[3], int num, int div) {
  for (int i = 0; i < 3; ++i) {
    out[i] =
      (static_cast<int>(a[i]) * (div - num) + static_cast<int>(b[i]) * num)
      / div;
  }
  out[3] = 0xFF;
}


static void Decode565(uint16_t x, uint8_t out[4]) {
  uint32_t r = (x >> 11);
  r = (r << 3) | (r >> 2);

  uint32_t g = (x >> 5) & 0x3F;
  g = (g << 2) | (g >> 4);

  uint32_t b = x & 0x1F;
  b = (b << 3) | (b >> 2);

  assert(r < 256);
  assert(g < 256);
  assert(b < 256);

  out[0] = r;
  out[1] = g;
  out[2] = b;
  out[3] = 255;
}


inline int BlockAt(int x , int y, uint32_t width, uint32_t height) {
  return (y/4) * (width/4) + (x/4);
}



uint8_t InterpolationValueAt(int x, int y, std::vector<LogicalDXTBlock> &logical_blocks,
                             uint32_t width, uint32_t height) {
  int block_idx = BlockAt(x, y, width, height);
  int pixel_idx = (y % 4) * 4 + (x % 4);
  return logical_blocks[block_idx].indices[pixel_idx];
}

LogicalDXTBlock PhysicalToLogical(const PhysicalDXTBlock &b) {
  LogicalDXTBlock out;

  Decode565(b.ep1, out.ep1);
  Decode565(b.ep2, out.ep2);

  memcpy(out.palette[0], out.ep1, 4);
  memcpy(out.palette[1], out.ep2, 4);

  if (b.ep1 <= b.ep2) {
    LerpChannels(out.ep1, out.ep2, out.palette[2], 1, 2);
    memset(out.palette[3], 0, 4);
  }
  else {
    LerpChannels(out.ep1, out.ep2, out.palette[2], 1, 3);
    LerpChannels(out.ep1, out.ep2, out.palette[3], 2, 3);
  }

  uint8_t const* bytes = reinterpret_cast<const uint8_t *>(&b.interp);
  for (int k = 0; k < 4; ++k) {
    uint8_t packed = bytes[k];

    out.indices[0 + 4 * k] = packed & 0x3;
    out.indices[1 + 4 * k] = (packed >> 2) & 0x3;
    out.indices[2 + 4 * k] = (packed >> 4) & 0x3;
    out.indices[3 + 4 * k] = (packed >> 6) & 0x3;
  }

  return out;
}


static std::vector<LogicalDXTBlock>
  PhysicalToLogicalBlocks(const PhysicalDXTBlock *blocks, size_t length) {
  std::vector<LogicalDXTBlock> out;
  out.reserve(length);

  for (size_t idx = 0; idx < length; idx++) {
    LogicalDXTBlock lb = PhysicalToLogical(blocks[idx]);
    out.push_back(lb);
  }

  return std::move(out);
}



void GetColorAt(int x, int y, uint8_t out[4], std::vector<LogicalDXTBlock> &logical_blocks,
                uint32_t width, uint32_t height) {
  const LogicalDXTBlock &b = logical_blocks[BlockAt(x, y, width, height)];
  uint8_t i = InterpolationValueAt(x, y, logical_blocks, width, height);
  out[0] = b.palette[i][0];
  out[1] = b.palette[i][1];
  out[2] = b.palette[i][2];
  out[3] = b.palette[i][3];
}

std::vector<uint8_t> DecompressedImage(std::vector<LogicalDXTBlock> &logical_blocks,
                                       uint32_t width, uint32_t height) {
  std::vector<uint8_t> result;

  for(int j = 0; j < height; j++) {
    for(int i = 0; i < width; i++) {
      uint8_t pixel[4];
      GetColorAt(i, j, pixel, logical_blocks, width, height);

      for(int c = 0; c < 3; c++){
	result.push_back(pixel[c]);
      }

    }
  }
  return std::move(result);
}

int main(int argc, char *argv []) {

  uint32_t width, height;
  width = std::stoi(argv[1], nullptr, 10);
  height = std::stoi(argv[2], nullptr, 10);
  
  std::string in_file(argv[3]);
  std::string out_dir(argv[4]);
  bool is_multi_thread = false;
  if(argc > 4 && argv[5][2] == 'm') {
    is_multi_thread = true;
  }


  uint32_t frame_number = 0;
  std::ifstream in_stream(in_file.c_str(), std::ios::binary);
  PhysicalDXTBlock *prev_dxt, *curr_dxt;
  PhysicalDXTBlock *temp_dxt;
  uint32_t num_blocks = (width/4 * height/4);


  MPTCDecodeInfo *decode_info;
  decode_info = (MPTCDecodeInfo *)(malloc(sizeof(MPTCDecodeInfo)));
  decode_info->is_start = true;

  std::vector<LogicalDXTBlock> logical_blocks; 
  std::vector<uint8_t> out_image;

  BufferStruct *ptr_buffer_struct = (BufferStruct*)malloc(sizeof(BufferStruct)); 
  InitBufferedDecode(4, ptr_buffer_struct, in_stream, num_blocks);

  assert(ptr_buffer_struct->ptr_decode_info != NULL);
  for(uint8_t idx = 0; idx < 4; idx++)
    assert(ptr_buffer_struct->buffered_dxts[idx] != NULL);

  while(1) {

   //GetFrameMultiThread(in_stream, prev_dxt, curr_dxt, decode_info);
   auto t1 = std::chrono::high_resolution_clock::now();
   GetBufferedFrame(ptr_buffer_struct, curr_dxt, in_stream);
   auto t2 = std::chrono::high_resolution_clock::now();
   std::chrono::duration<double> fp_ms = t2 - t1;
   std::cout << "Time:" << fp_ms.count() << std::endl;
   assert(curr_dxt != NULL);
   logical_blocks = PhysicalToLogicalBlocks(curr_dxt, num_blocks); 
   out_image = DecompressedImage(logical_blocks, width, height);
//#ifndef NDEBUG
/*   for(int i = 0; i < num_blocks; i++) {*/
     //std::bitset<32> x(curr_dxt[i].interp);
     ////std::cout<< "IDX: " << i << "  " <<"Ep1: " << curr_dxt[i].ep1 << "  " << "Ep2: " << curr_dxt[i].ep2 << "  " << "Interp: " << x << std::endl;
     //int a = 10 + 20;
   /*}*/
   frame_number++;
   std::string out_file = out_dir + "/" + std::to_string(frame_number) + ".png";
   stbi_write_png(out_file.c_str(), width, height, 3, out_image.data(), 3 * (width) );
//#endif
 }


}









