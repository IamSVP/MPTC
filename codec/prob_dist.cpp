#include "dxt_image.h"

#include <cmath>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <string>
#include <tuple>
#include <algorithm>
#include <functional>
#include <dirent.h>

#include "codec.h"
#include "bit_stream.h"
#include "dxt_image.h"
#include "arithmetic_codec.h"
#include "stb_image_write.h"

void AnalyzeBlocks(const std::string in_file, uint32_t search_area, int32_t vErrThreshold, std::vector<std::tuple<uint8_t, uint8_t> > &motion_indices, uint8_t search_space) {
  // Declare all pointers to DXTImages
  std::unique_ptr<MPTC::DXTImage> frame(new MPTC::DXTImage(in_file, true, search_area, vErrThreshold));
  std::unique_ptr<MPTC::DXTImage> null_dxt(nullptr);

  uint32_t frame_height, frame_width;
  frame_width = frame->Width();
  frame_height = frame->Height();

  frame->ReencodeAndAnalyze(null_dxt, 0, motion_indices, search_space);
}

void AnalyzeMotionIndices(std::vector<std::tuple<uint8_t, uint8_t> > &motion_indices, uint8_t search_space) {
  uint64_t x_dist[256] = {0};
  uint64_t y_dist[256] = {0};

  uint64_t joint_dist[256] = {0};

  // calculate frequencies
  for (auto pair : motion_indices) {
    uint8_t dx = std::get<0>(pair);
    uint8_t dy = std::get<1>(pair);
    assert(dx < 256 && dy < 256);
    x_dist[dx]++;
    y_dist[dy]++;
    joint_dist[dx]++;
    joint_dist[dy]++;
  }

  // calculate frequency sum
  uint64_t x_sum = 0, y_sum = 0, joint_sum = 0;
  for (int i = 0; i < 256; i += 4) {
    x_sum += x_dist[i];
    y_sum += y_dist[i];
  }
  joint_sum = x_sum + y_sum;

  // calculate entropy
  double x_entropy = 0, y_entropy = 0, joint_entropy = 0;
  for (int i = 0; i < 256; i += 4) {
    double px = double(x_dist[i]) / x_sum;
    double py = double(y_dist[i]) / y_sum;
    if (px > 0) x_entropy += px * log2(px);
    if (py > 0) y_entropy += py * log2(py);
    double p = double(joint_dist[i]) / joint_sum;
    if (p > 0) joint_entropy += p * log2(p);
  }
  x_entropy = -x_entropy;
  y_entropy = -y_entropy;
  joint_entropy = -joint_entropy;

  std::cout << "search space " << uint32_t(search_space) << std::endl; 
  // std::cout << "x entropy " << x_entropy << std::endl;
  // std::cout << "y entropy " << y_entropy << std::endl;
  // std::cout << "joint entropy " << joint_entropy << std::endl;
  std::cout << "entropy x #indices " << ((joint_entropy * motion_indices.size()) / 8) << std::endl << std::endl;
}

int main(int argc, char *argv[]) {
  if(argc < 4) {
    std::cout<< "All File arguments are required!!" << std::endl;
    exit(-1);
  }

  std::string in_file_path(argv[1]);
  uint32_t search_area = std::stoi(argv[2], nullptr, 10);
  int32_t vErrThreshold = std::stoi(argv[3], nullptr, 10);
  
  std::vector<std::tuple<uint8_t, uint8_t> > motion_indices;
  for (uint8_t search_space = 16; search_space <= 16; search_space *= 2) {
  // for (uint8_t search_space = 1; search_space <= 32; search_space *= 2) {
    AnalyzeBlocks(in_file_path, search_area, vErrThreshold, motion_indices, search_space);
    AnalyzeMotionIndices(motion_indices, search_space);
  }

  return 0;
}

// ./prob_dist ../../RawFrames/00330.png 64 50