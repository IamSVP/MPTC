#include "dxt_image.h"

#include <iostream>
#include <cstdlib>
#include <cassert>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <functional>

#ifdef _MSC_VER
#include "win/dirent.h"
#else // _MSC_VER
#include <dirent.h>
#endif

#include <chrono>
#include "codec.h"
#include "stb_image_write.h"

template <typename T>
void WriteImageToFile(std::string out_filename, T img) {
  assert( (img->BitDepth() & 7) == 0);  
  stbi_write_png(out_filename.c_str(), img->Width(), img->Height(), img->kNumChannels, img->Width() * (img->BitDepth()/8) );

}


void ReconstructImage(std::vector<std::unique_ptr<MPTC::DXTImage> > &dxt_frames, // dxt_frames
                      int frame_num // frame number to be decoded in the vector of frames
                     ) {

  std::vector<int32_t> index_mask(dxt_frames[frame_num]->_index_mask.begin(), dxt_frames[frame_num]->_index_mask.end());
  for(size_t i = 1; i < index_mask.size(); i++) {
    index_mask[i] += index_mask[i-1];
  }
  std::vector<uint32_t> interpolation_data; 

  int prev_mask_idx = 0; 
  for(int physical_idx = 0; physical_idx < dxt_frames[frame_num]->_num_blocks; physical_idx++) {
    int curr_mask_idx = index_mask[physical_idx];
    int final_idx;
    if(prev_mask_idx == curr_mask_idx) {
      final_idx = physical_idx - curr_mask_idx;

      uint8_t x = std::get<0>(dxt_frames[frame_num]->_motion_indices[final_idx]);
      uint8_t y = std::get<1>(dxt_frames[frame_num]->_motion_indices[final_idx]);

      if( (x%4!=0) || (y%4!=0) ) { // if this conditon holds inter-pixel motion. fetch data from previous frame
        int32_t motion_x = static_cast<int32_t>(x - 64);
	int32_t motion_y = static_cast<int32_t>(y - 64);

        int32_t curr_block_x = physical_idx % dxt_frames[frame_num]->_blocks_width;
        int32_t curr_block_y = physical_idx / dxt_frames[frame_num]->_blocks_width;

	int32_t pix_x = 4 * curr_block_x + motion_x;
	int32_t pix_y = 4 * curr_block_y + motion_y;

	uint32_t interpolation = dxt_frames[frame_num-1]->Get4X4InterpolationBlock(
	                                           static_cast<uint32_t>(pix_x), 
	                                           static_cast<uint32_t>(pix_y));
	interpolation_data.push_back(interpolation);
      } 
      else if((x&0b10000000)!=0 && (y&0b10000000)!=0) { //Inter block motion, fetch data from previous frame

        x = (x & 0b01111111);
        y = (y & 0b01111111);
        int32_t motion_x = static_cast<int32_t>(x - 64)/4;
        int32_t motion_y = static_cast<int32_t>(y - 64)/4;

        int32_t curr_block_x = physical_idx % dxt_frames[frame_num]->_blocks_width;
        int32_t curr_block_y = physical_idx / dxt_frames[frame_num]->_blocks_width;

        int32_t ref_block_x = curr_block_x + motion_x;
        int32_t ref_block_y = curr_block_y + motion_y;

        int ref_physical_idx = ref_block_y * dxt_frames[frame_num]->_blocks_width + ref_block_x;
        interpolation_data.push_back(dxt_frames[frame_num - 1]->_physical_blocks[ref_physical_idx].interp);
      }
      else { //Intra block motion, fetch data from own frame

        int32_t motion_x = static_cast<int32_t>(x - 64)/4;
        int32_t motion_y = static_cast<int32_t>(y - 64)/4;

        int32_t curr_block_x = physical_idx % dxt_frames[frame_num]->_blocks_width;
        int32_t curr_block_y = physical_idx / dxt_frames[frame_num]->_blocks_width;

        int32_t ref_block_x = curr_block_x + motion_x;
        int32_t ref_block_y = curr_block_y + motion_y;

        int32_t ref_physical_idx = ref_block_y * dxt_frames[frame_num]->_blocks_width + ref_block_x;

        assert(ref_physical_idx < static_cast<int32_t>(interpolation_data.size()));
        interpolation_data.push_back(interpolation_data[ref_physical_idx]);

      }

    } // all sorts of motion indices
    else {
      final_idx = index_mask[physical_idx] - 1;
      interpolation_data.push_back(dxt_frames[frame_num]->_unique_palette[final_idx]);
    }

    prev_mask_idx = curr_mask_idx;
  } //End of for over all blocks
  for(size_t i = 0; i < interpolation_data.size(); i++) {
    if(interpolation_data[i] != dxt_frames[frame_num]->_physical_blocks[i].interp) {
      assert(false);
    }
      dxt_frames[frame_num]->_physical_blocks[i].interp = interpolation_data[i];
      dxt_frames[frame_num]->_logical_blocks[i] = MPTC::PhysicalToLogical(
	                                           dxt_frames[frame_num]->_physical_blocks[i]);
  }
  std::unique_ptr<MPTC::RGBImage> decoded_img = std::move(
                                                     dxt_frames[frame_num]->DecompressedImage());

    stbi_write_png("decomp.png", decoded_img->Width(), decoded_img->Height(),
                  3, decoded_img->Pack().data(), 3*decoded_img->Width());
}
#if 0
int main( int argc, char* argv[]) {

  if(argc < 6) {
    std::cout<< "All File arguments are required!!" << std::endl;
    exit(-1);
  }

  uint32_t total_frames = std::stoi(argv[1], nullptr, 10);
  uint32_t key_frame_interval = std::stoi(argv[2], nullptr, 10);
  std::string dir_path(argv[3]);
  uint32_t search_area = std::stoi(argv[4], nullptr, 10);
  int32_t vErrThreshold = std::stoi(argv[5], nullptr, 10);
  std::string out_file_path(argv[6], nullptr, 10);


#if 0
  std::string dir_path(argv[1]);
  uint32_t first_frame_idx = std::stoi(argv[2], nullptr, 10);
  uint32_t frame_count = std::stoi(argv[3], nullptr, 10);
  uint32_t search_area = std::stoi(argv[4], nullptr, 10);
  uint32_t pad_zero = std::stoi(argv[5], nullptr, 10);
  int32_t vErrThreshold = std::stoi(argv[6], nullptr, 10);  

  std::vector<std::string> file_paths;
  std::vector< std::unique_ptr<MPTC::DXTImage> > dxt_frames;

  std::stringstream ss;
  std::fstream outfile;
  outfile.open("out.txt", std::ios::out);
  //MPTC::DXTImage dxt_img(img_path, true, 0);
  ss.str("");
  ss << std::setw(pad_zero) << std::setfill('0') << first_frame_idx;
  std::string frame_num_str = ss.str();
  std::string file_path = dir_path + "/" + frame_num_str+ ".png";
  file_paths.push_back(file_path);

  MPTC::DXTImage::SetPattern(static_cast<int32_t>(search_area));

  std::unique_ptr<MPTC::DXTImage> dxt_img(new MPTC::DXTImage(file_path, true, 0, vErrThreshold));
  std::unique_ptr<MPTC::DXTImage> null_dxt(nullptr);
  dxt_frames.push_back(std::move(dxt_img));

  std::cout << "Frame Number:" << 0 << std::endl;
  std::cout << "Before PSNR: " << dxt_frames[0]->PSNR() << std::endl;
  dxt_frames[0]->Reencode(null_dxt, 0);
  std::cout << "After PSNR: " << dxt_frames[0]->PSNR() << std::endl;
  std::cout << "Intra block motion size:" << dxt_frames[0]->_motion_indices.size() << std::endl;



  std::vector<uint8_t> palette = std::move(dxt_frames[0]->Get8BitPalette());

  std::vector<uint64_t> count_palette(256, 0);
  std::vector<uint64_t> count_intra(256, 0);
  std::vector<uint64_t> total_counts(256,0);

  for(auto a : dxt_frames[0]->_intra_motion_indices) {
    count_intra[std::get<0>(a)]++;
    count_intra[std::get<1>(a)]++;
    total_counts[std::get<0>(a)]++;
    total_counts[std::get<1>(a)]++;
  }

  uint64_t Total = std::accumulate(count_intra.begin(), count_intra.end(), 0U);
  double entropy = 0.0;
  for( auto e : count_intra ) {

    if(e!=0) {
      double p = static_cast<double>(e)/static_cast<double>(Total);
      entropy += (-1.0 * p * log2(p));
    }
  }
  
  std::cout << "Total:" << Total << std::endl;
  std::cout << "Entropy:" << entropy << std::endl;
  //Entropy encode motion indices 
  
  //*****************MAX BYTES *******************
  uint32_t max_bytes = 180000;
  std::vector<uint8_t> compressed_data(max_bytes, 0);

  entropy::Arithmetic_Codec ace(max_bytes, compressed_data.data());
  entropy::Adaptive_Data_Model model(257);
  ace.start_encoder();

  for(auto a : dxt_frames[0]->_motion_indices) {
    ace.encode(std::get<0>(a), model);
    ace.encode(std::get<1>(a), model);
  }

  ace.stop_encoder();
  std::cout << "Compressed motion index bytes:" << ace.get_num_bytes() << std::endl;

  // Entropy encode index mask
  std::vector<uint8_t> compressed_mask(max_bytes, 0);
  entropy::Arithmetic_Codec ace_mask(max_bytes, compressed_mask.data());
  entropy::Adaptive_Bit_Model mask_bit_model;
  ace_mask.start_encoder();
  for(auto a : dxt_frames[0]->_index_mask) {
    ace_mask.encode(a, mask_bit_model);
  }
  ace_mask.stop_encoder();
  std::cout << "Compressed Mask bytes:" << ace_mask.get_num_bytes() << std::endl;

#if 0
  //Entropy Decode
  entropy::Arithmetic_Codec ade(max_bytes, compressed_data.data());
  entropy::Adaptive_Data_Model decode_model(257);
  ade.start_decoder();
  std::vector<std::tuple<uint8_t, uint8_t> > decoded_symbols;

  for(int i = 0; i < dxt_frames[0]->_motion_indices.size(); i++) {
    uint8_t sym1 = ade.decode(decode_model);
    uint8_t sym2 = ade.decode(decode_model);
    decoded_symbols.push_back(std::make_tuple(sym1, sym2));
#ifdef NDEBUG
    auto a = dxt_frames[0]->_motion_indices[i];
    std::cout << sym1 << "-" << std::get<0>(a) << std::endl;
    std::cout << sym2 << "-" << std::get<1>(a) << std::endl;
#endif
    assert(dxt_frames[0]->_motion_indices[i] == decoded_symbols[i]);
  }
  ade.stop_decoder();

  //Entropy decode mask bits
  entropy::Arithmetic_Codec ade_mask(max_bytes,compressed_mask.data());
  entropy::Adaptive_Bit_Model decode_mask_bit_model;
  ade_mask.start_decoder();
  std::vector<uint8_t> decoded_mask;

  for(int i = 0; i < dxt_frames[0]->_motion_indices.size(); i++) {
    uint8_t sym = ade_mask.decode(decode_mask_bit_model);
    decoded_mask.push_back(sym);
#ifndef NDEBUG
    auto a = dxt_frames[0]->_index_mask[i];
    std::cout << static_cast<int>(sym) << " -- " << static_cast<int>(a) << std::endl;
#endif 
    assert(sym == dxt_frames[0]->_index_mask[i]);
  }

  ade_mask.stop_decoder();
#endif    
  uint32_t total_bits =  ace.get_num_bytes() * 8 + dxt_frames[0]->_unique_palette.size() * 32 + ace_mask.get_num_bytes() * 8;
  float total_bytes = static_cast<float>(total_bits)/8;
  outfile << total_bytes+10 << "\t" << dxt_frames[0]->PSNR() << std::endl;
  std::cout << "*****Total bytes****:" << total_bytes << std::endl;
  float bpp = static_cast<float>(total_bits)/(dxt_frames[0]->_width * dxt_frames[0]->_height);
  std::cout << "BPP:" << bpp << "\t" << dxt_frames[0]->PSNR() <<  std::endl;
  
  std::unique_ptr<MPTC::RGBAImage> ep1 = std::move(dxt_frames[0]->EndpointOneImage());
  std::vector<uint8_t> ep1_vector = std::move(ep1->Pack());

  stbi_write_png("ep1.png", ep1->Width(), ep1->Height(),
                  4, ep1->Pack().data(), 4 * ep1->Width());


  std::unique_ptr<MPTC::RGBAImage> ep2 = std::move(dxt_frames[0]->EndpointTwoImage());
  std::vector<uint8_t> ep2_vector = std::move(ep2->Pack());

  stbi_write_png("ep2.png", ep2->Width(), ep2->Height(),
                  4, ep2->Pack().data(), 4 * ep2->Width());
 
 
  std::vector<uint32_t> ep_diff;
  int max_diff = std::numeric_limits<int>::min();
  int min_diff = std::numeric_limits<int>::max();

  std::vector<uint32_t> count_ep(512, 0);
  for(size_t ep_idx = 0; ep_idx < ep1_vector.size(); ep_idx++) {

    if(ep_idx % 4 == 3) continue;
    int diff = static_cast<int>(ep1_vector[ep_idx]) - static_cast<int>(ep2_vector[ep_idx]);
    if(diff > max_diff) max_diff = diff;
    if(diff < min_diff) min_diff = diff;
    ep_diff.push_back(static_cast<uint32_t>(diff + 255));
    count_ep[ep_diff[ep_diff.size() - 1]]++;

  }
  uint64_t Total_ep = std::accumulate(count_ep.begin(), count_ep.end(), 0U);
  double entropy_ep = 0.0;
  for( auto e : count_ep ) {

    if(e!=0) {
      double p = static_cast<double>(e)/static_cast<double>(Total_ep);
      entropy_ep += (-1.0 * p * log2(p));
    }
  }
  // Entropy encode endpoint
  std::vector<uint8_t> compressed_ep(max_bytes, 0);
  entropy::Arithmetic_Codec ace_ep(max_bytes, compressed_ep.data());
  entropy::Adaptive_Data_Model ep_model;
  ace_ep.start_encoder();
  for(auto a :ep_diff) {
    ace_ep.encode(a, mask_bit_model);
  }
  ace_ep.stop_encoder();
  std::cout << "----EndPoint compressed----:" << ace_ep.get_num_bytes() << std::endl;

  std::cout << "Total end point:" << Total_ep << std::endl;
  std::cout << "Entropy end point:" << entropy_ep << std::endl;
 
  ReconstructImage(dxt_frames, 0);
  std::cout << "PSNR after decompression: " << dxt_frames[0]->PSNR() << std::endl;

  for(uint32_t i = first_frame_idx + 1; i <= first_frame_idx + frame_count; i++) {
    ss.str("");
    ss << std::setw(pad_zero) << std::setfill('0') << i;
    std::string frame_num_str = ss.str();
    std::string file_path = dir_path + "/" + frame_num_str+ ".png";
    std::unique_ptr<MPTC::DXTImage> dxt_img1(new MPTC::DXTImage(file_path, false, search_area, vErrThreshold));

    dxt_frames.push_back(std::move(dxt_img1));
    file_paths.push_back(file_path);
  }

  double combined_bpp = bpp;
  for(size_t i = 1; i < dxt_frames.size(); i++) {
  //*****************MAX BYTES *******************

    std::cout << std::endl << std::endl;
    std::cout << "Frame Number:" << i << std::endl;
    std::cout << "Before PSNR:" << dxt_frames[i]->PSNR() << std::endl;

    dxt_frames[i]->Reencode(dxt_frames[i-1], -1);

    std::cout << "After PSNR:" << dxt_frames[i]->PSNR() << std::endl;
    std::cout << "Total unique indices:" << dxt_frames[i]->_unique_palette.size()<< std::endl;
    std::cout << "Intra block motion size:" << dxt_frames[i]->_intra_motion_indices.size()<<std::endl;
    std::cout << "Inter block motion size:" << dxt_frames[i]->_inter_block_motion_indices.size()  << std::endl;
    std::cout << "Inter pixel motion size:" << dxt_frames[i]->_inter_pixel_motion_indices.size() << std::endl;

    uint32_t max_bytes_inter = 180000;
    std::vector<uint8_t> compressed_data_inter(max_bytes_inter, 0);

    entropy::Arithmetic_Codec ace_inter(max_bytes_inter, compressed_data_inter.data());
    entropy::Adaptive_Data_Model model_inter(257);
    ace_inter.start_encoder();

    for(auto a : dxt_frames[i]->_motion_indices) {
      ace_inter.encode(std::get<0>(a), model_inter);
      ace_inter.encode(std::get<1>(a), model_inter);
    }

    ace_inter.stop_encoder();

  // Entropy encode index mask
    std::vector<uint8_t> compressed_mask_inter(max_bytes_inter, 0);
    entropy::Arithmetic_Codec ace_mask_inter(max_bytes_inter, compressed_mask_inter.data());
    entropy::Adaptive_Bit_Model mask_bit_model_inter;
    ace_mask_inter.start_encoder();
    for(auto a : dxt_frames[i]->_index_mask) {
      ace_mask_inter.encode(a, mask_bit_model_inter);
    }
    ace_mask_inter.stop_encoder();
  //Entropy Decode
  entropy::Arithmetic_Codec ade(max_bytes, compressed_data_inter.data());
  entropy::Adaptive_Data_Model decode_model(257);
  ade.start_decoder();
  std::vector<std::tuple<uint8_t, uint8_t> > decoded_symbols;

  for(int ii = 0; ii < dxt_frames[i]->_motion_indices.size(); ii++) {
    uint8_t sym1 = ade.decode(decode_model);
    uint8_t sym2 = ade.decode(decode_model);
    decoded_symbols.push_back(std::make_tuple(sym1, sym2));
#if 0
    auto a = dxt_frames[]->_motion_indices[i];
    std::cout << sym1 << "-" << std::get<0>(a) << std::endl;
    std::cout << sym2 << "-" << std::get<1>(a) << std::endl;
#endif
    assert(dxt_frames[i]->_motion_indices[ii] == decoded_symbols[ii]);
  }
  ade.stop_decoder();

  //Entropy decode mask bits
  entropy::Arithmetic_Codec ade_mask(max_bytes,compressed_mask_inter.data());
  entropy::Adaptive_Bit_Model decode_mask_bit_model;
  ade_mask.start_decoder();
  std::vector<uint8_t> decoded_mask;

  for(int ii = 0; ii < dxt_frames[i]->_index_mask.size(); ii++) {
    uint8_t sym = ade_mask.decode(decode_mask_bit_model);
    decoded_mask.push_back(sym);
#if 0
    auto a = dxt_frames[0]->_index_mask[i];
    std::cout << static_cast<int>(sym) << " -- " << static_cast<int>(a) << std::endl;
#endif 
    assert(sym == dxt_frames[i]->_index_mask[ii]);
  }

  ade_mask.stop_decoder();

    std::vector<uint64_t> counts(256,0);

    uint8_t max = std::numeric_limits<uint8_t>::min();
    uint8_t min = std::numeric_limits<uint8_t>::max();

    for(auto a : dxt_frames[i]->_motion_indices) {
      counts[std::get<0>(a)]++;
      counts[std::get<1>(a)]++;
      total_counts[std::get<0>(a)]++;
      total_counts[std::get<1>(a)]++;

      max = std::max(std::max(max, std::get<0>(a)), std::get<1>(a));
      min = std::min(std::min(min, std::get<0>(a)), std::get<1>(a));
   
    }

    Total = std::accumulate(counts.begin(), counts.end(), 0U);
    entropy = 0.0;
    for( auto e : counts ) {
      if(e!=0) {
        double p = static_cast<double>(e)/static_cast<double>(Total);
        entropy += (-1.0 * p * log2(p));
      }
    }

    std::cout << "Total:" << Total << std::endl;
    std::cout << "Entropy:" << entropy << std::endl;

    total_bits =  ace_inter.get_num_bytes() * 8 + 1000 + dxt_frames[0]->_unique_palette.size() * 32 + ace_mask_inter.get_num_bytes() * 8;
    total_bytes = static_cast<float>(total_bits)/8;

    std::cout << "Compressed motion index bytes:" << ace_inter.get_num_bytes() << std::endl;

    std::cout << "Compressed Mask bytes:" << ace_mask_inter.get_num_bytes() << std::endl;
    std::cout << "Total bytes:" << total_bytes << std::endl;
    bpp = static_cast<float>(total_bits)/(dxt_frames[0]->_width * dxt_frames[0]->_height);
    std::cout << "BPP:" << bpp << std::endl;
    combined_bpp += bpp;

  }
  std::cout << std::endl << std::endl;
  std::cout << "Combined BPP:" << combined_bpp << std::endl;
#endif
  return 0;
}
#endif

int main(int argc, char *argv[]) {
  // Have to change this to only encoder
  if(argc < 7) {
    std::cout<< "All File arguments are required!!" << std::endl;
    exit(-1);
  }
  uint32_t search_area = std::stoi(argv[1], nullptr, 10);
  int32_t intra_interval = std::stoi(argv[2], nullptr, 10);
  uint32_t unique_interval = std::stoi(argv[3], nullptr, 10);
  int32_t vErrThreshold = std::stoi(argv[4], nullptr, 10);
  uint32_t thread_count = std::stoi(argv[5], nullptr, 10);
  std::string dir_path(argv[6]);
  std::string out_file_path(argv[7]);
  std::string out_dir(argv[8]);
  std::string ep_dir;
  if(argc > 7)
   ep_dir = std::string(argv[9]);
  
  //MPTC::CompressPNGStream(dir_path, out_file_path, search_area, vErrThreshold, interval, ep_dir);
  
  //MPTC::DecompressMPTCStream(out_file_path, out_dir, interval);
  /*MPTC::CompressMultiUnique(dir_path, out_file_path, search_area, vErrThreshold, */
                            /*intra_interval, unique_interval, ep_dir);*/
  //MPTC::DecompressMultiUnique(out_file_path, out_dir, ep_dir);
  //
  //
  uint32_t wavelet_block_sz;
  auto t1 = std::chrono::high_resolution_clock::now();

  MPTC::ThreadedCompressMultiUnique(dir_path,
				    out_file_path,
				    search_area,
				    vErrThreshold,
				    intra_interval,
				    unique_interval,
				    wavelet_block_sz,
				    ep_dir,
				    thread_count);
   auto t2 = std::chrono::high_resolution_clock::now();
   std::chrono::duration<double> fp_ms = t2 - t1;
   std::cout << "Time:" << fp_ms.count() << std::endl;

  //MPTC::DecompressMultiUnique(out_file_path, out_dir, ep_dir);
 
  return 0;
}
