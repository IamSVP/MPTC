#include "decoder.h"
#include "wavelet.h"

#include <iostream>
#include "stb_image_write.h"
#include "arithmetic_codec.h"


#include <fstream>
#include <tuple>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstring>
#include <atomic>
#include <typeinfo>
#include <cassert>
#include <limits>
#include <thread>
#include <functional>



static void ycocg667_to_rgb565(const int8_t *in, int8_t *out) {
  int8_t y = in[0];
  int8_t co = in[1];
  int8_t cg = in[2];

  assert(0 <= y && y < 64);
  assert(-31 <= co && co < 32);
  assert(-63 <= cg && cg < 64);

  int8_t t = y - (cg / 2);
  out[1] = cg + t;
  out[2] = (t - co) / 2;
  out[0] = out[2] + co;

  assert(0 <= out[0] && out[0] < 32);
  assert(0 <= out[1] && out[1] < 64);
  assert(0 <= out[2] && out[2] < 32);
}


void EntropyDecode(uint8_t *compressed_data, 
                   uint8_t *out_symbols, 
		   uint32_t compressed_size,
		   uint32_t out_size,
		   bool is_bit_model) {

  entropy::Arithmetic_Codec arith_decoder(compressed_size + 100, compressed_data);
  if(!is_bit_model) {
    entropy::Adaptive_Data_Model model(257);
    arith_decoder.start_decoder();
    for(size_t sym_idx = 0; sym_idx < out_size; sym_idx++)
      out_symbols[sym_idx] = arith_decoder.decode(model);
    arith_decoder.stop_decoder();
  }
  else {
    entropy::Adaptive_Bit_Model model;
    arith_decoder.start_decoder();
    for(size_t sym_idx = 0; sym_idx < out_size; sym_idx++)
      out_symbols[sym_idx] = arith_decoder.decode(model);
    arith_decoder.stop_decoder();
  }
  return;
}
const uint32_t BlockSize = 64;
void IWavelet2D(uint8_t *in, int8_t *out, uint32_t width, uint32_t height) {

    for (size_t j = 0; j < height; j += BlockSize) {
      for (size_t i = 0; i < width; i += BlockSize) {
        std::vector<int16_t> block(BlockSize * BlockSize);

        // Populate block
        for (size_t y = 0; y < BlockSize; ++y) {
          for (size_t x = 0; x < BlockSize; ++x) {
            size_t local_idx = y * BlockSize + x;

            int16_t pixel = static_cast<int16_t>(in[(i + x)+ width*(j + y)] - 128);
            assert(static_cast<int64_t>(pixel) <= std::numeric_limits<int16_t>::max());
            assert(static_cast<int64_t>(pixel) >= std::numeric_limits<int16_t>::min());
            block[local_idx] = static_cast<int16_t>(pixel);
          }
        }

        // Do transform
        static const size_t kRowBytes = sizeof(int16_t) * BlockSize;
        size_t dim = 2;
        while (dim <= BlockSize) {
	  MPTC::InverseWavelet2D(block.data(), kRowBytes, block.data(), kRowBytes, dim);
          dim *= 2;
        }

        // Output to image...
        for (size_t y = 0; y < BlockSize; ++y) {
          for (size_t x = 0; x < BlockSize; ++x) {
            size_t local_idx = y * BlockSize + x;
            assert(static_cast<int8_t>(block[local_idx]) <= std::numeric_limits<int8_t>::max());
            assert(static_cast<int8_t>(block[local_idx]) >= std::numeric_limits<int8_t>::min());
            out[(i + x) + width*(j + y)] = static_cast<int8_t>(block[local_idx]);
          }
        }
      }
    }


}

void ReconstructEndpoints(MPTCDecodeInfo *decode_info, 
                          PhysicalDXTBlock *curr_frame, int ep_number) {

  uint32_t width = decode_info->frame_width/4;
  uint32_t height = decode_info->frame_height/4;
  uint32_t num_blocks = decode_info->num_blocks;

  if(ep_number == 1) {
    //endpoint 1
    IWavelet2D(decode_info->wav_ep1_Y, decode_info->ep1_Y, width, height);
    IWavelet2D(decode_info->wav_ep1_C, decode_info->ep1_Co, width, height);
    IWavelet2D(decode_info->wav_ep1_C + num_blocks, decode_info->ep1_Cg, width, height);

    for (size_t j = 0; j < height; ++j) {
      for (size_t i = 0; i < width; ++i) {
        uint32_t physical_idx = j * width + i;  
        auto pix1 = decode_info->ep1_Y[physical_idx];
        auto pix2 = decode_info->ep1_Co[physical_idx];
        auto pix3 = decode_info->ep1_Cg[physical_idx];

        assert(0 <= pix1 && pix1 < 64);
        assert(-31 <= pix2 && pix2 < 32);
        assert(-63 <= pix3 && pix3 < 64);

        int8_t ycocg[3] = {
            static_cast<int8_t>(pix1),
            static_cast<int8_t>(pix2),
            static_cast<int8_t>(pix3) };
        int8_t rgb[3];
        ycocg667_to_rgb565(ycocg, rgb);

        assert(0 <= rgb[0] && rgb[0] < 32);
        assert(0 <= rgb[1] && rgb[1] < 64);
        assert(0 <= rgb[2] && rgb[2] < 32);

        // Pack it in...
        uint16_t x = 0;
        x |= static_cast<uint16_t>(rgb[0]);
        x <<= 6;
        x |= static_cast<uint16_t>(rgb[1]);
        x <<= 5;
        x |= static_cast<uint16_t>(rgb[2]);
        curr_frame[physical_idx].ep1 = x;
      }
    }


  } else if(ep_number == 2) {
    // endpoint 2
    IWavelet2D(decode_info->wav_ep2_Y, decode_info->ep2_Y, width, height);
    IWavelet2D(decode_info->wav_ep2_C, decode_info->ep2_Co, width, height);
    IWavelet2D(decode_info->wav_ep2_C + num_blocks, decode_info->ep2_Cg, width, height);
    

    for(size_t j = 0; j < height; ++j) {
      for(size_t i = 0; i < width; ++i) {

        uint32_t physical_idx = j * width + i;  

        auto pix1 = decode_info->ep2_Y[physical_idx];
        auto pix2 = decode_info->ep2_Co[physical_idx];
        auto pix3 = decode_info->ep2_Cg[physical_idx];

        assert(0 <= pix1 && pix1 < 64);
        assert(-31 <= pix2 && pix2 < 32);
        assert(-63 <= pix3 && pix3 < 64);

        int8_t ycocg[3] = {
            static_cast<int8_t>(pix1),
            static_cast<int8_t>(pix2),
            static_cast<int8_t>(pix3) };
        int8_t rgb[3];

        ycocg667_to_rgb565(ycocg, rgb);

        assert(0 <= rgb[0] && rgb[0] < 32);
        assert(0 <= rgb[1] && rgb[1] < 64);
        assert(0 <= rgb[2] && rgb[2] < 32);

      // Pack it in...
        uint16_t x = 0;
        x |= static_cast<uint16_t>(rgb[0]);
        x <<= 6;
        x |= static_cast<uint16_t>(rgb[1]);
        x <<= 5;
        x |= static_cast<uint16_t>(rgb[2]);
        curr_frame[physical_idx].ep2 = x;
      }
    }
  }

        //Endpoint 2
}

void ReconstructDXTFrame(uint32_t *unique_indices,
                         uint32_t num_unique,
                         MPTCDecodeInfo *decode_info, 
                         PhysicalDXTBlock *prev_dxt, 
                         PhysicalDXTBlock *curr_dxt) {


  int32_t blocks_width = decode_info->frame_width/4;
  uint32_t curr_unique_idx = 0;
  uint8_t search_area = decode_info->search_area;
  for(int32_t physical_idx = 0; physical_idx < decode_info->num_blocks; physical_idx++) {
    uint8_t x = decode_info->motion_indices[2 * physical_idx];
    uint8_t y = decode_info->motion_indices[2 * physical_idx + 1];

    if(x == 255 && y == 255) {
      assert(curr_unique_idx < num_unique);
      assert( unique_indices != NULL && "Unique Indies pointer cannot be null");
      curr_dxt[physical_idx].interp = unique_indices[curr_unique_idx];
      curr_unique_idx++;
    }
    else if((x&0b10000000)!=0 && (y&0b10000000)!=0) { //Inter block motion, fetch data from previous frame
      x = (x & 0b01111111);
      y = (y & 0b01111111);
      int32_t motion_x = static_cast<int32_t>(x - search_area);
      int32_t motion_y = static_cast<int32_t>(y - search_area);

      int32_t curr_block_x = physical_idx % blocks_width;
      int32_t curr_block_y = physical_idx / blocks_width;

      int32_t ref_block_x = curr_block_x + motion_x;
      int32_t ref_block_y = curr_block_y + motion_y;

      int ref_physical_idx = ref_block_y * blocks_width + ref_block_x;
      assert(prev_dxt != NULL && "Prev Frame cannot be NULL!!");
      curr_dxt[physical_idx].interp = prev_dxt[ref_physical_idx].interp; 
    }
    else { //Intra block motion, fetch data from own frame
        int32_t motion_x = static_cast<int32_t>(x - search_area);
        int32_t motion_y = static_cast<int32_t>(y - 2 * search_area + 1);

        int32_t curr_block_x = physical_idx % blocks_width;
        int32_t curr_block_y = physical_idx / blocks_width;

        int32_t ref_block_x = curr_block_x + motion_x;
        int32_t ref_block_y = curr_block_y + motion_y;

        int32_t ref_physical_idx = ref_block_y * blocks_width + ref_block_x;
	curr_dxt[physical_idx].interp = curr_dxt[ref_physical_idx].interp;
    }
  }
}

int GetFrame(std::ifstream &in_stream, PhysicalDXTBlock *prev_dxt, PhysicalDXTBlock *curr_dxt, 
            MPTCDecodeInfo *decode_info) {

  if(!in_stream.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    exit(-1);
  }

  if(in_stream.eof()) {
    std::cerr << "Error end of file reached, no more frames!" << std::endl;
    return -1;
  }
  // This is the start read all the frame meta data once and store it in the DecodeInfo for 
  // decoding further frames
  
  if(decode_info->is_start) {

    in_stream.read(reinterpret_cast<char*>(&(decode_info->frame_height)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->frame_width)), 4);
    decode_info->num_blocks = (decode_info->frame_height/4 * decode_info->frame_width/4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->unique_interval)), 1);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->search_area)), 1);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->total_frame_count)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_unique_count)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_palette)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_motion_indices)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_ep_Y)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_ep_C)), 4);
    decode_info->is_unique = true;
    decode_info->curr_frame = 0;

    // malloc memories to be used for further decoding of all the frames
    decode_info->comp_palette = (uint8_t*)(malloc(decode_info->max_compressed_palette));
    decode_info->uncomp_palette = (uint8_t*)(malloc(decode_info->max_unique_count));
    decode_info->comp_motion_indices = (uint8_t*)(malloc(decode_info->max_compressed_motion_indices));
    decode_info->motion_indices = (uint8_t*)(malloc(2 * decode_info->num_blocks));
    decode_info->comp_ep_Y = (uint8_t*)(malloc(decode_info->max_compressed_ep_Y));
    decode_info->comp_ep_C = (uint8_t*)(malloc(decode_info->max_compressed_ep_C));

    decode_info->wav_ep1_Y = (uint8_t*)(malloc(decode_info->num_blocks));
    decode_info->wav_ep1_C = (uint8_t*)(malloc(2 * decode_info->num_blocks));

    decode_info->wav_ep2_Y = (uint8_t*)(malloc(decode_info->num_blocks));
    decode_info->wav_ep2_C = (uint8_t*)(malloc(2 * decode_info->num_blocks));

    decode_info->ep1_Y = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep2_Y = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep1_Co = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep2_Co = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep1_Cg = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep2_Cg = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->is_start = false;

  }


  // If unique set then decode the current dictionary
  if(decode_info->is_unique) {

    decode_info->unique_idx_offset = 0;
    decode_info->curr_idx = 0;
    uint32_t compressed_palette_size;
    in_stream.read(reinterpret_cast<char*>(&compressed_palette_size), 4);
    in_stream.read(reinterpret_cast<char*>(decode_info->comp_palette), compressed_palette_size);
    uint32_t unique_count;
    in_stream.read(reinterpret_cast<char*>(&unique_count), 4);
    assert(decode_info->comp_palette != NULL && decode_info->uncomp_palette != NULL);

    EntropyDecode(decode_info->comp_palette, decode_info->uncomp_palette,
                  compressed_palette_size, unique_count, false);
    decode_info->is_unique = false;
     
    decode_info->curr_frame++;

  }


  // Decode a single frame
  uint32_t num_unique;
  in_stream.read(reinterpret_cast<char*>(&num_unique), 4);
  uint32_t *unique_indices = reinterpret_cast<uint32_t*>(decode_info->uncomp_palette + decode_info->unique_idx_offset);

  uint32_t comp_motion_indices_sz;
  in_stream.read(reinterpret_cast<char*>(&comp_motion_indices_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_motion_indices), comp_motion_indices_sz);
   
  EntropyDecode(decode_info->comp_motion_indices,
                decode_info->motion_indices,
		comp_motion_indices_sz,
		2 * decode_info->num_blocks,
		false);

  uint32_t comp_Y_sz, comp_C_sz;
  in_stream.read(reinterpret_cast<char*>(&comp_Y_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep_Y), comp_Y_sz);
  EntropyDecode(decode_info->comp_ep_Y,
                decode_info->wav_ep1_Y,
		comp_Y_sz,
		decode_info->num_blocks,
		false);


  in_stream.read(reinterpret_cast<char*>(&comp_C_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep_C), comp_C_sz);
  EntropyDecode(decode_info->comp_ep_C,
                decode_info->wav_ep1_C,
		comp_C_sz,
		2 * decode_info->num_blocks,
		false);

  in_stream.read(reinterpret_cast<char*>(&comp_Y_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep_Y), comp_Y_sz);
  EntropyDecode(decode_info->comp_ep_Y,
                decode_info->wav_ep2_Y,
		comp_Y_sz,
		decode_info->num_blocks,
		false);


  in_stream.read(reinterpret_cast<char*>(&comp_C_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep_C), comp_C_sz);
  EntropyDecode(decode_info->comp_ep_C,
                decode_info->wav_ep2_C,
		comp_C_sz,
		2 * decode_info->num_blocks,
		false);
              
  // Interpolation Data
  ReconstructDXTFrame(
      reinterpret_cast<uint32_t*>(decode_info->uncomp_palette + decode_info->unique_idx_offset),
      num_unique,
      decode_info,
      prev_dxt,
      curr_dxt);

  // End Point----1
  ReconstructEndpoints(decode_info, curr_dxt, 1);

  // End Point----2
  ReconstructEndpoints(decode_info, curr_dxt, 2);

  decode_info->unique_idx_offset += 4*num_unique;
  if(decode_info->curr_idx >= decode_info->unique_interval-1) {
    decode_info->curr_idx = 0;
    decode_info->is_unique = true;
  }

  decode_info->curr_idx++;

  if(decode_info->curr_frame >= decode_info->total_frame_count) {
    in_stream.seekg(0, in_stream.beg);
    in_stream.seekg(34);
    decode_info->is_unique = true;
    decode_info->curr_frame = 0;
  }

  return 0;
}


void GetFrameMultiThread(std::ifstream &in_stream, 
                        PhysicalDXTBlock *prev_dxt, 
			PhysicalDXTBlock *curr_dxt, 
			MPTCDecodeInfo *decode_info) 
{

  if(!in_stream.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    exit(-1);
  }

  if(in_stream.eof()) {
    std::cerr << "Error end of file reached, no more frames!" << std::endl;
    exit(-1);
  }
  // This is the start read all the frame meta data once and store it in the DecodeInfo for 
  // decoding further frames
  
  if(decode_info->is_start) {

    in_stream.read(reinterpret_cast<char*>(&(decode_info->frame_height)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->frame_width)), 4);
    decode_info->num_blocks = (decode_info->frame_height/4 * decode_info->frame_width/4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->unique_interval)), 1);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->search_area)), 1);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->total_frame_count)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_unique_count)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_palette)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_motion_indices)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_ep_Y)), 4);
    in_stream.read(reinterpret_cast<char*>(&(decode_info->max_compressed_ep_C)), 4);
    decode_info->is_unique = true;
    decode_info->curr_frame = 0;
    decode_info->is_multi_thread = true;

    // malloc memories to be used for further decoding of all the frames
    decode_info->comp_palette = (uint8_t*)(malloc(decode_info->max_compressed_palette));
    decode_info->uncomp_palette = (uint8_t*)(malloc(decode_info->max_unique_count));
    decode_info->comp_motion_indices = (uint8_t*)(malloc(decode_info->max_compressed_motion_indices));
    decode_info->motion_indices = (uint8_t*)(malloc(2 * decode_info->num_blocks));

    decode_info->wav_ep1_Y = (uint8_t*)(malloc(decode_info->num_blocks));
    decode_info->wav_ep1_C = (uint8_t*)(malloc(2 * decode_info->num_blocks));

    decode_info->wav_ep2_Y = (uint8_t*)(malloc(decode_info->num_blocks));
    decode_info->wav_ep2_C = (uint8_t*)(malloc(2 * decode_info->num_blocks));

    decode_info->ep1_Y = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep2_Y = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep1_Co = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep2_Co = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep1_Cg = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->ep2_Cg = (int8_t*)(malloc(decode_info->num_blocks));
    decode_info->is_start = false;

    if(decode_info->is_multi_thread) {
      decode_info->comp_ep1_Y = (uint8_t*)(malloc(decode_info->max_compressed_ep_Y));
      decode_info->comp_ep1_C = (uint8_t*)(malloc(decode_info->max_compressed_ep_C));
      decode_info->comp_ep2_Y = (uint8_t*)(malloc(decode_info->max_compressed_ep_Y));
      decode_info->comp_ep2_C = (uint8_t*)(malloc(decode_info->max_compressed_ep_C));
    }


  }


  // If unique set then decode the current dictionary
  if(decode_info->is_unique) {

    decode_info->unique_idx_offset = 0;
    decode_info->curr_idx = 0;
    uint32_t compressed_palette_size;
    in_stream.read(reinterpret_cast<char*>(&compressed_palette_size), 4);
    in_stream.read(reinterpret_cast<char*>(decode_info->comp_palette), compressed_palette_size);
    uint32_t unique_count;
    in_stream.read(reinterpret_cast<char*>(&unique_count), 4);
    assert(decode_info->comp_palette != NULL && decode_info->uncomp_palette != NULL);

    EntropyDecode(decode_info->comp_palette, decode_info->uncomp_palette,
                  compressed_palette_size, unique_count, false);
    decode_info->is_unique = false;
    decode_info->curr_frame++;

  }
  // Decode a single frame
  uint32_t num_unique;
  in_stream.read(reinterpret_cast<char*>(&num_unique), 4);
  uint32_t *unique_indices = reinterpret_cast<uint32_t*>(decode_info->uncomp_palette + decode_info->unique_idx_offset);

  uint32_t comp_motion_indices_sz;
  in_stream.read(reinterpret_cast<char*>(&comp_motion_indices_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_motion_indices), comp_motion_indices_sz);
 
  // Motion Indices Thread
  std::thread motion_decode(EntropyDecode, // Function pointer or Name
                            decode_info->comp_motion_indices,
                            decode_info->motion_indices,
		            comp_motion_indices_sz,
                            2 * decode_info->num_blocks,
                            false);

  uint32_t comp_Y2_sz, comp_C2_sz;
  uint32_t comp_Y1_sz, comp_C1_sz;


  //************************Endpoint 1********************//
  //Endpoint One(1) Thread for Y channel
  in_stream.read(reinterpret_cast<char*>(&comp_Y1_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep1_Y), comp_Y1_sz);

  std::thread ep1_Y_decode(EntropyDecode,
                           decode_info->comp_ep1_Y,
                           decode_info->wav_ep1_Y,
		           comp_Y1_sz,
		           decode_info->num_blocks,
                           false);

  // Endpoing One(1) Thread for C channel
  in_stream.read(reinterpret_cast<char*>(&comp_C1_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep1_C), comp_C1_sz);

  std::thread ep1_C_decode(EntropyDecode,
                           decode_info->comp_ep1_C,
                           decode_info->wav_ep1_C,
		           comp_C1_sz,
		           2 * decode_info->num_blocks,
		           false);





  ///******************Endpoint 2**********//
  // Endpoint Two(2) Thread for Y channel
  in_stream.read(reinterpret_cast<char*>(&comp_Y2_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep2_Y), comp_Y2_sz);
  std::thread ep2_Y_decode(EntropyDecode,
                           decode_info->comp_ep2_Y,
                           decode_info->wav_ep2_Y,
		           comp_Y2_sz,
		           decode_info->num_blocks,
                           false);


  //Endpoint Two(2) Thread C channel
  in_stream.read(reinterpret_cast<char*>(&comp_C2_sz), 4);
  in_stream.read(reinterpret_cast<char*>(decode_info->comp_ep2_C), comp_C2_sz);
  std::thread ep2_C_decode(EntropyDecode,
                           decode_info->comp_ep2_C,
                           decode_info->wav_ep2_C,
		           comp_C2_sz,
                           2 * decode_info->num_blocks,
                           false);

  // Wait for motion indices to be decoded
  if(motion_decode.joinable())
    motion_decode.join();
  else std::cout << "motion decode thread join error!" << std::endl;

   std::thread reconstruct_interp(ReconstructDXTFrame,
      reinterpret_cast<uint32_t*>(decode_info->uncomp_palette + decode_info->unique_idx_offset),
      num_unique,
      decode_info,
      prev_dxt,
      curr_dxt
      );


   // Wait for ep1 Y and C decoding
   if(ep1_Y_decode.joinable())
     ep1_Y_decode.join(); 
  else std::cout << "ep1 Y decode thread join error!" << std::endl;

   if(ep1_C_decode.joinable())
     ep1_C_decode.join();
   else std::cout << "ep1 C decode thread join error!" << std::endl;
  
   std::thread reconstruct_ep1(ReconstructEndpoints,
                               decode_info, 
			       curr_dxt, 
			       1);

   // Wait for ep2 Y and C decoding
   if(ep2_Y_decode.joinable())
     ep2_Y_decode.join(); 
  else std::cout << "ep2 Y decode thread join error!" << std::endl;

   if(ep2_C_decode.joinable())
     ep2_C_decode.join();
  else std::cout << "ep2 C decode thread join error!" << std::endl;


   std::thread reconstruct_ep2(ReconstructEndpoints,
                               decode_info,
			       curr_dxt,
			       2);

   if(reconstruct_interp.joinable())
     reconstruct_interp.join();
   else std::cout << "reconstruct interp thread join error!" << std::endl;

   if(reconstruct_ep1.joinable())
     reconstruct_ep1.join(); 
   else std::cout << "reconstruct ep1 thread join error!" << std::endl;
 
   if(reconstruct_ep2.joinable())
     reconstruct_ep2.join();
   else std::cout << "reconstruct ep2 thread join error!" << std::endl;

   
   decode_info->unique_idx_offset += 4*num_unique;
   if(decode_info->curr_idx >= decode_info->unique_interval-1) {
     decode_info->curr_idx = 0;
     decode_info->is_unique = true;
   }

   decode_info->curr_idx++;

/*   if(decode_info->curr_frame >= decode_info->total_frame_count) {*/
     //in_stream.seekg(0, in_stream.beg);
     //in_stream.seekg(34);
     //decode_info->is_unique = true;
     //decode_info->curr_frame = 0;
   //}

  return;
}


//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////        Buffered Decoding      //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

int InitBufferedDecode(uint8_t buffer_sz, 
                       BufferStruct* &ptr_buffer_struct,
		       std::ifstream &in_stream,
		       uint32_t num_blocks) {
  
  // Decode Info
  assert(2 < buffer_sz && buffer_sz < 20 && "!!Buffer Size too Big!!\n");

  ptr_buffer_struct->ptr_decode_info = (MPTCDecodeInfo*)malloc(sizeof(MPTCDecodeInfo));
  ptr_buffer_struct->ptr_decode_info->is_start = true;
  ptr_buffer_struct->buffer_sz = buffer_sz; 

  ptr_buffer_struct->buffered_dxts = (PhysicalDXTBlock**)malloc(buffer_sz * sizeof(PhysicalDXTBlock*));

  for(uint8_t idx = 0; idx < buffer_sz; idx++){
    ptr_buffer_struct->buffered_dxts[idx] = (PhysicalDXTBlock*)malloc(num_blocks * sizeof(PhysicalDXTBlock));
  }

  ptr_buffer_struct->curr_dxt_idx = 0;
  GetFrameMultiThread(in_stream,
                      NULL,
		      ptr_buffer_struct->buffered_dxts[0],
		      ptr_buffer_struct->ptr_decode_info
                     );

  for(uint8_t idx = 1; idx < buffer_sz - 1; idx++) {

    GetFrameMultiThread(in_stream,
	                ptr_buffer_struct->buffered_dxts[idx - 1],
			ptr_buffer_struct->buffered_dxts[idx],
			ptr_buffer_struct->ptr_decode_info
	               );

  }

  ptr_buffer_struct->curr_decode_idx = ptr_buffer_struct->buffer_sz - 1;
  ptr_buffer_struct->prev_decode_idx = ptr_buffer_struct->buffer_sz - 1 - 1;

}


int GetBufferedFrame(BufferStruct *ptr_buffer_struct, PhysicalDXTBlock * &curr_dxt, std::ifstream  &in_stream) {

  curr_dxt = ptr_buffer_struct->buffered_dxts[ptr_buffer_struct->curr_dxt_idx];

  std::thread th_decode_frame(GetFrameMultiThread,
                             std::ref(in_stream),
                             ptr_buffer_struct->buffered_dxts[ptr_buffer_struct->prev_decode_idx],
			     ptr_buffer_struct->buffered_dxts[ptr_buffer_struct->curr_decode_idx],
			     ptr_buffer_struct->ptr_decode_info);

  
  if(th_decode_frame.joinable())
    th_decode_frame.detach();
  else std::cout << "Get Frame MultiThread not joinable!" << std::endl;
  
  ptr_buffer_struct->curr_decode_idx  = 
    (ptr_buffer_struct->curr_decode_idx + 1) % ptr_buffer_struct->buffer_sz;

  ptr_buffer_struct->prev_decode_idx  = 
    (ptr_buffer_struct->prev_decode_idx + 1) % ptr_buffer_struct->buffer_sz;

  ptr_buffer_struct->curr_dxt_idx = 
    (ptr_buffer_struct->curr_dxt_idx + 1) % ptr_buffer_struct->buffer_sz;

}
 
