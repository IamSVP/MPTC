#include "codec.h"
#include "bit_stream.h"
#include "dxt_image.h"
#include "arithmetic_codec.h"
#include "stb_image_write.h"
#include "stb_image.h"
#include "utils.h"
#include "image.h"
#include "image_processing.h"
#include "image_utils.h"
#include "pipeline.h"
#include "entropy.h"

#include <dirent.h>
#include <fstream>
#include <tuple>
#include <algorithm>
#include <vector>
#include <cmath>
#include <atomic>
#include <typeinfo>
#include <sstream>
#include <iomanip>
#include <bitset>
#include <thread>
#include <functional>
#include <mutex>

//#define PALETTECOMP
//#define ENDPOINTIMG
//#define COMBINEPALETTE
#define VID_CODEC_EP
namespace MPTC {

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

static uint16_t Pack(const uint8_t in[3]) {
  uint16_t result = 0;
  result |= static_cast<uint16_t>(in[0] & 0xF8) << 8;
  result |= static_cast<uint16_t>(in[1] & 0xFC) << 3;
  result |= static_cast<uint16_t>(in[2] & 0xF8) >> 3;
  return result;
}

uint32_t max_unique_count = 0;
uint32_t max_compressed_palette = 0;
uint32_t max_compressed_motion_indices = 0;
uint32_t max_compressed_ep_C = 0;
uint32_t max_compressed_ep_Y = 0;
std::mutex global_mutex;
static const size_t kWaveletBlockDim = 64;
static_assert((kWaveletBlockDim % 2) == 0, "Wavelet dimension must be power of two!");

const uint32_t max_bytes = 1048576; // 1MB 

double MeasureEntropy(std::vector<uint8_t> &symbols) {
  std::vector<uint64_t> counts(257,0);
  for(auto a : symbols)
    counts[a]++;
  uint64_t total = std::accumulate(counts.begin(), counts.end(), 0);
  double entropy = 0.0;
  for(auto count : counts){
    if(count != 0) {
      double p = static_cast<double>(count) / static_cast<double>(total);
      entropy += (-1.0 * p * log2(p));
    }
  }
  return entropy;
}

void EntropyEncode(std::unique_ptr< std::vector<uint16_t> > &symbols, std::vector<uint8_t> &out_data, bool is_bit_model) {
  std::vector<uint8_t> temp_out_data;
  temp_out_data.resize(max_bytes, 0);
  entropy::Arithmetic_Codec arith_data_encoder(max_bytes, temp_out_data.data());
  if(!is_bit_model) {
    entropy::Adaptive_Data_Model data_model(513);
    arith_data_encoder.start_encoder();
    for(size_t idx = 0; idx < symbols->size(); idx++ ) {
      arith_data_encoder.encode(symbols->operator[](idx), data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size()-compressed_size), temp_out_data.data(), compressed_size);
  }
  else {
    entropy::Adaptive_Bit_Model data_model;
    arith_data_encoder.start_encoder();
    for(size_t idx = 0; idx < symbols->size(); idx++ ) {
      arith_data_encoder.encode(symbols->operator[](idx), data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size() - compressed_size), temp_out_data.data(), compressed_size);
  }
}

void EntropyEncode(std::vector<uint16_t> &symbols, std::vector<uint8_t> &out_data, 
                   bool is_bit_model) {
  std::vector<uint8_t> temp_out_data;
  temp_out_data.resize(max_bytes, 0);
  entropy::Arithmetic_Codec arith_data_encoder(max_bytes, temp_out_data.data());
  if(!is_bit_model) {
    entropy::Adaptive_Data_Model data_model(513);
    for(auto a: symbols) {
      arith_data_encoder.encode(a, data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size()-compressed_size), temp_out_data.data(), compressed_size);
  }
  else {
    entropy::Adaptive_Bit_Model data_model;
    arith_data_encoder.start_encoder();
    for(auto a: symbols) {
      arith_data_encoder.encode(a, data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size() - compressed_size), temp_out_data.data(), compressed_size);
  }
}


//Entropy Encode a bunch of symbols 
void EntropyEncode(std::unique_ptr< std::vector<uint8_t> > &symbols, std::vector<uint8_t> &out_data, 
                   bool is_bit_model) {
  std::vector<uint8_t> temp_out_data;
  temp_out_data.resize(max_bytes, 0);
  entropy::Arithmetic_Codec arith_data_encoder(max_bytes, temp_out_data.data());
  if(!is_bit_model) {
    entropy::Adaptive_Data_Model data_model(257);
    arith_data_encoder.start_encoder();
    for(size_t idx = 0; idx < symbols->size(); idx++ ) {
      arith_data_encoder.encode(symbols->operator[](idx), data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size()-compressed_size), temp_out_data.data(), compressed_size);
  }
  else {
    entropy::Adaptive_Bit_Model data_model;
    arith_data_encoder.start_encoder();
    for(size_t idx = 0; idx < symbols->size(); idx++ ) {
      arith_data_encoder.encode(symbols->operator[](idx), data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size() - compressed_size), temp_out_data.data(), compressed_size);
  }
}

void EntropyEncode(std::vector<uint8_t> &symbols, std::vector<uint8_t> &out_data, 
                   bool is_bit_model) {
  std::vector<uint8_t> temp_out_data;
  temp_out_data.resize(max_bytes, 0);
  entropy::Arithmetic_Codec arith_data_encoder(max_bytes, temp_out_data.data());
  if(!is_bit_model) {
    entropy::Adaptive_Data_Model data_model(257);
    arith_data_encoder.start_encoder();
    for(auto a: symbols) {
      arith_data_encoder.encode(a, data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size()-compressed_size), temp_out_data.data(), compressed_size);
  }
  else {
    entropy::Adaptive_Bit_Model data_model;
    arith_data_encoder.start_encoder();
    for(auto a: symbols) {
      arith_data_encoder.encode(a, data_model);
    }
    arith_data_encoder.stop_encoder();
    uint32_t compressed_size = arith_data_encoder.get_num_bytes();
    out_data.resize(out_data.size() + compressed_size);
    memcpy(out_data.data()+(out_data.size() - compressed_size), temp_out_data.data(), compressed_size);
  }
}

#ifdef UNIQUE
void CompressPNGStream(const std::string dir_name, const std::string out_file, uint32_t search_area, int32_t vErrThreshold, uint32_t interval, std::string ep_dir) {
  DIR *dir = opendir(dir_name.c_str());
  if (!dir) {
    std::cerr<< "Error opening directory: " << dir_name << std::endl;
    exit(-1);
  }

  std::string first_file, curr_file, prev_file;
  struct dirent *entry = NULL;
  uint32_t frame_number = 0; 
  // skip '.' and '..' 
  std::vector<std::string> file_names;
  while ((entry = readdir(dir)) != NULL) {
    if (strlen(entry->d_name) == 1 && strncmp(entry->d_name, ".", 1) == 0) continue;
    else if (strlen(entry->d_name) == 2 && strncmp(entry->d_name, "..", 2) == 0) continue;
    else file_names.push_back(entry->d_name);
  }
  std::sort(file_names.begin(), file_names.end());

    
#ifndef NDEBUG
  for(auto a : file_names)
    std::cout << a << std::endl;
#endif
  // read the first file name
  //entry = readdir(dir);

  first_file = dir_name + "/" + file_names[0];
  file_names.erase(file_names.begin());
  // Declare all pointers to DXTImages
  std::unique_ptr<MPTC::DXTImage> first_frame(new MPTC::DXTImage(first_file, true, search_area, vErrThreshold));
  std::unique_ptr<MPTC::DXTImage> null_dxt(nullptr);
  std::unique_ptr<MPTC::DXTImage> curr_frame(nullptr), prev_frame(nullptr);
  uint32_t frame_count = 1;
#ifndef NDEBUG
  std::unique_ptr<GreyImage> recons_image = std::move(first_frame->InterpolationImage());  
  stbi_write_png("orig.png", recons_image->Width(), recons_image->Height(),
                 1, recons_image->Pack().data(), recons_image->Width());
#endif
#ifdef ENDPOINTIMG
  std::unique_ptr<RGBImage> ep1_ptr_fir = std::move(first_frame->EndpointOneImage());
  stbi_write_png("ep1_0.png", ep1_ptr_fir->Width(), ep1_ptr_fir->Height(), 
                  3, ep1_ptr_fir->Pack().data(), 3 * ep1_ptr_fir->Width());
#endif
  uint32_t frame_height, frame_width;
  frame_width = first_frame->Width();
  frame_height = first_frame->Height();

  //open out file for writing to it
  std::ofstream out_stream(out_file.c_str(), std::ofstream::binary);
  // Write out frame height and frame width
   
  out_stream.write(reinterpret_cast<const char*>(&frame_height), 4);
  out_stream.write(reinterpret_cast<const char*>(&frame_width), 4);

  // out_data to be written to file
  std::vector<uint8_t> out_data;
  // copyt height and widht 
  // copy the size of the mask index
  // copy mask index data
  // copy size of of motion indices
  // copy unique_indices
  // copy data of motion indices

#ifndef NDEBUG
  std::cout << "Frame Number: " << frame_count << std::endl;
  std::cout << "PSNR before reencoding: " << first_frame->PSNR() << std::endl;
#endif
  first_frame->Reencode(null_dxt, 0);
#ifndef NDEBUG
  std::cout << "PSNR after reencoding: " << first_frame->PSNR() << std::endl;
  std::cout << "Intra blocks found: " << first_frame->_motion_indices.size() << std::endl;
#endif 

#ifdef PALETTECOMP
  std::cout << "****Temp palette 8 bit****" << std::endl;
  std::vector<uint8_t> palette_8bit = first_frame->Get8BitPalette();
  std::cout << "Uncompress palette:" << first_frame->_unique_palette.size() * 4 << std::endl;
  std::vector<uint8_t> compressed_palette_8bit;
  EntropyEncode(palette_8bit, compressed_palette_8bit);
  std::cout << "Compressed palette size:" << compressed_palette_8bit.size() << std::endl;
#endif

  uint32_t curr_num = 2;
  bool set_intra = false;
  //Entropy Encode
  EntropyEncode(first_frame, out_data);
  out_stream.write(reinterpret_cast<const char*>(out_data.data()), out_data.size());
  uint32_t num_pixels = frame_height * frame_width;
#ifndef NDEBUG
  std::cout << "Total Bytes: " << out_data.size() << std::endl;
  double bpp = static_cast<double>(out_data.size() * 8)/num_pixels;
  std::cout << "BPP: " << bpp << std::endl;
#endif

  prev_frame = std::move(first_frame);
  prev_file.clear();
  prev_file += first_file;
  std::vector<uint8_t> combined_8bit_palette;
  for(auto a : file_names) {
    
    frame_number++;
    curr_file.clear();
    out_data.clear();
    curr_file = dir_name + "/" + a;
    // ******Decide intra or inter here...*********
    curr_frame.reset(new MPTC::DXTImage(curr_file, set_intra, search_area, vErrThreshold));
    std::cout << "Frame Name: " << a << std::endl;
#ifndef NDEBUG
    std::cout << "PSNR before reencoding: " << curr_frame->PSNR() << std::endl;
#endif  
    curr_frame->Reencode(prev_frame, -1);
#ifndef NDEBUG
    std::cout << "PSNR after reencoding: " << curr_frame->PSNR() << std::endl;
    std::cout << "Intra blocks found: " << curr_frame->_motion_indices.size() << std::endl;
#endif 
#ifdef ENDPOINTIMG
    std::string ep_file;
    ep_file.clear();
    ep_file = ep_dir + "/" + a + "_ep1.png";
    std::unique_ptr<RGBImage> ep1_ptr = std::move(curr_frame->EndpointOneImage());
    stbi_write_png(ep_file.c_str(), ep1_ptr->Width(), ep1_ptr->Height(), 
                  3, ep1_ptr->Pack().data(), 3 * ep1_ptr->Width());    
#endif 
#ifdef PALETTECOMP
  std::cout << "****Temp palette 8 bit****" << std::endl;
  std::vector<uint8_t> temp_palette_8bit = curr_frame->Get8BitPalette();
  std::cout << "Uncompress palette:" << curr_frame->_unique_palette.size() * 4 << std::endl;
  std::vector<uint8_t> temp_compressed_palette_8bit;
  EntropyEncode(temp_palette_8bit, temp_compressed_palette_8bit);
  std::cout << "Compressed palette size:" << temp_compressed_palette_8bit.size() << std::endl;
#endif

    //Entropy Encode
    EntropyEncode(curr_frame, out_data);
    out_stream.write(reinterpret_cast<const char*>(out_data.data()), out_data.size());
    out_stream.flush();
#ifndef NDEBUG
    std::cout << "Total Bytes: " << out_data.size() << std::endl;
    bpp = static_cast<double>(out_data.size() * 8)/num_pixels;
    std::cout << "BPP: " << bpp << std::endl;
    std::cout << std::endl << std::endl;
#endif
    if(curr_num > interval){
      set_intra = true;
      curr_num = 1;
#ifdef COMBINEPALETTE
      std::vector<uint8_t> compressed_combined_8bit_palette;
      EntropyEncode(combined_8bit_palette, compressed_combined_8bit_palette, false);
      std::cout << std::endl;
      std::cout << "------------------------------" << std::endl;
      std::cout << "Uncompressed combined Palette:" << combined_8bit_palette.size() << std::endl;
      std::cout << "Compressed combined Palette:" << compressed_combined_8bit_palette.size() << std::endl;
      std::cout << "-----------------------------" << std::endl;
      combined_8bit_palette.clear();
#endif

    }
    else { 
      set_intra = false;
      curr_num++;
#ifdef COMBINEPALETTE
      std::vector<uint8_t> temp_palette_8bit = curr_frame->Get8BitPalette();
      combined_8bit_palette.insert(std::end(combined_8bit_palette), std::begin(temp_palette_8bit), std::end(temp_palette_8bit));
#endif
   }

    prev_frame = std::move(curr_frame);
  }
  out_stream.close();
}

#endif

// number of symbols expected 
// assumes memory is already allocated for out_symbols
//
void ReconstructDXTData(std::vector<uint32_t> &unique_indices,
                                  std::vector<std::tuple<uint8_t ,uint8_t> > &motion_indices,
                                  std::unique_ptr<DXTImage> &curr_frame,
                                  std::unique_ptr<DXTImage> &prev_frame, uint8_t search_area,
				  std::string ep_dir, uint32_t frame_number) {

  std::vector<uint32_t> interpolation_data;
  //std::cout << frame_number << std::endl;
  std::string ep_file1, ep_file2;
  std::string ep_orig1, ep_orig2;
  std::ostringstream ss;
  ss << std::setw(5) << std::setfill('0') << frame_number;
 
  std::string ep_orig = "../../RawFrames/CostRica2K/endpoint";
  ep_file1 = ep_dir + "/1/" + ss.str() + ".png";
  ep_file2 = ep_dir + "/2/" + ss.str() + ".png";

  ep_orig1 = ep_orig + "/1/" + ss.str() + ".png";
  ep_orig2 = ep_orig + "/2/" + ss.str() + ".png";
  int x,y,n;
  uint8_t *ep1 = stbi_load(ep_file1.c_str(), &x, &y, &n, 3);
  uint8_t *ep2 = stbi_load(ep_file2.c_str(), &x, &y, &n, 3);
  
  //std::ifstream ep_stream1, ep_stream2, ep_stream_orig1, ep_stream_orig2;

  //ep_stream1.open(ep_file1.c_str(), std::ifstream::binary);
  //ep_stream2.open(ep_file2.c_str(), std::ifstream::binary);

  //ep_stream_orig1.open(ep_orig1.c_str(), std::ifstream::binary);
  //ep_stream_orig2.open(ep_orig2.c_str(), std::ifstream::binary);

  //std::vector<uint8_t> ep1, ep2;
  //if(!ep_stream1.is_open() || !ep_stream2.is_open()) {
    //std::cerr << "Endpoint files not opened!" << std::endl;
    //exit(-1);
  //}
  //ep_stream1.seekg(0, ep_stream1.end);
  //int length = ep_stream1.tellg();
  //ep_stream1.seekg(0, ep_stream1.beg);

  //ep1.resize(length);
  //ep2.resize(length);

  //ep_stream1.read(reinterpret_cast<char*>(ep1.data()), length);
  //ep_stream2.read(reinterpret_cast<char*>(ep2.data()), length);

  //entropy::BitReader R1(ep1.data()), R2(ep2.data());
  uint16_t r, g, b; 
  uint16_t rgb565;

  int prev_mask_idx = 0; 
  int32_t blocks_width = curr_frame->_blocks_width;
  uint32_t curr_unique_idx = 0;
  for(int physical_idx = 0; physical_idx < curr_frame->_num_blocks; physical_idx++) {

      uint8_t x = std::get<0>(motion_indices[physical_idx]);
      uint8_t y = std::get<1>(motion_indices[physical_idx]);
      if(x == 255 && y == 255) { // if this conditon holds it's a unique idx
	assert(curr_unique_idx < unique_indices.size());
        interpolation_data.push_back(unique_indices[curr_unique_idx]);
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
        interpolation_data.push_back(prev_frame->_physical_blocks[ref_physical_idx].interp);
      }
      else { //Intra block motion, fetch data from own frame

        int32_t motion_x = static_cast<int32_t>(x - search_area);
        int32_t motion_y = static_cast<int32_t>(y - 2 * search_area + 1);

        int32_t curr_block_x = physical_idx % blocks_width;
        int32_t curr_block_y = physical_idx / blocks_width;

        int32_t ref_block_x = curr_block_x + motion_x;
        int32_t ref_block_y = curr_block_y + motion_y;

        int32_t ref_physical_idx = ref_block_y * blocks_width + ref_block_x;
        if(ref_physical_idx >  static_cast<int32_t>(interpolation_data.size())) {
	  std::cout << "Should never come here!!" << std::endl;
	}
        interpolation_data.push_back(interpolation_data[ref_physical_idx]);

      }
    curr_frame->_physical_blocks[physical_idx].interp = interpolation_data[physical_idx];

    /*r = g = b = 0;*/
    //r = ep1[3 * physical_idx];
    //g = ep1[3 * physical_idx + 1];
    /*b = ep1[3 * physical_idx + 2];*/
    //r = R1.ReadBits(5);
    //g = R1.ReadBits(6);
    //b = R1.ReadBits(5);
/*    uint16_t rgb565_1 = Pack(ep1 + 3 * physical_idx);*/
    //[>rgb565_1 |= (r << 11);<]
    ////rgb565_1 |= (g << 5);
    //[>rgb565_1 |= (b);<]
    //curr_frame->_physical_blocks[physical_idx].ep1 = rgb565_1;
 
    //[>r = g = b = 0;<]
    ////r = ep2[3 * physical_idx];
    ////g = ep2[3 * physical_idx + 1];
    ////b = ep2[3 * physical_idx + 2];

    ////r = R2.ReadBits(5);
    ////g = R2.ReadBits(6);
    ////b = R2.ReadBits(5);
    //uint16_t rgb565_2 = Pack(ep2 + 3 * physical_idx);
//[>    rgb565_2 |= (r << 11);<]
    ////rgb565_2 |= (g << 5);
    ////rgb565_2 |= (b);
    //curr_frame->_physical_blocks[physical_idx].ep2 = rgb565_2;
    ////RecomputeIndex();

    //if(curr_frame->_physical_blocks[physical_idx].ep1 < curr_frame->_physical_blocks[physical_idx].ep2) {

      //std::swap(curr_frame->_physical_blocks[physical_idx].ep1, curr_frame->_physical_blocks[physical_idx].ep2);
     //std::bitset<32> x1(curr_frame->_physical_blocks[physical_idx].interp);
     
////uint32_t interpol1 =   curr_frame->_physical_blocks[physical_idx].interp;
     ////curr_frame->_physical_blocks[physical_idx].interp = (0x55555555 ^ curr_frame->_physical_blocks[physical_idx].interp);   

    //std::bitset<32> x2(curr_frame->_physical_blocks[physical_idx].interp);
////uint32_t interpol2 =   curr_frame->_physical_blocks[physical_idx].interp;
    //////std::cout << interpol1 << interpol2 << std::endl;
    ////int bbb = interpol1;
    ////int c = bbb * 500;
    //}
    //else if(curr_frame->_physical_blocks[physical_idx].ep1 == curr_frame->_physical_blocks[physical_idx].ep2) {
      //curr_frame->_physical_blocks[physical_idx].interp = 0x00000000;
    //}

  } // all sorts of motion indices
  return;
}

void EntropyDecode(std::vector<uint8_t> &compressed_data, std::vector<uint8_t> &out_symbols,                   bool is_bit_model) {
    entropy::Arithmetic_Codec arith_decoder(compressed_data.size() + 100, compressed_data.data());
  if(!is_bit_model) {
    entropy::Adaptive_Data_Model model(257);
    arith_decoder.start_decoder();
    for(size_t sym_idx = 0; sym_idx < out_symbols.size(); sym_idx++)
      out_symbols[sym_idx] = arith_decoder.decode(model);
    arith_decoder.stop_decoder();
  }
  else {
    entropy::Adaptive_Bit_Model model;
    arith_decoder.start_decoder();
    for(size_t sym_idx = 0; sym_idx < out_symbols.size(); sym_idx++)
      out_symbols[sym_idx] = arith_decoder.decode(model);
    arith_decoder.stop_decoder();
  }
  return;
}

template <typename T> std::unique_ptr<std::vector<uint16_t> >
RunDXTEndpointPipeline_16bit(const std::unique_ptr<Image<T> > &img) {
  static_assert(PixelTraits::NumChannels<T>::value,
    "This should operate on each DXT endpoing channel separately");

  const bool kIsSixBits = PixelTraits::BitsUsed<T>::value == 6;
  typedef typename WaveletResultTy<T, kIsSixBits>::DstTy WaveletSignedTy;
  typedef typename PixelTraits::UnsignedForSigned<WaveletSignedTy>::Ty WaveletUnsignedTy;

  auto pipeline = Pipeline<Image<T>, Image<WaveletSignedTy> >
    ::Create(FWavelet2D<T, kWaveletBlockDim>::New())
    ->Chain(MakeUnsigned<WaveletSignedTy>::New())
    ->Chain(Linearize<WaveletUnsignedTy>::New())
    ->Chain(ReducePrecision<WaveletUnsignedTy, uint16_t>::New());

  return std::move(pipeline->Run(img));
}


template <typename T> std::unique_ptr<std::vector<uint8_t> >
RunDXTEndpointPipeline(const std::unique_ptr<Image<T> > &img) {
  static_assert(PixelTraits::NumChannels<T>::value,
    "This should operate on each DXT endpoing channel separately");

  const bool kIsSixBits = PixelTraits::BitsUsed<T>::value == 6;
  typedef typename WaveletResultTy<T, kIsSixBits>::DstTy WaveletSignedTy;
  typedef typename PixelTraits::UnsignedForSigned<WaveletSignedTy>::Ty WaveletUnsignedTy;

  auto pipeline = Pipeline<Image<T>, Image<WaveletSignedTy> >
    ::Create(FWavelet2D<T, kWaveletBlockDim>::New())
    ->Chain(MakeUnsigned<WaveletSignedTy>::New())
    ->Chain(Linearize<WaveletUnsignedTy>::New())
    ->Chain(ReducePrecision<WaveletUnsignedTy, uint8_t>::New());

  return std::move(pipeline->Run(img));
}

template <typename T> std::unique_ptr<Image<T> >
RunInverseDXTEndpointPipeline( const std::unique_ptr<std::vector<uint8_t> > &coefficients, uint32_t width, uint32_t height ) {
   static_assert(PixelTraits::NumChannels<T>::value,
    "This should operate on each DXT endpoing channel separately");

  const bool kIsSixBits = PixelTraits::BitsUsed<T>::value == 6;
  typedef typename WaveletResultTy<T, kIsSixBits>::DstTy WaveletSignedTy;
  typedef typename PixelTraits::UnsignedForSigned<WaveletSignedTy>::Ty WaveletUnsignedTy;
  auto pipeline = Pipeline<std::vector<uint8_t>, std::vector<WaveletUnsignedTy> >
                  ::Create(ReducePrecision<uint8_t, WaveletUnsignedTy>::New())
		  ->Chain(DeLinearize<WaveletUnsignedTy>::New(width, height))
		  ->Chain(MakeSigned<WaveletUnsignedTy>::New())
		  ->Chain(IWavelet2D<WaveletSignedTy, T, kWaveletBlockDim>::New());
  return std::move(pipeline->Run(coefficients));
}

void CompressEndpoint(std::unique_ptr<DXTImage> &curr_frame, std::unique_ptr<DXTImage> &prev_frame, int number) {

  auto initial_endpoint_pipeline =
  Pipeline<RGB565Image, YCoCg667Image>
  ::Create(RGB565toYCoCg667::New())
  ->Chain(std::move(ImageSplit<YCoCg667>::New()));
 
  std::unique_ptr<RGB565Image> ep1, ep2;
  if(number == 1) {
    ep1 = std::move(curr_frame->EndpointOneValues());
    ep2 = std::move(prev_frame->EndpointOneValues());
  }
  else {
    ep1 = std::move(curr_frame->EndpointTwoValues());
    ep2 = std::move(prev_frame->EndpointTwoValues());
  }

  auto ep1_YCoCg667 = initial_endpoint_pipeline->Run(ep1);
  auto ep2_YCoCg667 = initial_endpoint_pipeline->Run(ep2);

  auto ep1_Y = std::move(std::get<0>(*ep1_YCoCg667));
  auto ep1_Co = std::move(std::get<1>(*ep1_YCoCg667));
  auto ep1_Cg = std::move(std::get<2>(*ep1_YCoCg667));

  auto ep2_Y = std::move(std::get<0>(*ep2_YCoCg667));
  auto ep2_Co = std::move(std::get<1>(*ep2_YCoCg667));
  auto ep2_Cg = std::move(std::get<2>(*ep2_YCoCg667));

//***********************Compress Deltas of curr-prev frame***************//
  uint32_t W = ep1_Y->Width(); 
  uint32_t H = ep1_Y->Height();
  std::unique_ptr<SigSevenBitImage> delta_Y( new SigSevenBitImage(W, H));
  std::unique_ptr<SigSevenBitImage> delta_Cg( new SigSevenBitImage(W, H));
  std::unique_ptr<SigEightBitImage> delta_Co( new SigEightBitImage(W, H));

  for(uint32_t h = 0; h < H; h++) {
    for(uint32_t w = 0; w < W; w++) {
      delta_Y->SetAt(w, h, ep1_Y->GetAt(w, h) - ep2_Y->GetAt(w, h));
      delta_Cg->SetAt(w, h, ep1_Cg->GetAt(w, h) - ep2_Cg->GetAt(w, h));
      delta_Co->SetAt(w, h, ep1_Co->GetAt(w, h) - ep2_Co->GetAt(w, h));
    }
  }
  auto wav_delta_Y = RunDXTEndpointPipeline(delta_Y);
  auto wav_delta_Co = RunDXTEndpointPipeline(delta_Co);
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!! Co DONEE!!!!!!!!!!!!!!!!!" << std::endl;
  auto wav_delta_Cg = RunDXTEndpointPipeline(delta_Cg);
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!Cg DONEE !!!!!!!!!!!!!!" << std::endl;

  std::vector<uint8_t> cmp_wav_delta_Y, cmp_wav_delta_C;
  wav_delta_Co->insert(wav_delta_Co->end(), wav_delta_Cg->begin(), wav_delta_Cg->end());
  
  EntropyEncode(wav_delta_Y, cmp_wav_delta_Y, false);
  EntropyEncode(wav_delta_Co, cmp_wav_delta_C, false);

  std::cout << "-------------------------" << std::endl;
  std::cout << " Inter Compressed Compress delta image: " << cmp_wav_delta_Y.size() + cmp_wav_delta_C.size() << std::endl;
  std::cout << "-------------------------" << std::endl;
}

void ReconstructEndpointsFile(std::unique_ptr<DXTImage> &dxt_image,
                              std::string ep_file1, std::string ep_file2) {
  int x,y,n;

}

void ReconstructEndPoints(std::unique_ptr<DXTImage> &dxt_image,
                          std::unique_ptr< std::vector<uint8_t> > &wav_ep1_Y, 
			  std::unique_ptr< std::vector<uint8_t> > &wav_ep1_C,
			  std::unique_ptr< std::vector<uint8_t> > &wav_ep2_Y,
			  std::unique_ptr< std::vector<uint8_t> > &wav_ep2_C) {

  uint32_t ep_width = dxt_image->_width/4;
  uint32_t ep_height = dxt_image->_height/4;

  auto ep1_Y = RunInverseDXTEndpointPipeline<UnsignedBits<6> >(wav_ep1_Y, ep_width, ep_height);
  std::unique_ptr<std::vector<uint8_t> > wav_ep1_Co(new std::vector<uint8_t>());
  std::unique_ptr<std::vector<uint8_t> > wav_ep1_Cg(new std::vector<uint8_t>());

  //!!Insert increases the size of the array!!
  wav_ep1_Co->insert(wav_ep1_Co->begin(), wav_ep1_C->begin(), wav_ep1_C->begin()+wav_ep1_C->size() /2);

  wav_ep1_Cg->insert(wav_ep1_Cg->begin(), wav_ep1_C->begin()+wav_ep1_C->size() /2, wav_ep1_C->end());

  auto ep1_Co = RunInverseDXTEndpointPipeline< SignedBits<6> >(wav_ep1_Co, ep_width, ep_height);
  auto ep1_Cg = RunInverseDXTEndpointPipeline< SignedBits<7> >(wav_ep1_Cg, ep_width, ep_height);


  auto ep2_Y = RunInverseDXTEndpointPipeline<UnsignedBits<6> >(wav_ep2_Y, ep_width, ep_height);
  std::unique_ptr<std::vector<uint8_t> > wav_ep2_Co(new std::vector<uint8_t>());
  std::unique_ptr<std::vector<uint8_t> > wav_ep2_Cg(new std::vector<uint8_t>());

  //!!Insert increases the array size//
  wav_ep2_Co->insert(wav_ep2_Co->begin(), wav_ep2_C->begin(), wav_ep2_C->begin()+wav_ep2_C->size() /2);

  wav_ep2_Cg->insert(wav_ep2_Cg->begin(), wav_ep2_C->begin()+wav_ep2_C->size() /2, wav_ep2_C->end());

  auto ep2_Co = RunInverseDXTEndpointPipeline< SignedBits<6> >(wav_ep2_Co, ep_width, ep_height);
  auto ep2_Cg = RunInverseDXTEndpointPipeline< SignedBits<7> >(wav_ep2_Cg, ep_width, ep_height);

  for (size_t j = 0; j < ep_height; ++j) {
    for (size_t i = 0; i < ep_width; ++i) {
      uint32_t physical_idx = j * ep_width + i;  
      auto pix1 = ep1_Y->GetAt(i, j);
      auto pix2 = ep1_Co->GetAt(i, j);
      auto pix3 = ep1_Cg->GetAt(i, j);

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

      dxt_image->_physical_blocks[physical_idx].ep1 = x;
    }
  }


  for (size_t j = 0; j < ep_height; ++j) {
    for (size_t i = 0; i < ep_width; ++i) {
      uint32_t physical_idx = j * ep_width + i;  
      auto pix1 = ep2_Y->GetAt(i, j);
      auto pix2 = ep2_Co->GetAt(i, j);
      auto pix3 = ep2_Cg->GetAt(i, j);

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

      dxt_image->_physical_blocks[physical_idx].ep2 = x;
    }
  }

}


//void DecompressEndpoint(std::vector<uint8_t> &com_data, 
void CompressEndpoint(std::unique_ptr<DXTImage> &dxt_frame, std::vector<uint8_t> &out_data) {

  auto initial_endpoint_pipeline =
  Pipeline<RGB565Image, YCoCg667Image>
  ::Create(RGB565toYCoCg667::New())
  ->Chain(std::move(ImageSplit<YCoCg667>::New()));

  std::unique_ptr<RGB565Image> ep1, ep2;
  ep1 = std::move(dxt_frame->EndpointOneValues());
  ep2 = std::move(dxt_frame->EndpointTwoValues());

  auto ep1_YCoCg667 = initial_endpoint_pipeline->Run(ep1);
  auto ep2_YCoCg667 = initial_endpoint_pipeline->Run(ep2);

  auto ep1_Y = std::move(std::get<0>(*ep1_YCoCg667));
  auto ep1_Co = std::move(std::get<1>(*ep1_YCoCg667));
  auto ep1_Cg = std::move(std::get<2>(*ep1_YCoCg667));

  auto wav_ep1_Y = RunDXTEndpointPipeline(ep1_Y);
  auto test_ep1_Y = RunInverseDXTEndpointPipeline<UnsignedBits<6> >(wav_ep1_Y, ep1_Y->Width(),
                                                                    ep1_Y->Height());

  auto wav_ep1_Co = RunDXTEndpointPipeline(ep1_Co);

  auto wav_ep1_Cg = RunDXTEndpointPipeline(ep1_Cg);


  auto ep2_Y = std::move(std::get<0>(*ep2_YCoCg667));
  auto ep2_Co = std::move(std::get<1>(*ep2_YCoCg667));
  auto ep2_Cg = std::move(std::get<2>(*ep2_YCoCg667));

  auto wav_ep2_Y = RunDXTEndpointPipeline(ep2_Y);

  auto wav_ep2_Co = RunDXTEndpointPipeline(ep2_Co);
  auto wav_ep2_Cg = RunDXTEndpointPipeline(ep2_Cg);


  std::vector<uint8_t> comp_ep1_Y, comp_ep1_C, comp_ep2_Y, comp_ep2_C;
  //Endpoint 1
  wav_ep1_Co->insert(wav_ep1_Co->end(), wav_ep1_Cg->begin(), wav_ep1_Cg->end());
  EntropyEncode(wav_ep1_Y, comp_ep1_Y, false);
  EntropyEncode(wav_ep1_Co, comp_ep1_C, false);
  std::cout << "-------------------------" << std::endl;
  std::cout << "Compress ep1 image: " << comp_ep1_Y.size() + comp_ep1_C.size() << std::endl;
  std::cout << "-------------------------" << std::endl;

  // write Size of Y planes
  // write Y planes data
  uint32_t comp_Y_sz = comp_ep1_Y.size();
  uint32_t comp_C_sz = comp_ep1_C.size();
  
  global_mutex.lock();
  max_compressed_ep_C = std::max(max_compressed_ep_C, comp_C_sz);
  max_compressed_ep_Y = std::max(max_compressed_ep_Y, comp_Y_sz);
  global_mutex.unlock();

  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() + (out_data.size() - 4), reinterpret_cast<uint8_t*>(&comp_Y_sz), 4);
  out_data.resize(out_data.size() + comp_Y_sz, 0);
  memcpy(out_data.data() + (out_data.size() - comp_Y_sz), comp_ep1_Y.data(), comp_Y_sz); 

  // write size of C planes
  // write C planes data
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() + (out_data.size() - 4) , reinterpret_cast<uint8_t*>(&comp_C_sz), 4);
  out_data.resize(out_data.size() + comp_C_sz, 0);
  memcpy(out_data.data() + (out_data.size() - comp_C_sz), comp_ep1_C.data(), comp_C_sz);
  
  // Endpoint 2
  wav_ep2_Co->insert(wav_ep2_Co->end(), wav_ep2_Cg->begin(), wav_ep2_Cg->end());
  EntropyEncode(wav_ep2_Y, comp_ep2_Y, false);
  EntropyEncode(wav_ep2_Co, comp_ep2_C, false);
  std::cout << "-------------------------" << std::endl;
  std::cout << "Compress ep2 image: " << comp_ep2_Y.size() + comp_ep2_C.size() << std::endl;
  std::cout << "-------------------------" << std::endl;

  comp_Y_sz = comp_ep2_Y.size();
  comp_C_sz = comp_ep2_C.size();

  global_mutex.lock();
  max_compressed_ep_C = std::max(max_compressed_ep_C, comp_C_sz);
  max_compressed_ep_Y = std::max(max_compressed_ep_Y, comp_Y_sz);
  global_mutex.unlock();
 // write Size of Y planes
  // write Y planes data
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() + (out_data.size() - 4), reinterpret_cast<uint8_t*>(&comp_Y_sz), 4);
  out_data.resize(out_data.size() + comp_Y_sz, 0);
  memcpy(out_data.data() + (out_data.size() - comp_Y_sz), comp_ep2_Y.data(), comp_Y_sz); 

  // write size of C planes
  // write C planes data
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() + (out_data.size() - 4) , reinterpret_cast<uint8_t*>(&comp_C_sz), 4);
  out_data.resize(out_data.size() + comp_C_sz, 0);
  memcpy(out_data.data() + (out_data.size() - comp_C_sz), comp_ep2_C.data(), comp_C_sz);


/**********************COMMENTINGGG*************************************/
/*  auto endpoint_splitter = (ImageSplit<RGB565>::New());*/

  //std::unique_ptr<RGB565Image> ep1 = std::move(dxt_frame->EndpointOneValues());
  //std::unique_ptr<RGB565Image> ep2 = std::move(dxt_frame->EndpointTwoValues());
  //auto ep1_RGB565  = endpoint_splitter->Run(ep1);
  //auto ep2_RGB565 = endpoint_splitter->Run(ep2);
  
  //auto ep1_R = std::move(std::get<0>(*ep1_RGB565));
  //auto ep1_G = std::move(std::get<1>(*ep1_RGB565));
  //auto ep1_B = std::move(std::get<2>(*ep1_RGB565));

  //auto ep2_R = std::move(std::get<0>(*ep2_RGB565));
  //auto ep2_G = std::move(std::get<1>(*ep2_RGB565));
  //auto ep2_B = std::move(std::get<2>(*ep2_RGB565));

  //uint32_t ep_width = ep1_R->Width();
  //uint32_t ep_height = ep1_R->Height();
///[>*********************************Combined endpoint images *************************<]/
  //std::unique_ptr<RGB565Image> combined_ep(new RGB565Image(2*ep_width, ep_height));
  //for(uint32_t h = 0; h < ep_height; h++) {
    //for(uint32_t w = 0; w < 2*ep_width; w+=2) {
      //combined_ep->SetAt(w, h, ep1->GetAt(w/2, h));
      //combined_ep->SetAt(w + 1, h, ep2->GetAt(w/2, h));
    //}
  //}

  //auto combined_ep_YCoCg667 = initial_endpoint_pipeline->Run(combined_ep);

  //auto combined_ep_Y = std::move(std::get<0>(*combined_ep_YCoCg667));
  //auto combined_ep_Co = std::move(std::get<1>(*combined_ep_YCoCg667));
  //auto combined_ep_Cg = std::move(std::get<2>(*combined_ep_YCoCg667));

  //auto combined_wav_ep_Y = RunDXTEndpointPipeline(combined_ep_Y);
  //auto combined_wav_ep_Co = RunDXTEndpointPipeline(combined_ep_Co);
  //auto combined_wav_ep_Cg = RunDXTEndpointPipeline(combined_ep_Cg);


  //std::vector<uint8_t> combined_comp_ep_Y, combined_comp_ep_C;
  ////Endpoint 1
  //combined_wav_ep_Co->insert(combined_wav_ep_Co->end(), combined_wav_ep_Cg->begin(), combined_wav_ep_Cg->end());
  //EntropyEncode(combined_wav_ep_Y, combined_comp_ep_Y, false);
  //EntropyEncode(combined_wav_ep_Co, combined_comp_ep_C, false);
  //std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
  //std::cout << "Combined Compressed endpoint images: " << combined_comp_ep_Y.size() + combined_comp_ep_C.size() << std::endl;
  //std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;

///[>************************************************** *************************<]/


///[>***********************************************************RGB delats with wavelet ************************<]/
  //std::unique_ptr<SixBitImage> delta_r(new SixBitImage(ep_width, ep_height));
  //std::unique_ptr<SevenBitImage> delta_g(new SevenBitImage(ep_width, ep_height));
  //std::unique_ptr<SixBitImage> delta_b(new SixBitImage(ep_width, ep_height));

  //std::vector<uint8_t> comp_delta_r, comp_delta_g, comp_delta_b;
  //for(uint32_t h = 0; h < ep1_R->Height(); h++) {
    //for(uint32_t w = 0; w < ep1_R->Width(); w++) {
      //delta_r->SetAt(w, h, ep1_R->GetAt(w, h) - ep2_R->GetAt(w, h) + 32);
      //delta_g->SetAt(w, h, ep1_G->GetAt(w, h) - ep2_G->GetAt(w, h) + 64);
      //delta_b->SetAt(w, h, ep1_B->GetAt(w, h) - ep2_B->GetAt(w, h) + 32);
    //}
  //}
  //auto wav_delta_r = RunDXTEndpointPipeline(delta_r);
  //auto wav_delta_g = RunDXTEndpointPipeline(delta_g);
  //auto wav_delta_b = RunDXTEndpointPipeline(delta_b);
  ////Combine wav_coefficients
  //wav_delta_r->insert(wav_delta_r->end(), wav_delta_g->begin(), wav_delta_g->end());
  //wav_delta_r->insert(wav_delta_r->end(), wav_delta_b->begin(), wav_delta_b->end());
  //EntropyEncode(wav_delta_r, comp_delta_r, false);
  ////EntropyEncode(wav_delta_g, comp_delta_g, false);
  ////EntropyEncode(wav_delta_b, comp_delta_b, false);
  //std::cout << "--------------------------------" << std::endl;
  //std::cout << "Compressed Deltas size: " << comp_delta_r.size() + comp_delta_g.size() + comp_delta_b.size() << std::endl;
  //std::cout << "---------------------------------" << std::endl;
///[>**************************************************************************************************************<]/




  //*********************************Verify Endpoint compresssion*************************//
  //**************************************************************************************//
  //*************************************************************************************//
/*  std::unique_ptr<std::vector<uint8_t> > out_wav_ep1_Y(new std::vector<uint8_t>()), out_wav_ep1_C(new std::vector<uint8_t>());*/
  //out_wav_ep1_Y->resize(wav_ep1_Y->size()), out_wav_ep1_C->resize(wav_ep1_Co->size());
  //EntropyDecode(comp_ep1_Y, (*out_wav_ep1_Y), false);
  //EntropyDecode(comp_ep1_C, (*out_wav_ep1_C), false);
  //auto out_ep1_Y = RunInverseDXTEndpointPipeline< UnsignedBits<6> >(out_wav_ep1_Y, ep_width, ep_height);
  //for(uint32_t h = 0; h < ep_height; h++) {
    //for(uint32_t w = 0; w < ep_width; w++) {
      //auto pix1 = ep1_Y->GetAt(w, h);
      //auto pix2 = out_ep1_Y->GetAt(w, h);
      //assert(pix1 == pix2);
    //}
  //}
  //size_t temp_size = out_wav_ep1_C->size();
  //std::cout << temp_size << std::endl;
  //std::unique_ptr<std::vector<uint8_t> > out_wav_ep1_Co(new std::vector<uint8_t>());
  //std::unique_ptr<std::vector<uint8_t> > out_wav_ep1_Cg(new std::vector<uint8_t>());
  ////out_wav_ep1_Co->resize(out_wav_ep1_C->size()/2);
  ////out_wav_ep1_Cg->resize(out_wav_ep1_C->size()/2);
  //out_wav_ep1_Co->insert(out_wav_ep1_Co->begin(), out_wav_ep1_C->begin(), out_wav_ep1_C->begin() + out_wav_ep1_C->size() /2);
  //out_wav_ep1_Cg->insert(out_wav_ep1_Cg->begin(), out_wav_ep1_C->begin() + out_wav_ep1_C->size() /2, out_wav_ep1_C->end());
  //auto out_ep1_Co = RunInverseDXTEndpointPipeline< SignedBits<6> >(out_wav_ep1_Co, ep_width, ep_height);
  //for(uint32_t h = 0; h < ep_height; h++) {
    //for(uint32_t w = 0; w < ep_width; w++) {
      //auto pix1 = ep1_Co->GetAt(w, h);
      //auto pix2 = out_ep1_Co->GetAt(w, h);
      //assert(pix1 == pix2);
    //}
  //}

  //auto out_ep1_Cg = RunInverseDXTEndpointPipeline<SignedBits<7> >(out_wav_ep1_Cg, ep_width, ep_height);
  //for(uint32_t h = 0; h < ep_height; h++) {
    //for(uint32_t w = 0; w < ep_width; w++) {
      //auto pix1 = ep1_Cg->GetAt(w, h);
      //auto pix2 = out_ep1_Cg->GetAt(w, h);
      //assert(pix1 == pix2);
    //}
  //}

  
/*  uint32_t W = ep1_Y->Width(); */
  //uint32_t H = ep1_Y->Height();
  ////Compress deltas YCoCg deltas *************************************************8
  //std::unique_ptr<SigSevenBitImage> delta_Y( new SigSevenBitImage(W, H));
  //std::unique_ptr<SigSevenBitImage> delta_Cg( new SigSevenBitImage(W, H));
  //std::unique_ptr<SigEightBitImage> delta_Co( new SigEightBitImage(W, H));

  //for(uint32_t h = 0; h < H; h++) {
    //for(uint32_t w = 0; w < W; w++) {
      //delta_Y->SetAt(w, h, ep1_Y->GetAt(w, h) - ep2_Y->GetAt(w, h));
      //delta_Cg->SetAt(w, h, ep1_Cg->GetAt(w, h) - ep2_Cg->GetAt(w, h));
      //delta_Co->SetAt(w, h, ep1_Co->GetAt(w, h) - ep2_Co->GetAt(w, h));
    //}
  //}
  //auto wav_delta_Y = RunDXTEndpointPipeline(delta_Y);
  //auto wav_delta_Co = RunDXTEndpointPipeline(delta_Co);
  //auto wav_delta_Cg = RunDXTEndpointPipeline(delta_Cg);

  //std::vector<uint8_t> cmp_wav_delta_Y, cmp_wav_delta_C;
  //wav_delta_Co->insert(wav_delta_Co->end(), wav_delta_Cg->begin(), wav_delta_Cg->end());
  
  //EntropyEncode(wav_delta_Y, cmp_wav_delta_Y, false);
  //EntropyEncode(wav_delta_Co, cmp_wav_delta_C, false);

  //std::cout << "-------------------------" << std::endl;
  //std::cout << "Compress delta image: " << cmp_wav_delta_Y.size() + cmp_wav_delta_C.size() << std::endl;
  //std::cout << "-------------------------" << std::endl;



/*  std::unique_ptr<GreyImage> delta_Y(new GreyImage(W, H));*/
  //std::unique_ptr<GreyImage> delta_Cg(new GreyImage(W, H));
  //std::unique_ptr<GreyImage> delta_Co(new GreyImage(W, H));
  //std::vector<int32_t> test_delta_values1, test_delta_values2, test_delta_values3;
  //for(uint32_t h = 0; h < ep1->Height(); h++){
    //for(uint32_t w = 0; w < ep1->Width(); w++) {
      ////6 bits for Y6
      //test_delta_values1.push_back(static_cast<int32_t>(ep1_Y->GetAt(w, h) - ep2_Y->GetAt(w, h)));
      //delta_Y->SetAt(w, h, static_cast<uint8_t>(ep1_Y->GetAt(w, h) - ep2_Y->GetAt(w, h) + 63));
      ////6 bits for Co6
      //test_delta_values2.push_back(static_cast<int32_t>(ep1_Co->GetAt(w, h) - ep2_Co->GetAt(w, h)));
     //delta_Co->SetAt(w, h, static_cast<uint8_t>(ep1_Co->GetAt(w, h) - ep2_Co->GetAt(w, h) + 63));
      //test_delta_values2.push_back(static_cast<int32_t>(ep1_Co->GetAt(w, h) - ep2_Co->GetAt(w, h)));

      ////7 bits for Cg7
      //delta_Cg->SetAt(w, h, static_cast<uint8_t>(ep1_Cg->GetAt(w, h) - ep2_Cg->GetAt(w, h) + 127));

    //}
  //}
  
  //auto wav_delta_Y = RunDXTEndpointPipeline_16bit(delta_Y);
  //auto wav_delta_Co = RunDXTEndpointPipeline_16bit(delta_Co);
  //auto wav_delta_Cg = RunDXTEndpointPipeline_16bit(delta_Cg);

  //std::vector<uint8_t> cmp_wav_delta_Y, cmp_wav_delta_C;
  //wav_delta_Co->insert(wav_delta_Co->end(), wav_delta_Cg->begin(), wav_delta_Cg->end());
  
  //EntropyEncode(wav_delta_Y, cmp_wav_delta_Y, false);
  //EntropyEncode(wav_delta_Co, cmp_wav_delta_C, false);


/*  auto ep1_planes = initial_endpoint_pipeline->Run(ep1);*/
  //auto ep1_y_cmp = RunDXTEndpointPipeline(std::get<0>(*ep1_planes));

 
  //auto ep1_y_plane = std::move(std::get<0>(*ep1_planes));
  //auto test_ep1_y_plane = RunInverseDXTEndpointPipeline< UnsignedBits<6> >(ep1_y_cmp, ep1_y_plane->Width(), ep1_y_plane->Height());


 //// Testing inverse pipeline for Signed bitsss!!
  //auto ep1_co_cmp = RunDXTEndpointPipeline(std::get<1>(*ep1_planes));
  //auto ep1_co_plane = std::move(std::get<1>(*ep1_planes));

  //auto ep1_cg_cmp = RunDXTEndpointPipeline(std::get<2>(*ep1_planes));
  //auto ep1_cg_plane = std::move(std::get<2>(*ep1_planes));
    //// Concatenate Y planes
  //std::vector<uint8_t> y_planes;
  //EntropyEncode(ep1_y_cmp, y_planes, false);

  //// Concatenate Chroma planes
  //ep1_co_cmp->insert(ep1_co_cmp->end(), ep1_cg_cmp->begin(), ep1_cg_cmp->end());
  //std::vector<uint8_t> chroma_planes;
  //EntropyEncode(ep1_co_cmp, chroma_planes, false);
  //std::cout <<"**********************************************************************" << std::endl;
  //std::cout << "Total bytes of Endpoints:" << y_planes.size() + chroma_planes.size() << std::endl;
  /*std::cout <<"**********************************************************************" << std::endl;*/

}


//Entropy Encode different symbols in DXTImage Reencoded class and return the data
void EntropyEncode(std::unique_ptr<MPTC::DXTImage> &dxt_frame, std::vector<uint8_t> &out_data) {
  // This might have to change for lager files
  //Encode the motion indices
  uint32_t indices_bytes, mask_bytes, total_bytes;
  std::vector<uint8_t> compressed_motion_indices(max_bytes,0);
  entropy::Arithmetic_Codec arith_data_encoder(max_bytes, compressed_motion_indices.data()); 
  entropy::Adaptive_Data_Model data_model(257);
  arith_data_encoder.start_encoder();
  assert(dxt_frame->_motion_indices.size() == dxt_frame->_num_blocks);
  for(auto a : dxt_frame->_motion_indices) {
    arith_data_encoder.encode(std::get<0>(a), data_model);
    arith_data_encoder.encode(std::get<1>(a), data_model);
  }
  arith_data_encoder.stop_encoder();
  indices_bytes = arith_data_encoder.get_num_bytes();

#ifndef NDEBUG
  std::cout << "compressed motion indices size: " << indices_bytes << std::endl;
#endif

  total_bytes = dxt_frame->_unique_palette.size() * 4 + indices_bytes;
#ifndef NDEBUG
  std::cout << "Total bytes: " << total_bytes << std::endl;
#endif
  //copy data to out_data
  //1.size of unique indices
  uint32_t num_unique = dxt_frame->_unique_palette.size();
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data()+(out_data.size()-4), reinterpret_cast<uint8_t*>(&num_unique), 4);
  
  //4.size of compressed motion indices
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() + (out_data.size() - 4), reinterpret_cast<uint8_t*>(&indices_bytes), 4);

  global_mutex.lock();
  max_compressed_motion_indices = std::max(max_compressed_motion_indices, indices_bytes);  
  global_mutex.unlock();
  //6.compressed motion_index data
  out_data.resize(out_data.size() + indices_bytes, 0);
  memcpy(out_data.data() + (out_data.size() - indices_bytes), compressed_motion_indices.data(), indices_bytes);

  CompressEndpoint(dxt_frame, out_data);

  return; 
}


void DecompressMultiUnique(const std::string input_file,
                           const std::string out_dir,
			   const std::string rgb_dir) {

  std::ifstream in_stream;
  in_stream.open(input_file.c_str(), std::ifstream::binary);

  if(!in_stream.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    exit(-1);
  }
  uint32_t frame_height, frame_width, total_frame_count;
  uint8_t unique_interval, search_area;
  in_stream.read(reinterpret_cast<char*>(&frame_height), 4);
  in_stream.read(reinterpret_cast<char*>(&frame_width), 4);
  uint32_t num_blocks = (frame_height/4 * frame_width/4);
  in_stream.read(reinterpret_cast<char*>(&unique_interval), 1);
  in_stream.read(reinterpret_cast<char*>(&search_area), 1);
  in_stream.read(reinterpret_cast<char*>(&total_frame_count), 4);
  in_stream.read(reinterpret_cast<char*>(&max_unique_count), 4);
  in_stream.read(reinterpret_cast<char*>(&max_compressed_palette), 4);
  in_stream.read(reinterpret_cast<char*>(&max_compressed_motion_indices), 4);
  in_stream.read(reinterpret_cast<char*>(&max_compressed_ep_Y), 4);
  in_stream.read(reinterpret_cast<char*>(&max_compressed_ep_C), 4);

  std::unique_ptr<DXTImage> null_dxt(nullptr);
  std::unique_ptr<DXTImage> prev_frame(nullptr), curr_frame(nullptr), first_frame(nullptr);
  uint32_t frame_number = 0;
  std::vector<uint8_t> compressed_combined_palette;
  for(uint32_t curr_frame_idx = 0; curr_frame_idx < total_frame_count; curr_frame_idx++) {
    compressed_combined_palette.clear();
    uint32_t compressed_palette_size, unique_count, unique_idx_offset = 0;
    in_stream.read(reinterpret_cast<char*>(&compressed_palette_size), 4);
    compressed_combined_palette.resize(compressed_palette_size);
    in_stream.read(reinterpret_cast<char*>(compressed_combined_palette.data()), compressed_palette_size);
    in_stream.read(reinterpret_cast<char*>(&unique_count), 4);
    std::vector<uint8_t> combined_8bit_palette(unique_count);
    EntropyDecode(compressed_combined_palette, combined_8bit_palette, false);
     
    for(uint8_t curr_idx = 0; curr_idx < unique_interval; curr_idx++) {
      uint8_t intra;
      frame_number++;
      bool is_intra = false;
      // read 4 bytes which give total unique indices count
      uint32_t num_unique;
      in_stream.read(reinterpret_cast<char*>(&num_unique), 4);
      // read 4 bytes which give size of compressed motion indices
      uint32_t motion_indices_bytes;
      in_stream.read(reinterpret_cast<char*>(&motion_indices_bytes), 4);
     // allocate memory for unique indices values and read unique indices bytes
      std::vector<uint32_t> unique_indices(num_unique, 0);
      memcpy(unique_indices.data(), combined_8bit_palette.data() + unique_idx_offset, 4 * num_unique);
      unique_idx_offset += 4*num_unique;
     // allocate memory for motion_index data and read compressed motion indices
      std::vector<uint8_t> compressed_motion_indices(motion_indices_bytes, 0);
      in_stream.read(reinterpret_cast<char*>(compressed_motion_indices.data()), motion_indices_bytes);


     //Decompress the motion indices uisng arithmetic decoder
      uint32_t num_motion_indices = num_blocks;
      std::vector<uint8_t> motion_indices(2 * num_motion_indices, 0);
      EntropyDecode(compressed_motion_indices, motion_indices, false);
      std::vector< std::tuple<uint8_t, uint8_t> > out_motion_indices;

      for(size_t ii = 0; ii < motion_indices.size(); ii+=2) 
        out_motion_indices.push_back(std::make_tuple(motion_indices[ii], motion_indices[ii + 1]));
         
      // Populate the physical and logical blocks using the data
      uint32_t comp_Y_sz;
      uint32_t comp_C_sz;
      std::vector<uint8_t> comp_ep1_Y, comp_ep1_C;
      std::unique_ptr<std::vector<uint8_t> > wav_ep1_Y(new std::vector<uint8_t>());
      std::unique_ptr<std::vector<uint8_t> > wav_ep1_C(new std::vector<uint8_t>());
      wav_ep1_Y->resize(num_blocks); wav_ep1_C->resize(2 * num_blocks);
      in_stream.read(reinterpret_cast<char*>(&comp_Y_sz), 4);
      comp_ep1_Y.resize(comp_Y_sz);
      in_stream.read(reinterpret_cast<char*>(comp_ep1_Y.data()), comp_Y_sz);
      EntropyDecode(comp_ep1_Y, *wav_ep1_Y, false);

      in_stream.read(reinterpret_cast<char*>(&comp_C_sz), 4);
      comp_ep1_C.resize(comp_C_sz);
      in_stream.read(reinterpret_cast<char*>(comp_ep1_C.data()), comp_C_sz);
      EntropyDecode(comp_ep1_C, *wav_ep1_C, false);


      std::vector<uint8_t> comp_ep2_Y, comp_ep2_C;
      std::unique_ptr<std::vector<uint8_t> > wav_ep2_Y(new std::vector<uint8_t>());
      std::unique_ptr<std::vector<uint8_t> > wav_ep2_C(new std::vector<uint8_t>());
      wav_ep2_Y->resize(num_blocks); wav_ep2_C->resize(2 * num_blocks);

      in_stream.read(reinterpret_cast<char*>(&comp_Y_sz), 4);
      comp_ep2_Y.resize(comp_Y_sz);
      in_stream.read(reinterpret_cast<char*>(comp_ep2_Y.data()), comp_Y_sz);
      EntropyDecode(comp_ep2_Y, *wav_ep2_Y, false);

      in_stream.read(reinterpret_cast<char*>(&comp_C_sz), 4);
      comp_ep2_C.resize(comp_C_sz);
      in_stream.read(reinterpret_cast<char*>(comp_ep2_C.data()), comp_C_sz);
      EntropyDecode(comp_ep2_C, *wav_ep2_C, false); 


      curr_frame.reset(new DXTImage(frame_width, frame_height, is_intra, unique_indices));

      ReconstructDXTData(unique_indices, 
	                 out_motion_indices, 
			 curr_frame, 
			 prev_frame, 
	                 search_area, 
			 rgb_dir, 
			 frame_number);

      ReconstructEndPoints(curr_frame,
			   wav_ep1_Y,
			   wav_ep1_C,
			   wav_ep2_Y,
			   wav_ep2_C);
 /*  for(int i = 0; i < num_blocks; i++) {*/
      //std::bitset<32> x(curr_frame->_physical_blocks[i].interp);
////    std::cout<< "IDX: " << i << "  " << "Ep1: " << curr_frame->_physical_blocks[i].ep1 << "  " << "Ep2: " << curr_frame->_physical_blocks[i].ep2 << "  " << "Interp: " << x << std::endl;
     //int a = 10 + 20;
   //}

      curr_frame->SetLogicalBlocks(); 
     
      std::string out_frame = out_dir + "/" + std::to_string(frame_number) + ".png";
#ifndef NDEBUG  
      std::unique_ptr<RGBImage> recons_image = std::move(curr_frame->DecompressedImage());  
      stbi_write_png(out_frame.c_str(), recons_image->Width(), recons_image->Height(),
                   3, recons_image->Pack().data(), 3*recons_image->Width());
#endif
      prev_frame = std::move(curr_frame);

    }
  }

  if(in_stream.eof()) {
    std::cout << "End of File Reached!" << std::endl;
  }
  in_stream.close();
}






void CompressMultiUnique(const std::string dir_name, const std::string out_file, 
                         uint32_t search_area, int32_t vErrThreshold, uint32_t intra_interval, 
			 uint32_t unique_interval, std::string ep_dir) {

  DIR *dir = opendir(dir_name.c_str());
  if (!dir) {
    std::cerr<< "Error opening directory: " << dir_name << std::endl;
    exit(-1);
  }

  std::string first_file, curr_file, prev_file;
  struct dirent *entry = NULL;
  uint32_t frame_number = 0; 
  // skip '.' and '..' 
  std::vector<std::string> file_names;
  while ((entry = readdir(dir)) != NULL) {
    if (strlen(entry->d_name) == 1 && strncmp(entry->d_name, ".", 1) == 0) continue;
    else if (strlen(entry->d_name) == 2 && strncmp(entry->d_name, "..", 2) == 0) continue;
    else file_names.push_back(entry->d_name);
  }
  std::sort(file_names.begin(), file_names.end());

  uint32_t curr_intra_num = 1, curr_unique_num = 1;
#ifndef NDEBUG
  for(auto a : file_names)
    std::cout << a << std::endl;
#endif
  // read the first file name
  //entry = readdir(dir);

  std::unique_ptr<MPTC::DXTImage> null_dxt(nullptr);
  std::unique_ptr<MPTC::DXTImage> curr_frame(nullptr), prev_frame(nullptr);

  curr_file = dir_name + "/" + file_names[0];
  std::unique_ptr<MPTC::DXTImage> first_frame(new MPTC::DXTImage(curr_file, true, search_area, vErrThreshold));
  curr_file.clear();
  uint32_t frame_count = 1;
  uint32_t frame_height, frame_width, unique_count = 0;
  frame_width = first_frame->Width();
  frame_height = first_frame->Height();

  //open out file for writing to it
  std::ofstream out_stream(out_file.c_str(), std::ofstream::binary);
  // Write out frame height and frame width
  // Write the ---unique interval value--- 

  uint8_t unique_int = static_cast<uint8_t>(unique_interval);
  uint8_t search_area_8bit = static_cast<uint8_t>(search_area);
  //total frame count
  uint32_t total_frame_count = file_names.size() / unique_interval;
  out_stream.write(reinterpret_cast<const char*>(&frame_height), 4);
  out_stream.write(reinterpret_cast<const char*>(&frame_width), 4);
  out_stream.write(reinterpret_cast<const char*>(&unique_int), 1);
  out_stream.write(reinterpret_cast<const char*>(&search_area), 1);
  out_stream.write(reinterpret_cast<const char*>(&total_frame_count), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_unique_count), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_palette), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_motion_indices), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_Y), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_C), 4);

  out_stream.flush();

  // out_data to be written to file
  std::vector<uint8_t> out_data;
  // copyt height and widht 
  // copy the size of the mask index
  // copy mask index data
  // copy size of of motion indices
  // copy unique_indices
  // copy data of motion indices
  bool set_intra = true;
  std::vector<uint8_t> combined_8bit_palette;
  std::vector<uint8_t> combined_motion_data;
  std::vector<uint8_t> compressed_combined_motion_data;
  for(auto a : file_names) {
    frame_number++;
    curr_file.clear();
    curr_file = dir_name + "/" + a;

    curr_frame.reset(new DXTImage(curr_file, set_intra, search_area, vErrThreshold));
    std::cout << "Frame Name:" << a << std::endl;
#ifndef NDEBUG
    std::cout << "PSNR before reencoding: " << curr_frame->PSNR() << std::endl;
#endif

    curr_frame->Reencode(prev_frame, -1);
#ifndef NDEBUG
    std::cout << "PSNR after reencoding: " << curr_frame->PSNR() << std::endl;
    std::cout << "Total blocks found: " << curr_frame->_motion_indices.size() << std::endl;
    std::cout << "Inter pixel blocks:" << curr_frame->_inter_pixel_motion_indices.size() << std::endl;
#endif


  std::unique_ptr<RGBImage> ep1 = std::move(curr_frame->EndpointOneImage());
  std::unique_ptr<RGBImage> ep2 = std::move(curr_frame->EndpointTwoImage());
  size_t W = ep1->Width();
  size_t H = ep1->Height();
  std::string ep_file1, ep_file2;
  ep_file1.clear();
  ep_file2.clear();
  ep_file1 = ep_dir + "/1/" + std::to_string(frame_number) + ".png";
  ep_file2 = ep_dir + "/2/" + std::to_string(frame_number) + ".png";
  stbi_write_png(ep_file1.c_str(), W, H, 3, ep1->Pack().data(), 3*W);
  stbi_write_png(ep_file2.c_str(), W, H, 3, ep2->Pack().data(), 3*W);
  /*std::unique_ptr<RGB565Image> ep1 = std::move(curr_frame->EndpointOneValues());*/
  //std::unique_ptr<RGB565Image> ep2 = std::move(curr_frame->EndpointTwoValues());
  //size_t W = ep1->Width();
  //size_t H = ep1->Height();

  //std::string ep_file1, ep_file2;
  //ep_file1.clear();
  //ep_file1 = ep_dir + "/1/" + std::to_string(frame_number) + ".raw";

  //ep_file2.clear();
  //ep_file2 = ep_dir + "/2/" + std::to_string(frame_number) + ".raw";



  //entropy::ContainedBitWriter writer1;
  //entropy::ContainedBitWriter writer2;

  ////std::cout << "Width: " << W << std::endl;
  ////std::cout << "Height: " << H << std::endl;
  //for(size_t h = 0; h < ep1->Height(); h++) {
    //for(size_t w = 0; w < ep1->Width(); w++) {
      //auto pix1 = ep1->GetAt(w, h);
      //auto pix2 = ep2->GetAt(w, h);
      //uint8_t r = std::get<0>(pix1);
      //uint8_t g = std::get<1>(pix1);
      //uint8_t b = std::get<2>(pix1);
      //writer1.WriteBits(r, 5);
      //writer1.WriteBits(g, 6);
      //writer1.WriteBits(b, 5);
      //r = std::get<0>(pix2);
      //g = std::get<1>(pix2);
      //b = std::get<2>(pix2);
      //writer2.WriteBits(r, 5);
      //writer2.WriteBits(g, 6);
      //writer2.WriteBits(b, 5);
    //}
  //}

  //std::vector<uint8_t> endpoint_data1 = std::move(writer1.GetData());
  //std::ofstream endpoint_stream1(ep_file1.c_str(), std::ofstream::binary);
  //endpoint_stream1.write(reinterpret_cast<const char*>(endpoint_data1.data()), endpoint_data1.size());

  //std::vector<uint8_t> endpoint_data2 = std::move(writer2.GetData());
  //std::ofstream endpoint_stream2(ep_file2.c_str(), std::ofstream::binary);
  //endpoint_stream2.write(reinterpret_cast<const char*>(endpoint_data2.data()), endpoint_data2.size());

  
    EntropyEncode(curr_frame, compressed_combined_motion_data); 
    if(curr_intra_num <= intra_interval){
      set_intra = false;
      if(curr_intra_num == intra_interval) {
	set_intra = true;
	curr_intra_num = 1;
      }
      else curr_intra_num++;
   }
   
   if(curr_unique_num <= unique_interval) {

     std::vector<uint8_t> temp_palette_8bit = curr_frame->Get8BitPalette();
     combined_8bit_palette.insert(std::end(combined_8bit_palette), std::begin(temp_palette_8bit), std::end(temp_palette_8bit));
     unique_count += static_cast<uint32_t>(temp_palette_8bit.size());

     if(curr_unique_num == unique_interval) {
       std::vector<uint8_t> compressed_combined_8bit_palette;
       EntropyEncode(combined_8bit_palette, compressed_combined_8bit_palette, false);
#ifndef NDEBUG
       std::cout << "--------------------------" << std::endl;
       std::cout << "Uncompressed combined Palette:" << combined_8bit_palette.size() << std::endl;
       std::cout << "Compressed combined Palette:" << compressed_combined_8bit_palette.size() << std::endl;
       std::cout << "-------------------------" << std::endl;
#endif
       uint32_t compressed_palette_size = compressed_combined_8bit_palette.size();
       out_stream.write(reinterpret_cast<const char*>(&compressed_palette_size), 4);
       
       max_compressed_palette = std::max(max_compressed_palette, compressed_palette_size);
       out_stream.write(reinterpret_cast<const char*>(compressed_combined_8bit_palette.data()), compressed_combined_8bit_palette.size());

       // Unique count size of all the indices in uncompressed Dictionary
       out_stream.write(reinterpret_cast<const char*>(&unique_count), 4);
       max_unique_count = std::max(max_unique_count, unique_count);
       out_stream.write(reinterpret_cast<const char*>(compressed_combined_motion_data.data()), compressed_combined_motion_data.size());
       out_stream.flush();
       compressed_combined_motion_data.clear();
       compressed_combined_8bit_palette.clear(); 
       combined_8bit_palette.clear();
       unique_count = 0;
       curr_unique_num = 1;
     }
     else curr_unique_num++;

   }
    prev_frame = std::move(curr_frame);
  }
#ifndef NDEBUG
  std::cout << "Number of total bytes written:" << out_stream.tellp() << std::endl;
#endif 
  // go to top of the stream and the move 14 bytes forward and the max values in the order requi ed
  out_stream.seekp(0, out_stream.beg);
  out_stream.seekp(14);
  out_stream.write(reinterpret_cast<const char*>(&max_unique_count), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_palette), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_motion_indices), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_Y), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_C), 4);
#ifndef NDEBUG
  std::cout << "Max Values!" << std::endl;
  std::cout << max_unique_count << std::endl;
  std::cout << max_compressed_palette << std::endl;
  std::cout << max_compressed_motion_indices << std::endl;
  std::cout << max_compressed_ep_Y << std::endl;
  std::cout << max_compressed_ep_C << std::endl;
  std::cout << "!!!!!!!!!!!" << std::endl;
#endif
 
  out_stream.close();
}


void SingleThreadCompressMulti(const std::string dir_name,
                               const std::vector<std::string> &file_names,
                               const uint32_t index,
			       const uint32_t count,
			       uint32_t search_area,
			       int32_t vErrThreshold,
			       std::vector<uint8_t> &out,
			       std::string ep_dir) {

  std::unique_ptr<MPTC::DXTImage> null_dxt(nullptr);
  std::unique_ptr<MPTC::DXTImage> curr_frame(nullptr), prev_frame(nullptr);
  
  uint32_t frame_number = 0;
  std::string first_file, curr_file, prev_file;

  curr_file = dir_name + "/" + file_names[0];

  std::unique_ptr<MPTC::DXTImage> first_frame(new MPTC::DXTImage(curr_file, 
	                                                         true, 
								 search_area, 
								 vErrThreshold));

  curr_file.clear();
  uint32_t frame_count = 1;
  uint32_t frame_height, frame_width, unique_count = 0;
  frame_width = first_frame->Width();
  frame_height = first_frame->Height();

  std::vector<uint8_t> combined_8bit_palette;
  std::vector<uint8_t> combined_motion_data;
  std::vector<uint8_t> compressed_combined_motion_data;
  bool set_intra = true;

  for(uint32_t idx = index; idx < index + count; idx++) {

    frame_number++;
    curr_file.clear();
    curr_file = dir_name + "/" + file_names[idx];
    if(frame_number > 2) {
      set_intra = false;
    }
    curr_frame.reset(new DXTImage(curr_file, set_intra, search_area, vErrThreshold));
    std::cout << "Frame Name:" << file_names[idx] << std::endl;
#ifndef NDEBUG
    std::cout << "PSNR before reencoding: " << curr_frame->PSNR() << std::endl;
#endif

    curr_frame->Reencode(prev_frame, -1);
#ifndef NDEBUG
    std::cout << "PSNR after reencoding: " << curr_frame->PSNR() << std::endl;
    std::cout << "Total blocks found: " << curr_frame->_motion_indices.size() << std::endl;
    std::cout << "Inter pixel blocks:" << curr_frame->_inter_pixel_motion_indices.size() << std::endl;
#endif


/*  std::unique_ptr<RGBImage> ep1 = std::move(curr_frame->EndpointOneImage());*/
  //std::unique_ptr<RGBImage> ep2 = std::move(curr_frame->EndpointTwoImage());
  //size_t W = ep1->Width();
  //size_t H = ep1->Height();
  //std::string ep_file1, ep_file2;
  //ep_file1.clear();
  //ep_file2.clear();
  //ep_file1 = ep_dir + "/1/" + std::to_string(frame_number) + ".png";
  //ep_file2 = ep_dir + "/2/" + std::to_string(frame_number) + ".png";
  //stbi_write_png(ep_file1.c_str(), W, H, 3, ep1->Pack().data(), 3*W);
  /*stbi_write_png(ep_file2.c_str(), W, H, 3, ep2->Pack().data(), 3*W);*/
  /*std::unique_ptr<RGB565Image> ep1 = std::move(curr_frame->EndpointOneValues());*/
  //std::unique_ptr<RGB565Image> ep2 = std::move(curr_frame->EndpointTwoValues());
  //size_t W = ep1->Width();
  //size_t H = ep1->Height();

  //std::string ep_file1, ep_file2;
  //ep_file1.clear();
  //ep_file1 = ep_dir + "/1/" + std::to_string(frame_number) + ".raw";

  //ep_file2.clear();
  //ep_file2 = ep_dir + "/2/" + std::to_string(frame_number) + ".raw";



  //entropy::ContainedBitWriter writer1;
  //entropy::ContainedBitWriter writer2;

  ////std::cout << "Width: " << W << std::endl;
  ////std::cout << "Height: " << H << std::endl;
  //for(size_t h = 0; h < ep1->Height(); h++) {
    //for(size_t w = 0; w < ep1->Width(); w++) {
      //auto pix1 = ep1->GetAt(w, h);
      //auto pix2 = ep2->GetAt(w, h);
      //uint8_t r = std::get<0>(pix1);
      //uint8_t g = std::get<1>(pix1);
      //uint8_t b = std::get<2>(pix1);
      //writer1.WriteBits(r, 5);
      //writer1.WriteBits(g, 6);
      //writer1.WriteBits(b, 5);
      //r = std::get<0>(pix2);
      //g = std::get<1>(pix2);
      //b = std::get<2>(pix2);
      //writer2.WriteBits(r, 5);
      //writer2.WriteBits(g, 6);
      //writer2.WriteBits(b, 5);
    //}
  //}

  //std::vector<uint8_t> endpoint_data1 = std::move(writer1.GetData());
  //std::ofstream endpoint_stream1(ep_file1.c_str(), std::ofstream::binary);
  //endpoint_stream1.write(reinterpret_cast<const char*>(endpoint_data1.data()), endpoint_data1.size());

  //std::vector<uint8_t> endpoint_data2 = std::move(writer2.GetData());
  //std::ofstream endpoint_stream2(ep_file2.c_str(), std::ofstream::binary);
  //endpoint_stream2.write(reinterpret_cast<const char*>(endpoint_data2.data()), endpoint_data2.size());

  
    EntropyEncode(curr_frame, compressed_combined_motion_data); 
     std::vector<uint8_t> temp_palette_8bit = curr_frame->Get8BitPalette();
     combined_8bit_palette.insert(std::end(combined_8bit_palette), std::begin(temp_palette_8bit),                                  std::end(temp_palette_8bit));
     unique_count += static_cast<uint32_t>(temp_palette_8bit.size());

    prev_frame = std::move(curr_frame);
  }

       std::vector<uint8_t> compressed_combined_8bit_palette;
       EntropyEncode(combined_8bit_palette, compressed_combined_8bit_palette, false);
#ifndef NDEBUG
       std::cout << "--------------------------" << std::endl;
       std::cout << "Uncompressed combined Palette:" << combined_8bit_palette.size() << std::endl;
       std::cout << "Compressed combined Palette:" << compressed_combined_8bit_palette.size() << std::endl;
       std::cout << "-------------------------" << std::endl;
#endif
       uint32_t compressed_palette_size = compressed_combined_8bit_palette.size();
       out.resize(out.size() + 4, 0);
       memcpy(out.data() + (out.size() - 4),
	          reinterpret_cast<const char*>(&compressed_palette_size), 
		  4);
       global_mutex.lock();
       max_compressed_palette = std::max(max_compressed_palette, compressed_palette_size);
       global_mutex.unlock();
       
       out.resize(out.size() + compressed_combined_8bit_palette.size(), 0);
       memcpy(out.data() + (out.size() - compressed_combined_8bit_palette.size()),
	          reinterpret_cast<const char*>(compressed_combined_8bit_palette.data()), 
		  compressed_combined_8bit_palette.size());

       // Unique count size of all the indices in uncompressed Dictionary
       out.resize(out.size() + 4, 0);
       memcpy(out.data() + (out.size() - 4),
	          reinterpret_cast<const char*>(&unique_count), 
		  4);

       global_mutex.lock();
       max_unique_count = std::max(max_unique_count, unique_count);
       global_mutex.unlock();
       out.resize(out.size() + compressed_combined_motion_data.size(), 0);
       memcpy( out.data() + (out.size() - compressed_combined_motion_data.size()),
	          reinterpret_cast<const char*>(compressed_combined_motion_data.data()), 
		  compressed_combined_motion_data.size());
 


}

void ThreadedCompressMultiUnique(const std::string dir_name,
                                 const std::string out_file,
				 uint32_t search_area,
				 int32_t vErrThreshold,
				 uint32_t intra_interval,
				 uint32_t unique_interval,
				 uint32_t wavelet_block_sz,
				 std::string ep_dir,
				 uint32_t thread_count) {

    
  DIR *dir = opendir(dir_name.c_str());
  if (!dir) {
    std::cerr<< "Error opening directory: " << dir_name << std::endl;
    exit(-1);
  }

  std::string first_file, curr_file, prev_file;
  struct dirent *entry = NULL;
  uint32_t frame_number = 0; 
  // skip '.' and '..' 
  std::vector<std::string> file_names;
  while ((entry = readdir(dir)) != NULL) {
    if (strlen(entry->d_name) == 1 && strncmp(entry->d_name, ".", 1) == 0) continue;
    else if (strlen(entry->d_name) == 2 && strncmp(entry->d_name, "..", 2) == 0) continue;
    else file_names.push_back(entry->d_name);
  }
  std::sort(file_names.begin(), file_names.end());

  uint32_t curr_intra_num = 1, curr_unique_num = 1;
#ifndef NDEBUG
  for(auto a : file_names)
    std::cout << a << std::endl;
#endif
  // read the first file name
  //entry = readdir(dir);

  std::unique_ptr<MPTC::DXTImage> null_dxt(nullptr);
  std::unique_ptr<MPTC::DXTImage> curr_frame(nullptr), prev_frame(nullptr);

  curr_file = dir_name + "/" + file_names[0];
  std::unique_ptr<MPTC::DXTImage> first_frame(new MPTC::DXTImage(curr_file, true, search_area, vErrThreshold));
  curr_file.clear();
  uint32_t frame_count = 1;
  uint32_t frame_height, frame_width, unique_count = 0;
  frame_width = first_frame->Width();
  frame_height = first_frame->Height();

  //open out file for writing to it
  std::ofstream out_stream(out_file.c_str(), std::ofstream::binary);
  // Write out frame height and frame width
  // Write the ---unique interval value--- 

  uint8_t unique_int = static_cast<uint8_t>(unique_interval);
  uint8_t search_area_8bit = static_cast<uint8_t>(search_area);
  //total frame count
  uint32_t total_frame_count = file_names.size() / unique_interval;
  out_stream.write(reinterpret_cast<const char*>(&frame_height), 4);
  out_stream.write(reinterpret_cast<const char*>(&frame_width), 4);
  out_stream.write(reinterpret_cast<const char*>(&unique_int), 1);
  out_stream.write(reinterpret_cast<const char*>(&search_area), 1);
  out_stream.write(reinterpret_cast<const char*>(&total_frame_count), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_unique_count), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_palette), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_motion_indices), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_Y), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_C), 4);

  out_stream.flush();

  // out_data to be written to file
  std::vector<uint8_t> out_data[5];
  // copyt height and widht 
  // copy the size of the mask index
  // copy mask index data
  // copy size of of motion indices
  // copy unique_indices
  // copy data of motion indices
  bool set_intra = true;
  std::vector<std::thread> compress_thread;
  compress_thread.resize(thread_count);


  uint32_t max[5] = {0};

  for(uint32_t index = 0; index < file_names.size(); index += (thread_count * unique_int)) { 
    for(uint32_t thd_idx = 0; thd_idx < compress_thread.size(); thd_idx++) {
      compress_thread[thd_idx] = std::thread(SingleThreadCompressMulti,
				       dir_name,
				       std::ref(file_names),
				       index + (thd_idx * unique_int),
				       unique_int,
				       search_area,
				       vErrThreshold,
				       std::ref(out_data[thd_idx]),
				       ep_dir);

    }

/*    compress_thread[0] = std::thread(SingleThreadCompressMulti,*/
				     //dir_name,
                                     //std::ref(file_names),
                                     //index,
				     //unique_int,
				     //search_area,
				     //vErrThreshold,
				     //std::ref(out_data[0]),
				     //ep_dir);

    //compress_thread[1] = std::thread(SingleThreadCompressMulti,
				     //dir_name,
				     //std::ref(file_names),
				     //index + unique_int,
				     //unique_int,
				     //search_area,
				     //vErrThreshold,
				     //std::ref(out_data[1]),
				     //ep_dir);


    //compress_thread[2] = std::thread(SingleThreadCompressMulti,
				     //dir_name,
				     //std::ref(file_names),
				     //index + 2 * unique_int,
				     //unique_int,
				     //search_area,
				     //vErrThreshold,
				     //std::ref(out_data[2]),
				     //ep_dir);

    //compress_thread[3] = std::thread(SingleThreadCompressMulti,
				     //dir_name,
				     //std::ref(file_names),
				     //index + 3 * unique_int,
				     //unique_int,
				     //search_area,
				     //vErrThreshold,
				     //std::ref(out_data[3]),
				     //ep_dir);

    //compress_thread[4] = std::thread(SingleThreadCompressMulti,
				     //dir_name,
				     //std::ref(file_names),
				     //index + 4 * unique_int,
				     //unique_int,
				     //search_area,
				     //vErrThreshold,
				     //std::ref(out_data[4]),
				     //ep_dir);

    for(int i = 0; i < compress_thread.size(); i++){
      if(compress_thread[i].joinable())
	compress_thread[i].join();
    }

    for(int i = 0; i < compress_thread.size(); i++) {
      out_stream.write(reinterpret_cast<const char*>(out_data[i].data()), out_data[i].size());
      out_data[i].clear();
    }
      
  }

  out_stream.seekp(0, out_stream.beg);
  out_stream.seekp(14);
  out_stream.write(reinterpret_cast<const char*>(&max_unique_count), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_palette), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_motion_indices), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_Y), 4);
  out_stream.write(reinterpret_cast<const char*>(&max_compressed_ep_C), 4);

}

#ifdef UNIQUE
void DecompressMPTCStream(const std::string input_file, const std::string out_dir, uint32_t interval) {
  //open the file
  //
  std::ifstream in_stream;
  in_stream.open(input_file.c_str(), std::ifstream::binary);
  if(!in_stream.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
    exit(-1);
  }
  //read the height and width once
  uint32_t frame_height, frame_width;
  uint8_t search_area;
  in_stream.read(reinterpret_cast<char*>(&frame_height), 4);
  in_stream.read(reinterpret_cast<char*>(&frame_width), 4);
  uint32_t num_blocks = frame_height/4 * frame_width/4;

  uint8_t intra;
  in_stream.read(reinterpret_cast<char*>(&intra), 1);
  bool is_intra = static_cast<bool>(intra);
  // read 4 bytes which give total unique indices count
  uint32_t num_unique;
  in_stream.read(reinterpret_cast<char*>(&num_unique), 4);
  // read 4 bytes which give size of compressed mask bytes
  uint32_t mask_bytes;
  in_stream.read(reinterpret_cast<char*>(&mask_bytes), 4);
  // read 4 bytes which give size of compressed motion indices
  uint32_t indices_bytes;
  in_stream.read(reinterpret_cast<char*>(&indices_bytes), 4);
  // allocate memory for mask bytes and read mask bytes
  std::vector<uint8_t> compressed_index_mask(mask_bytes, 0);
  in_stream.read(reinterpret_cast<char*>(compressed_index_mask.data()), mask_bytes);
  // allocate memory for unique indices values and read unique indices bytes
  std::vector<uint32_t> unique_indices(num_unique, 0);
  in_stream.read(reinterpret_cast<char*>(unique_indices.data()), num_unique*4);
  // allocate memory for motion_index data and read compressed motion indices
  std::vector<uint8_t> compressed_motion_indices(indices_bytes, 0);
  in_stream.read(reinterpret_cast<char*>(compressed_motion_indices.data()), indices_bytes);


  //Decompress the mask bytes using arithmetic decoder
  std::vector<uint8_t> out_mask_bits(num_blocks,0);
  EntropyDecode(compressed_index_mask, out_mask_bits, true);


  //Decompress the motion indices uisng arithmetic decoder
  uint32_t num_motion_indices = (num_blocks - num_unique);
  std::vector<uint8_t> motion_indices(2 * num_motion_indices, 0);
  EntropyDecode(compressed_motion_indices, motion_indices, false);
  std::vector< std::tuple<uint8_t, uint8_t> > out_motion_indices;

  for(size_t ii = 0; ii < motion_indices.size(); ii+=2) 
    out_motion_indices.push_back(std::make_tuple(motion_indices[ii], motion_indices[ii + 1]));
    
  std::unique_ptr<DXTImage> null_dxt(nullptr);
  std::unique_ptr<DXTImage> prev_frame(nullptr), curr_frame(nullptr), first_frame(nullptr);
  // Populate the physical and logical blocks using the data
  first_frame.reset(new DXTImage(frame_width, frame_height, is_intra, unique_indices));
  //ReconstructInterpolationData(unique_indices, out_motion_indices, first_frame, null_dxt, search_area);
  first_frame->SetLogicalBlocks(); 
  uint32_t frame_number = 0;
  std::string out_frame = out_dir + "/" + std::to_string(frame_number);
#ifndef NDEBUG  
  std::unique_ptr<GreyImage> recons_image = std::move(first_frame->InterpolationImage());  
  stbi_write_png(out_frame.c_str(), recons_image->Width(), recons_image->Height(),
                 1, recons_image->Pack().data(), recons_image->Width());
#endif

  prev_frame = std::move(first_frame);

  while(!in_stream.eof()) {
    out_frame.clear();
    frame_number++;
    out_frame = out_dir + "/" + std::to_string(frame_number);
    // read 1 byte intra of inter
    in_stream.read(reinterpret_cast<char*>(&intra), 1);
    is_intra = static_cast<bool>(intra);
    // read 4 bytes which give total unique indices count
    in_stream.read(reinterpret_cast<char*>(&num_unique), 4);
    // read 4 bytes which give size of compressed mask bytes
    in_stream.read(reinterpret_cast<char*>(&mask_bytes), 4);
    // read 4 bytes which give size of compressed motion indices
    in_stream.read(reinterpret_cast<char*>(&indices_bytes), 4);
    // allocate memory for mask bytes and read mask bytes
    compressed_index_mask.resize(mask_bytes, 0);
    in_stream.read(reinterpret_cast<char*>(compressed_index_mask.data()), mask_bytes);
    // allocate memory for unique indices values and read unique indices bytes
    unique_indices.resize(num_unique, 0);
    in_stream.read(reinterpret_cast<char*>(unique_indices.data()), num_unique*4);
    // allocate memory for motion_index data and read compressed motion indices
    compressed_motion_indices.resize(indices_bytes, 0);
    in_stream.read(reinterpret_cast<char*>(compressed_motion_indices.data()), indices_bytes);


    //Decompress the mask bytes using arithmetic decoder
    out_mask_bits.resize(num_blocks,0);
    EntropyDecode(compressed_index_mask, out_mask_bits, true);


    //Decompress the motion indices uisng arithmetic decoder
    uint32_t num_motion_indices = (num_blocks - num_unique);
    motion_indices.resize(2 * num_motion_indices, 0);
    std::vector< std::tuple<uint8_t, uint8_t> > curr_out_motion_indices;

    EntropyDecode(compressed_motion_indices, motion_indices, false);
    for(size_t ii = 0; ii < motion_indices.size(); ii+=2) 
      curr_out_motion_indices.push_back(std::make_tuple(motion_indices[ii], motion_indices[ii + 1]));
   // Populate the physical and logical blocks using the data
    curr_frame.reset(new DXTImage(frame_width, frame_height, is_intra, unique_indices));
    //ReconstructInterpolationData(unique_indices, curr_out_motion_indices,
	                         //curr_frame, prev_frame, search_area);
    curr_frame->SetLogicalBlocks();
 #ifndef NDEBUG  

  std::unique_ptr<GreyImage> recon_image = std::move(curr_frame->InterpolationImage());  
  stbi_write_png(out_frame.c_str(), recon_image->Width(), recon_image->Height(),
                 1, recon_image->Pack().data(), recon_image->Width());
#endif
   
  }
  
  return;
}
#endif


} //namespace MPTC
