#include "codec.h"
#include "bit_stream.h"
#include "dxt_image.h"
#include "arithmetic_codec.h"
#include "stb_image_write.h"

#include <dirent.h>
#include <fstream>
#include <tuple>
#include <algorithm>

//#define PALETTECOMP
//#define ENDPOINTIMG
#define COMBINEPALETTE
namespace MPTC {

const uint32_t max_bytes = 1048576; // 1MB 

void MeasureEntropy(std::vector<uint8_t> &symbols) {
}
//Entropy Encode a bunch of symbols 
void EntropyEncode(std::vector<uint8_t> &symbols, std::vector<uint8_t> &out_data) {
  out_data.resize(max_bytes, 0);
  entropy::Arithmetic_Codec arith_data_encoder(max_bytes, symbols.data());
  entropy::Adaptive_Data_Model data_model(257);
  arith_data_encoder.start_encoder();
  for(auto a: symbols) {
    arith_data_encoder.encode(a, data_model);
  }
  arith_data_encoder.stop_encoder();
  uint32_t compressed_size = arith_data_encoder.get_num_bytes();
  out_data.resize(compressed_size);
  out_data.shrink_to_fit();
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
  for(auto a : dxt_frame->_motion_indices) {
    arith_data_encoder.encode(std::get<0>(a), data_model);
    arith_data_encoder.encode(std::get<1>(a), data_model);
  }
  arith_data_encoder.stop_encoder();
  indices_bytes = arith_data_encoder.get_num_bytes();

#ifndef NDEBUG
  std::cout << "compressed motion indices size: " << indices_bytes << std::endl;
#endif
  //Encode the index mask
  std::vector<uint8_t> compressed_index_mask(max_bytes, 0);
  entropy::Arithmetic_Codec arith_bit_encoder(max_bytes, compressed_index_mask.data());
  entropy::Adaptive_Bit_Model bit_model;
  arith_bit_encoder.start_encoder();
  for(auto a : dxt_frame->_index_mask) {
    arith_bit_encoder.encode(a, bit_model);
  }
  arith_bit_encoder.stop_encoder();
  mask_bytes = arith_bit_encoder.get_num_bytes();

#ifndef NDEBUG  
  std::cout << "Compressed index mask size: " << mask_bytes << std::endl;
#endif
  total_bytes = dxt_frame->_unique_palette.size() * 4 + mask_bytes + indices_bytes;
#ifndef NDEBUG
  std::cout << "Total bytes: " << total_bytes << std::endl;
#endif
  double bpp;
  bpp = static_cast<double>(total_bytes * 8) / (dxt_frame->_width * dxt_frame->_height);
#ifndef NDEBUG
  std::cout << "bpp: " << bpp << std::endl;
#endif
  //copy data to out_data
  //0.intra or inter
   out_data.resize(out_data.size() + 1, 0);
   memcpy(out_data.data() + (out_data.size() - 1), reinterpret_cast<uint8_t*>(&dxt_frame->_is_intra), 1);
  //1.size of unique indices
  uint32_t num_unique = dxt_frame->_unique_palette.size();
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data()+(out_data.size()-4), reinterpret_cast<uint8_t*>(&num_unique), 4);
  
  //2.size of compressed mask indices
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() +(out_data.size() - 4), reinterpret_cast<uint8_t*>(&mask_bytes), 4);

  //4.size of compressed motion indices
  out_data.resize(out_data.size() + 4, 0);
  memcpy(out_data.data() + (out_data.size() - 4) , reinterpret_cast<uint8_t*>(&indices_bytes), 4);

   //5.mask data
  out_data.resize(out_data.size() + mask_bytes, 0);
  memcpy(out_data.data() + (out_data.size() - mask_bytes) , compressed_index_mask.data(), mask_bytes);
  
  // copy unique_indices values
  out_data.resize(out_data.size() + num_unique * 4, 0);
  memcpy(out_data.data() + (out_data.size() - num_unique * 4), dxt_frame->_unique_palette.data(), num_unique * 4);

  
  //6.compressed motion_index data
  out_data.resize(out_data.size() + indices_bytes, 0);
  memcpy(out_data.data() + (out_data.size() - indices_bytes), compressed_motion_indices.data(), indices_bytes);

  return; 
}

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

  uint32_t curr_num = 0;
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
    if(curr_num > interval){
      set_intra = true;
      curr_num = 0;
    }
    else { 
      set_intra = false;
      curr_num++;
    }

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
#ifdef COMBINEPALETTE
    if(--curr_num > interval) {
      std::vector<uint8_t> compressed_combined_8bit_palette;
      EntropyEncode(combined_8bit_palette, compressed_combined_8bit_palette);
      std::cout << std::endl;
      std::cout << "------------------------------" << std::endl;
      std::cout << "Uncompressed combined Palette:" << combined_8bit_palette.size() << std::endl;
      std::cout << "Compressed combined Palette:" << compressed_combined_8bit_palette.size() << std::endl;
      std::cout << "-----------------------------" << std::endl;
      
      combined_8bit_palette.clear();
    }
    else {
      std::vector<uint8_t> temp_palette_8bit = curr_frame->Get8BitPalette();
      combined_8bit_palette.insert(std::end(combined_8bit_palette), std::begin(temp_palette_8bit), std::end(temp_palette_8bit));
    }
#endif 
    prev_frame = std::move(curr_frame);
  }
  out_stream.close();
}

// number of symbols expected 
// assumes memory is already allocated for out_symbols
//
void ReconstructInterpolationData(std::vector<uint32_t> &unique_indices,
                                  std::vector<std::tuple<uint8_t ,uint8_t> > &motion_indices,
                                  std::vector<uint8_t> &index_mask_in,
                                  std::unique_ptr<DXTImage> &curr_frame,
                                  std::unique_ptr<DXTImage> &prev_frame) {

  std::vector<int32_t> index_mask(index_mask_in.begin(), index_mask_in.end());
  for(size_t i = 1; i < index_mask_in.size(); i++) {
    index_mask[i] += index_mask[i-1];
  }

  std::vector<uint32_t> interpolation_data; 
  int prev_mask_idx = 0; 
  int32_t blocks_width = curr_frame->_blocks_width;
  for(int physical_idx = 0; physical_idx < curr_frame->_num_blocks; physical_idx++) {
    int curr_mask_idx = index_mask[physical_idx];
    int final_idx;
    if(prev_mask_idx == curr_mask_idx) {
      final_idx = physical_idx - curr_mask_idx;

      uint8_t x = std::get<0>(motion_indices[final_idx]);
      uint8_t y = std::get<1>(motion_indices[final_idx]);

      if( (x%4!=0) || (y%4!=0) ) { // if this conditon holds inter-pixel motion. fetch data from previous frame
        int32_t motion_x = static_cast<int32_t>(x - 64);
	int32_t motion_y = static_cast<int32_t>(y - 64);

        int32_t curr_block_x = physical_idx % blocks_width;
        int32_t curr_block_y = physical_idx / blocks_width;

	int32_t pix_x = 4 * curr_block_x + motion_x;
	int32_t pix_y = 4 * curr_block_y + motion_y;

	uint32_t interpolation = prev_frame->Get4X4InterpolationBlock(
	                                           static_cast<uint32_t>(pix_x), 
	                                           static_cast<uint32_t>(pix_y));
	interpolation_data.push_back(interpolation);
      } 
      else if((x&0b10000000)!=0 && (y&0b10000000)!=0) { //Inter block motion, fetch data from previous frame

        x = (x & 0b01111111);
        y = (y & 0b01111111);
        int32_t motion_x = static_cast<int32_t>(x - 64)/4;
        int32_t motion_y = static_cast<int32_t>(y - 64)/4;

        int32_t curr_block_x = physical_idx % blocks_width;
        int32_t curr_block_y = physical_idx / blocks_width;

        int32_t ref_block_x = curr_block_x + motion_x;
        int32_t ref_block_y = curr_block_y + motion_y;

        int ref_physical_idx = ref_block_y * blocks_width + ref_block_x;
        interpolation_data.push_back(prev_frame->_physical_blocks[ref_physical_idx].interp);
      }
      else { //Intra block motion, fetch data from own frame

        int32_t motion_x = static_cast<int32_t>(x - 64)/4;
        int32_t motion_y = static_cast<int32_t>(y - 64)/4;

        int32_t curr_block_x = physical_idx % blocks_width;
        int32_t curr_block_y = physical_idx / blocks_width;

        int32_t ref_block_x = curr_block_x + motion_x;
        int32_t ref_block_y = curr_block_y + motion_y;

        int32_t ref_physical_idx = ref_block_y * blocks_width + ref_block_x;

        assert(ref_physical_idx < static_cast<int32_t>(interpolation_data.size()));
        interpolation_data.push_back(interpolation_data[ref_physical_idx]);

      }

    } // all sorts of motion indices
    else {
      final_idx = index_mask[physical_idx] - 1;
      interpolation_data.push_back(curr_frame->_unique_palette[final_idx]);
    }
    prev_mask_idx = curr_mask_idx;
    curr_frame->_physical_blocks[physical_idx].interp = interpolation_data[physical_idx];
  }
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
  ReconstructInterpolationData(unique_indices, out_motion_indices,
			       out_mask_bits, first_frame, null_dxt);
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
    ReconstructInterpolationData(unique_indices, curr_out_motion_indices,
	                         out_mask_bits, curr_frame, prev_frame);
    curr_frame->SetLogicalBlocks();
 #ifndef NDEBUG  

  std::unique_ptr<GreyImage> recon_image = std::move(curr_frame->InterpolationImage());  
  stbi_write_png(out_frame.c_str(), recon_image->Width(), recon_image->Height(),
                 1, recon_image->Pack().data(), recon_image->Width());
#endif
   
  }
  
  return;
}


} //namespace MPTC
