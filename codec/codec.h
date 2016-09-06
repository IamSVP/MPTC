#ifndef __CODEC_CODEC_H__
#define __CODEC_CODEC_H__

#include <string>
#include <vector>
#include <memory>

#include "dxt_image.h"

namespace MPTC {

void CompressPNGStream(const std::string dir_name, const std::string out_file, uint32_t search_area, int32_t vErrThreshold, uint32_t interval, std::string ep_dir = std::string());

void EntropyEncode(std::unique_ptr<MPTC::DXTImage> &dxt_frame, std::vector<uint8_t> &out_data);

void DecompressMultiUnique(const std::string input_file, const std::string out_dir);

void CompressMultiUnique(const std::string dir_name, const std::string out_file, 
                         uint32_t search_area, int32_t vErrThreshold, uint32_t intra_interval, 
			 uint32_t unique_interval, std::string ep_dir);

void EntroypDecode(const std::vector<uint8_t> &compressed_data, std::vector<uint8_t> &out_symbols, bool is_bit_model);

void DecompressMPTCStream(const std::string input_file, const std::string out_dir, uint32_t interval);

} //namespace MPTC

#endif // __CODEC_CODEC_H__
