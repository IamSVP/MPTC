#include "dxt_image.h"


#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <random>
#include <tuple>


#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_DXT_IMPLEMENTATION
#include "stb_dxt.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//#define NDEBUG

int32_t vErrThreshold;


template <typename T>
static inline T AbsDiff(T a, T b) {
  return a > b ? a - b : b - a;
}


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

static uint16_t Pack565(const uint8_t in[3]) {
  uint16_t result = 0;
  result |= static_cast<uint16_t>(in[0] & 0xF8) << 8;
  result |= static_cast<uint16_t>(in[1] & 0xFC) << 3;
  result |= static_cast<uint16_t>(in[2] & 0xF8) >> 3;
  return result;
}

// Snap to closest 5-bit value
static uint8_t ToFiveBits(const uint8_t x) {
  uint8_t base = x & 0xF8;
  uint8_t high = base == 255 ? base : base + 0x4;
  uint8_t low = base == 0 ? base : base - 0x4;

  base = base | (base >> 5);
  high = high | (high >> 5);
  low = low | (low >> 5);

  uint8_t diff_base = AbsDiff(x, base);
  uint8_t diff_high = AbsDiff(x, high);
  uint8_t diff_low = AbsDiff(x, low);

  if (diff_base <= diff_high) {
    if (diff_base < diff_low) {
      return base;
    } else {
      return low;
    }
  } 

  assert(diff_high < diff_low);
  return high;
}

// Snap to closest 6-bit value
static uint8_t ToSixBits(const uint8_t x) {
  uint8_t base = x & 0xFC;
  uint8_t high = base == 255 ? base : base + 0x2;
  uint8_t low = base == 0 ? base : base - 0x2;

  base = base | (base >> 6);
  high = high | (high >> 6);
  low = low | (low >> 6);

  uint8_t diff_base = AbsDiff(x, base);
  uint8_t diff_high = AbsDiff(x, high);
  uint8_t diff_low = AbsDiff(x, low);

  if (diff_base <= diff_high) {
    if (diff_base < diff_low) {
      return base;
    } else {
      return low;
    }
  }

  assert(diff_high < diff_low);
  return high;
}

static uint64_t CompressRGB(const uint8_t *img, int width) {
  unsigned char block[64];
  memset(block, 0, sizeof(block));

  for (int j = 0; j < 4; ++j) {
    for (int i = 0; i < 4; ++i) {
      int src_idx = (j * width + i) * 3;
      int dst_idx = (j * 4 + i) * 4;

      unsigned char *block_pixel = block + dst_idx;
      const unsigned char *img_pixel = img + src_idx;

      block_pixel[0] = img_pixel[0];
      block_pixel[1] = img_pixel[1];
      block_pixel[2] = img_pixel[2];
      block_pixel[3] = 0xFF;
    }
  }

  PhysicalDXTBlock result;
  //squish::Compress(block, reinterpret_cast<unsigned char *>(&result.dxt_block), squish::kDxt1);

  stb_compress_dxt_block(reinterpret_cast<unsigned char *>(&result.dxt_block),
    block, 0, STB_DXT_HIGHQUAL);

  return result.dxt_block;
}




static bool PhysicalBlockNeedsSwap(const LogicalDXTBlock &b) {
  uint16_t p0 = Pack565(b.ep1);
  uint16_t p1 = Pack565(b.ep2);

  bool swap = p0 > p1 && b.palette[3][3] == 0;
  swap = swap || (p0 <= p1 && b.palette[3][3] == 255);
  return swap;
}

namespace MPTC {

PhysicalDXTBlock LogicalToPhysical(const LogicalDXTBlock &b) {
  PhysicalDXTBlock result;
  result.ep1 = Pack565(b.ep1);
  result.ep2 = Pack565(b.ep2);

  bool swap = PhysicalBlockNeedsSwap(b);
  if (swap) {
    std::swap(result.ep1, result.ep2);
  }

  result.interp = 0;
  uint8_t *bytes = reinterpret_cast<uint8_t *>(&result.interp);
  for (int k = 0; k < 4; ++k) {
    assert(b.indices[0 + 4 * k] < 4);
    bytes[k] |= b.indices[0 + 4 * k];

    assert(b.indices[1 + 4 * k] < 4);
    bytes[k] |= b.indices[1 + 4 * k] << 2;

    assert(b.indices[2 + 4 * k] < 4);
    bytes[k] |= b.indices[2 + 4 * k] << 4;

    assert(b.indices[3 + 4 * k] < 4);
    bytes[k] |= b.indices[3 + 4 * k] << 6;
  }

  if (swap) {
    result.interp ^= 0x55555555;
  }

  return result;
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
  PhysicalToLogicalBlocks(const std::vector<PhysicalDXTBlock> &blocks) {
  std::vector<LogicalDXTBlock> out;
  out.reserve(blocks.size());

  for (const PhysicalDXTBlock &b : blocks) {
    LogicalDXTBlock lb = PhysicalToLogical(b);
    assert(LogicalToPhysical(lb).dxt_block == b.dxt_block);
    out.push_back(lb);
  }

  return std::move(out);
}


struct CompressedBlock {

  std::vector<uint8_t> _uncompressed;
  LogicalDXTBlock _logical;

  bool operator==(const CompressedBlock other) const {
    bool b1 = true;
    for(size_t i = 0; i < _uncompressed.size(); i++) {
      b1 = b1 && (this->_uncompressed[i] == other._uncompressed[i]);
    }
    bool b2 = this->_logical == other._logical;
    return b1 && b2;
  }

  size_t Error() const {
    size_t err = 0;
    for (size_t idx = 0; idx < 16; idx++) {
      uint8_t i = _logical.indices[idx];

      uint8_t pixel[3];
      pixel[0] = _logical.palette[i][0];
      pixel[1] = _logical.palette[i][1];
      pixel[2] = _logical.palette[i][2];

      size_t diff_r = AbsDiff(_uncompressed[idx * 3 + 0], pixel[0]);
      size_t diff_g = AbsDiff(_uncompressed[idx * 3 + 1], pixel[1]);
      size_t diff_b = AbsDiff(_uncompressed[idx * 3 + 2], pixel[2]);

      err += diff_r * diff_r + diff_g * diff_g + diff_b * diff_b;
    }

    return err / (16 * 3);
  }

  size_t CompareAgainst(const LogicalDXTBlock &other) const {
    CompressedBlock dup = *this;
    dup._logical = other;
    dup.RecalculateEndpoints();
    return dup.Error();
  }

  void AssignIndices(const uint32_t idx) {
    PhysicalDXTBlock pblk = LogicalToPhysical(_logical);
    pblk.interp = idx;
    _logical = PhysicalToLogical(pblk);
  }
  void RecalculateEndpoints() {
    // Now that we know the index of each pixel, we can assign the endpoints based
    // on a least squares fit of the clusters. For more information, take a look
    // at this article by NVidia: http://developer.download.nvidia.com/compute/
    // cuda/1.1-Beta/x86_website/projects/dxtc/doc/cuda_dxtc.pdf
    float asq = 0.0, bsq = 0.0, ab = 0.0;
    float ax[3] = { 0.0f, 0.0f, 0.0f };
    float bx[3] = { 0.0f, 0.0f, 0.0f };
    for (size_t i = 0; i < 16; i++) {
      const uint8_t *orig_pixel = _uncompressed.data() + i * 3;

      static const float idx_to_order[4] = { 0.f, 3.f, 1.f, 2.f };
      const float order = idx_to_order[_logical.indices[i]];
      const float fbi = 3.0f - order;
      const float fb = 3.0f;
      const float fi = order;

      const float a = fbi / fb;
      const float b = fi / fb;

      asq += a * a;
      bsq += b * b;
      ab += a * b;

      for (size_t j = 0; j < 3; ++j) {
        ax[j] += static_cast<float>(orig_pixel[j]) * a;
        bx[j] += static_cast<float>(orig_pixel[j]) * b;
      }
    }

    float f = 1.0f / (asq * bsq - ab * ab);

    float p1[3], p2[3];
    for (int i = 0; i < 3; ++i) {
      p1[i] = f * (ax[i] * bsq - bx[i] * ab);
      p2[i] = f * (bx[i] * asq - ax[i] * ab);
    }

    // Quantize the endpoints...
    for (int i = 0; i < 3; ++i) {
      _logical.ep1[i] = std::max(0, std::min(255, static_cast<int32_t>(p1[i] + 0.5f)));
      _logical.ep2[i] = std::max(0, std::min(255, static_cast<int32_t>(p2[i] + 0.5f)));
    }

    _logical.ep1[0] = ToFiveBits(_logical.ep1[0]);
    _logical.ep2[0] = ToFiveBits(_logical.ep2[0]);

    _logical.ep1[1] = ToSixBits(_logical.ep1[1]);
    _logical.ep2[1] = ToSixBits(_logical.ep2[1]);

    _logical.ep1[2] = ToFiveBits(_logical.ep1[2]);
    _logical.ep2[2] = ToFiveBits(_logical.ep2[2]);

    memcpy(_logical.palette[0], _logical.ep1, 4);
    memcpy(_logical.palette[1], _logical.ep2, 4);

    LerpChannels(_logical.ep1, _logical.ep2, _logical.palette[2], 1, 3);
    LerpChannels(_logical.ep1, _logical.ep2, _logical.palette[3], 2, 3);

    //PhysicalDXTBlock pblk = LogicalToPhysical(_logical);
    //_logical = PhysicalToLogical(pblk);
  }

};


//Declare static variables....
std::vector<std::tuple<int,int> > DXTImage::_search_pattern;
int32_t DXTImage::_err_threshold;
std::vector<std::tuple<uint32_t, uint32_t> > DXTImage::_global_palette;
std::set<uint32_t> DXTImage::_global_palette_dict;


double DXTImage::PSNR() const {

  double orig_mse = 0.0;
  for(int j = 0; j < _height; j++) {
    for(int i = 0; i < _width; i++) {
      uint8_t pixel[4];
      GetColorAt(i, j, pixel);

      const size_t src_idx = (j * _width + i) * 3;
      for(int c = 0; c < 3; c++) {
	double orig = static_cast<double>(_src_img[src_idx + c]);
	double cmp = static_cast<double>(pixel[c]);
	double diff = orig - cmp;
	orig_mse += diff * diff;
      }
    }
  }

  orig_mse /= static_cast<double>(_width * _height);
  return 10.0 * log10( (3.0 * 255.0 * 255.0) / orig_mse);
}

DXTImage::DXTImage(const std::string orig_file, bool is_intra, int search_area, 
                   int32_t err_threshold)
          :_is_intra(is_intra)
	  ,_search_area(search_area)
	  
{
    // load the file 
  // Otherwise, load the file
  _err_threshold = err_threshold;
  vErrThreshold = err_threshold;

  stbi_uc *data = stbi_load(orig_file.c_str(), &_width, &_height, NULL, 3);
  if (!data) {
    assert(!"Error loading image");
    std::cout << "Error loading image" << orig_file << std::endl;
    return;
  }

  _blocks_width = ( _width + 3) >> 2;
  _blocks_height = ( _height + 3) >> 2;
  const int num_blocks = _blocks_width * _blocks_height;
  _num_blocks = num_blocks;

  _found_at.resize(_blocks_height * _blocks_width);
  _found.resize(_blocks_height * _blocks_width);
  std::fill(_found.begin(), _found.end(), 0);
  _index_mask.resize(_blocks_height * _blocks_width);
 
    //Can do way way better W*H bytes extra !! 
  uint32_t src_img_sz = _width * _height * 3;
  _src_img.resize(src_img_sz);
  memcpy(_src_img.data(), data, src_img_sz);


  if (_physical_blocks.size() == 0) {
    // Compress the DXT data
    _physical_blocks.resize(num_blocks);
    for (int physical_idx = 0; physical_idx < num_blocks; ++physical_idx) {
      uint16_t i, j;
      i = static_cast<uint16_t>(physical_idx % _blocks_width);
      j = static_cast<uint16_t>(physical_idx / _blocks_width);

      int block_idx = j * _blocks_width + i;
      const unsigned char *offset_data = _src_img.data() + (j * 4 * _width + i * 4) * 3;
      _physical_blocks[block_idx].dxt_block = CompressRGB(offset_data, _width);
    }
  }

  _logical_blocks = std::move(PhysicalToLogicalBlocks(_physical_blocks));
  MakeDict();

}
DXTImage::DXTImage(int width, int height, bool is_intra, const std::vector<uint32_t> unique_indices = std::vector<uint32_t>() ) {
  _width = width;
  _height = height;

  _blocks_width = ( _width + 3) >> 2;
  _blocks_height = ( _height + 3) >> 2;
  const int num_blocks = _blocks_width * _blocks_height;
  _num_blocks = num_blocks;
  _is_intra = is_intra;
  _physical_blocks.resize(num_blocks);
  for(auto a : unique_indices) 
    _unique_palette.push_back(a);

}

void DXTImage::SetLogicalBlocks() {

  _logical_blocks = std::move(PhysicalToLogicalBlocks(_physical_blocks));
}
void DXTImage::SetInterpolationData(std::vector<uint32_t> &interpolation_data) {
  size_t idx = 0;
  for(auto e : interpolation_data) {
    _physical_blocks[idx].interp = e;
    idx++;
  }
}
std::unique_ptr<RGBImage> DXTImage::DecompressedImage() const {
  std::vector<uint8_t> result;

  for(int j = 0; j < _height; j++) {
    for(int i = 0; i < _width; i++) {
      uint8_t pixel[4];
      GetColorAt(i, j, pixel);

      for(int c = 0; c < 3; c++){
	result.push_back(pixel[c]);
      }

    }
  }
  std::unique_ptr<RGBImage> img(new RGBImage(_width, _height, std::move(result)));
  return std::move(img);
}

std::unique_ptr<GreyImage> DXTImage::FoundMaskImage() const {
  std::vector<uint8_t> result;
  result.reserve(_blocks_width * _blocks_height);

  for(auto val : _found) {
    if(val) result.push_back(255);
    else result.push_back(0);
  }
  std::unique_ptr<GreyImage> img(new GreyImage(_blocks_width, _blocks_height, std::move(result)));
  return std::move(img);
}

DXTImage::DXTImage() {
//Empty constructor
}
std::unique_ptr<RGB565Image> DXTImage::EndpointOneValues() const {
  std::vector<uint8_t> result;
  const size_t img_sz = 2 * BlocksWide() * BlocksHigh();
  result.reserve(img_sz);

  for (const auto &pb : _physical_blocks) {
    uint32_t x = pb.ep1;
    result.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    result.push_back(static_cast<uint8_t>(x & 0xFF));
  }

  assert(result.size() == img_sz);

  std::unique_ptr<RGB565Image> img
    (new RGB565Image(BlocksWide(), BlocksHigh(), std::move(result)));
  return std::move(img);
}

std::unique_ptr<RGB565Image> DXTImage::EndpointTwoValues() const {
  std::vector<uint8_t> result;
  const size_t img_sz = 2 * BlocksWide() * BlocksHigh();
  result.reserve(img_sz);

  for (const auto &pb : _physical_blocks) {
    uint32_t x = pb.ep2;
    result.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
    result.push_back(static_cast<uint8_t>(x & 0xFF));
  }

  assert(result.size() == img_sz);

  std::unique_ptr<RGB565Image> img
    (new RGB565Image(BlocksWide(), BlocksHigh(), std::move(result)));
  return std::move(img);
}


std::unique_ptr<RGBImage> DXTImage::EndpointOneImage() const {
  std::vector<uint8_t> result;
  result.reserve(3 * _blocks_width * _blocks_height);

  for(auto &lb : _logical_blocks) {
    result.push_back(lb.ep1[0]);
    result.push_back(lb.ep1[1]);
    result.push_back(lb.ep1[2]);
  }
  

  std::unique_ptr<RGBImage> img 
    (new RGBImage(_blocks_width, _blocks_height, std::move(result)));
  return std::move(img);

}

std::unique_ptr<GreyImage> DXTImage::InterpolationImage() const {
  std::vector<uint8_t> result = std::move(InterpolationValues());

  for(uint32_t i = 0; i < result.size(); i++) {
    switch(result[i]) {
      case 0:
	result[i] = 0;
	break;
      case 1:
	result[i] = 85;
	break;
      case 2:
	result[i] = 175;
	break;
      case 3:
	result[i] = 225;
	break;
    }
  }
  
  std::unique_ptr<GreyImage> img(new GreyImage(_width, _height, std::move(result)));
  return std::move(img);    
}

std::unique_ptr<RGBImage> DXTImage::EndpointTwoImage() const {
  std::vector<uint8_t> result;
  result.reserve(3 * _blocks_width * _blocks_height);
  
  for(auto &lb : _logical_blocks) {
    result.push_back(lb.ep2[0]);
    result.push_back(lb.ep2[1]);
    result.push_back(lb.ep2[2]);
  }

  std::unique_ptr<RGBImage> img
    (new RGBImage(_blocks_width, _blocks_height, std::move(result)));

  return std::move(img);
}

std::vector<uint8_t> DXTImage::InterpolationValues() const {
  std::vector<uint8_t> values;
  values.reserve(_width * _height);

  for (int y = 0; y < _height; ++y) {
    for (int x = 0; x < _width; ++x) {
      values.push_back(InterpolationValueAt(x, y));
    }
  }

  assert(values.size() == static_cast<size_t>(_width * _height));
  return std::move(values);
}

uint8_t DXTImage::InterpolationValueAt(int x, int y) const {
  int block_idx = BlockAt(x, y);
  int pixel_idx = (y % 4) * 4 + (x % 4);
  return _logical_blocks[block_idx].indices[pixel_idx];
}

void DXTImage::GetColorAt(int x, int y, uint8_t out[4]) const {
  const LogicalDXTBlock &b = _logical_blocks[BlockAt(x, y)];
  uint8_t i = InterpolationValueAt(x, y);
  out[0] = b.palette[i][0];
  out[1] = b.palette[i][1];
  out[2] = b.palette[i][2];
  out[3] = b.palette[i][3];
}

uint32_t DXTImage::Get4X4InterpolationBlock(uint32_t x, uint32_t y) {

  assert( (x+3) < static_cast<uint32_t>(_width) && x>=0);
  assert( (y+3) < static_cast<uint32_t>(_height) && y>=0);
  LogicalDXTBlock lb;
  int idx = 0;
  for(uint32_t j = 0; j < 4; j++) {
    for(uint32_t i = 0; i < 4; i++) {
      lb.indices[idx] = InterpolationValueAt(x+i, y+j);
      idx++;
    }
  }
  PhysicalDXTBlock pb;
  pb = LogicalToPhysical(lb);
 return pb.interp;
}

std::vector<uint8_t> DXTImage::Get4X4ColorsBlock(uint32_t i, uint32_t j) {
  std::vector<uint8_t> result(48, 0);

  const unsigned char *offset_data = _src_img.data() + (j * _width + i ) * 3;
  for (int row = 0; row < 4; ++row) {
     uint8_t block_row[12];
     for (int p = 0; p < 4; ++p) {
       block_row[3 * p + 0] = offset_data[(row * _width + p) * 3 + 0];
       block_row[3 * p + 1] = offset_data[(row * _width + p) * 3 + 1];
       block_row[3 * p + 2] = offset_data[(row * _width + p) * 3 + 2];
     }
     memcpy(result.data() + 12 * row, block_row, sizeof(block_row));
  }
  return std::move(result);
}

int DXTImage::IntraSearch(int32_t block_idx, int32_t &min_err_x, int32_t &min_err_y, 
                          uint32_t &index, CompressedBlock &blk, bool &re_assigned, uint8_t search_space) {

  int32_t offset;

  int32_t offset_x = _search_area;
  int32_t offset_y = 2 * _search_area - 1;

  offset = 4 * search_space;
  int32_t block_x  = block_idx % _blocks_width;
  int32_t block_y = block_idx / _blocks_width;

  blk._logical = _logical_blocks[block_idx];
  assert(block_x < _blocks_width && block_y < _blocks_height);
  blk._uncompressed = std::move(Get4X4ColorsBlock(4*block_x, 4*block_y));

  const int orig_err = static_cast<int>(blk.Error());
  int min_err = std::numeric_limits<int>::max();
  bool reset_endpoints = false;

  for(int32_t j = block_y; j >= block_y - 2 * _search_area + 1; j--) {
    for(int32_t i = block_x + (_search_area - 1); i >= block_x - _search_area; i--) {
      if( (i >= _blocks_width || j >= _blocks_height || j < 0 || i < 0) || 
	                (j == block_y && i >= block_x)) continue;

      reset_endpoints = false;
      uint32_t curr_blk_idx = j * _blocks_width + i;
      uint32_t indices = _physical_blocks[curr_blk_idx].interp;
      CompressedBlock blk2 = blk;
      blk2.AssignIndices(indices);
      if(!(blk == blk2)) {
        blk2.RecalculateEndpoints();
        reset_endpoints = true; 
      // !HACK! Check if it flips the indices... There has to be a
      // better way to deal with this... In principle we can just leave
      // them flipped and then reflip them back to the proper value
      // in the decompressor...
        PhysicalDXTBlock maybe_blk = LogicalToPhysical(blk2._logical);
        bool ok = maybe_blk.interp == indices;
        ok = ok && blk2._logical.palette[3][3] == 0xFF;
        if(!ok) {
          continue;
        }
      } 
      
      int err = static_cast<int>(blk2.Error());
      int err_diff = err - orig_err;
      if(err_diff < min_err) {
        min_err = err_diff;
        min_err_x = (i - block_x) + offset_x; min_err_y = (j - block_y) + offset_y;
	index = indices;
	re_assigned = reset_endpoints;
        if(err_diff <= 0) {
	  min_err = 0;
	  break;
        }
      }
    
    }
  }
  return min_err;
}

int DXTImage::InterBlockSearch(std::unique_ptr<DXTImage> &reference, int32_t block_idx,
                               int32_t &min_err_x, int32_t &min_err_y, int ref_number, 
		               uint32_t &index, CompressedBlock &blk, bool &re_assigned) {

  int32_t block_x = block_idx % _blocks_width;
  int32_t block_y = block_idx / _blocks_width;
  int32_t search_space = _search_area;
  int32_t offset = _search_area;
  assert(block_x < _blocks_width && block_y < _blocks_height);

  blk._logical = _logical_blocks[block_idx];
  blk._uncompressed = std::move(Get4X4ColorsBlock(4*block_x, 4*block_y));

  const int orig_err = static_cast<int>(blk.Error());
  int min_err = std::numeric_limits<int>::max();
  bool reset_endpoints = false;
  for(int32_t j = block_y - _search_area; j < block_y + _search_area; j++) {
    for(int32_t i = block_x - _search_area; i < block_x + _search_area; i++) {

      reset_endpoints = false;
      if(i >= _blocks_width || j >= _blocks_height || j < 0 || i < 0) continue;


      int32_t curr_blk_idx = j * _blocks_width + i;
      uint32_t indices = reference->_physical_blocks[curr_blk_idx].interp;
      CompressedBlock blk2 = blk;
      blk2.AssignIndices(indices);
      if(!(blk == blk2)) {
        blk2.RecalculateEndpoints();
	reset_endpoints = true;
    
      // !HACK! Check if it flips the indices... There has to be a
      // better way to deal with this... In principle we can just leave
      // them flipped and then reflip them back to the proper value
      // in the decompressor...
        PhysicalDXTBlock maybe_blk = LogicalToPhysical(blk2._logical);
        bool ok = maybe_blk.interp == indices;
        ok = ok && blk2._logical.palette[3][3] == 0xFF;
        if(!ok) {
          continue;
        }
      }
      int err = static_cast<int>(blk2.Error());
      int err_diff = err - orig_err;
      if(err_diff < min_err) {
        min_err = err_diff;
	min_err_x = (i - block_x) + offset; min_err_y = (j - block_y) + offset;
	index = indices;
	re_assigned = reset_endpoints;
        if(err_diff <= 0) {
	  min_err = 0;
	  break;
        }
      }
    
    }
  }

  return min_err;
}

int DXTImage::InterPixelSearch(std::unique_ptr<DXTImage> &reference, int32_t block_idx,
		               int32_t &min_err_x, int32_t &min_err_y, int ref_number, 
		               uint32_t &index, CompressedBlock &blk, bool &re_assigned) {

  int32_t block_x = block_idx % _blocks_width;
  int32_t block_y = block_idx / _blocks_width;
  assert(block_x < _blocks_width && block_y < _blocks_height);

  blk._logical = _logical_blocks[block_idx];
  blk._uncompressed = std::move(Get4X4ColorsBlock(4*block_x, 4*block_y));

  const int orig_err = static_cast<int>(blk.Error());
  int min_err = std::numeric_limits<int>::max();
  bool reset_endpoints = false; 
  for(auto val : _search_pattern) {
    reset_endpoints = false;
    int i = std::get<0>(val);
    int j = std::get<1>(val);
    int32_t pix_x = 4 * block_x;
    int32_t pix_y = 4 * block_y;
    if( pix_x + i >= 0 && pix_x+i <= _width-4 && pix_y+j >= 0 && pix_y+j <= _height-4 ) {
      uint32_t indices = reference->Get4X4InterpolationBlock(pix_x + i, pix_y + j);
      CompressedBlock blk2 = blk;
      blk2.AssignIndices(indices);
      if(!(blk2 == blk)) {
        blk2.RecalculateEndpoints();
	reset_endpoints = true;
      // !HACK! Check if it flips the indices... There has to be a
      // better way to deal with this... In principle we can just leave
      // them flipped and then reflip them back to the proper value
      // in the decompressor...
        PhysicalDXTBlock maybe_blk = LogicalToPhysical(blk2._logical);
        bool ok = maybe_blk.interp == indices;
        ok = ok && blk2._logical.palette[3][3] == 0xFF;
        if(!ok) {
          continue;
        }
      }
      int err = static_cast<int>(blk2.Error());
      int err_diff = err - orig_err;
      if(err_diff < min_err) {
        min_err = err_diff;
	min_err_x = i + 64; min_err_y = j + 64;
	index = indices;
	re_assigned = reset_endpoints;
        if(err_diff <= 0) {
	  min_err = 0;
	  break;
        }
      }

    }

  }

  return min_err;
}


void DXTImage::ReencodeAndAnalyze(std::unique_ptr<DXTImage> &reference, int ref_number, std::vector<std::tuple<uint8_t, uint8_t> > &motion_indices, uint8_t search_space) {
  for(int32_t physical_idx = 0; physical_idx < static_cast<int32_t>(_physical_blocks.size()); physical_idx++) {
    int err_inter_pixel, err_inter_block, err_intra;
    int32_t min_err_x, min_err_y;
    uint32_t min_interp;
    CompressedBlock blk;
    bool re_assigned;

   re_assigned = false;
    err_intra = 
      IntraSearch(physical_idx, min_err_x, min_err_y, min_interp, blk, re_assigned, search_space);
    if(err_intra <= vErrThreshold) {
      _found[physical_idx] = true;
      std::tuple<uint8_t, uint8_t> motion_idx(static_cast<uint8_t>(min_err_x), static_cast<uint8_t>(min_err_y));
      _intra_motion_indices.push_back(motion_idx);
      _motion_indices.push_back(motion_idx);
      motion_indices.push_back(motion_idx);
      _index_mask[physical_idx] = 0;
      //Re-assign Indices
      blk.AssignIndices(min_interp);
      if(re_assigned)
        blk.RecalculateEndpoints();
      _logical_blocks[physical_idx] = blk._logical;
      _physical_blocks[physical_idx] = LogicalToPhysical(blk._logical);
      continue;
    }
    _index_mask[physical_idx] = 1;
    _unique_palette.push_back(_physical_blocks[physical_idx].interp);
    _global_palette.push_back(std::make_tuple(_physical_blocks[physical_idx].interp, physical_idx));
    _global_palette_dict.insert(_physical_blocks[physical_idx].interp);
  }
}

void DXTImage::Reencode(std::unique_ptr<DXTImage> &reference, int ref_number) {
  // add check for reference NULL!!!
  // 0 - unique indices
  // 1 - intra
  // 2 - inter = found in reference frame
  //
#ifndef NDEBUG
    std::cout << "Frame intepolation data: " << std::endl;
#endif
  std::tuple<uint8_t, uint8_t> unique_motion_idx(255, 255);
  for(int32_t physical_idx = 0; physical_idx < static_cast<int32_t>(_physical_blocks.size()); physical_idx++) {
    int err_inter_pixel, err_inter_block, err_intra;
    int32_t min_err_x, min_err_y;
    uint32_t min_interp;
    CompressedBlock blk;
    bool re_assigned;

    if(!_is_intra) {
      re_assigned = false;
      err_inter_block = 
	InterBlockSearch(reference, physical_idx, min_err_x, min_err_y, ref_number, min_interp,
	                 blk, re_assigned);
      if(err_inter_block<= vErrThreshold) {
        _found[physical_idx] = true;
	std::tuple<uint8_t, uint8_t> motion_idx(static_cast<uint8_t>(min_err_x), static_cast<uint8_t>(min_err_y));
	std::get<0>(motion_idx) |= 0b10000000;
	std::get<1>(motion_idx) |= 0b10000000;
	assert(std::get<0>(motion_idx) < 256 && std::get<1>(motion_idx) < 256);
	_inter_block_motion_indices.push_back(motion_idx);
	_motion_indices.push_back(motion_idx);
	_index_mask[physical_idx] = 0;
	//!!Make sure the error is less than the threshold!!//
        blk.AssignIndices(min_interp);
	if(re_assigned)
	  blk.RecalculateEndpoints();

	_logical_blocks[physical_idx] = blk._logical;
	_physical_blocks[physical_idx] = LogicalToPhysical(blk._logical);

	continue;
      }
    }


   re_assigned = false;
    err_intra = 
      IntraSearch(physical_idx, min_err_x, min_err_y, min_interp, blk, re_assigned, 16);
    if(err_intra <= vErrThreshold) {
      _found[physical_idx] = true;
      std::tuple<uint8_t, uint8_t> motion_idx(static_cast<uint8_t>(min_err_x), static_cast<uint8_t>(min_err_y));
      _intra_motion_indices.push_back(motion_idx);
      _motion_indices.push_back(motion_idx);
      _index_mask[physical_idx] = 0;
      //Re-assign Indices
      blk.AssignIndices(min_interp);
      if(re_assigned)
        blk.RecalculateEndpoints();
      _logical_blocks[physical_idx] = blk._logical;
      _physical_blocks[physical_idx] = LogicalToPhysical(blk._logical);
      continue;
    }

/*    if(!_is_intra) {*/
      //re_assigned = false;
      //err_inter_pixel = 
	//InterPixelSearch(reference, physical_idx, min_err_x, min_err_y, ref_number, min_interp,
			 //blk, re_assigned);
      //if(err_inter_pixel <= vErrThreshold) {
	//_found[physical_idx] = true;
	//std::tuple<uint8_t, uint8_t> motion_idx(static_cast<uint8_t>(min_err_x), static_cast<uint8_t>(min_err_y));
        //_inter_pixel_motion_indices.push_back(motion_idx);
	//_motion_indices.push_back(motion_idx);
	//_index_mask[physical_idx] = 0;

	////Re-assign Indices
        //blk.AssignIndices(min_interp);
	//if(re_assigned)
	  //blk.RecalculateEndpoints();

	//_logical_blocks[physical_idx] = blk._logical;
	//_physical_blocks[physical_idx] = LogicalToPhysical(blk._logical);
	//continue;
      //}
    //}
 
    _motion_indices.push_back(unique_motion_idx);
    _unique_palette.push_back(_physical_blocks[physical_idx].interp);
    
  }
}

uint32_t DXTImage::Error(std::unique_ptr<DXTImage> &other) {
  size_t  error = 0;
  for(size_t idx = 0; idx < _src_img.size(); idx++) 
    error += std::abs(_src_img[idx] - other->_src_img[idx]);

  return error/(_width * _height *3);
  
}

} // namespace MPTC

