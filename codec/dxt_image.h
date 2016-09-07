#ifndef __DXT_IMAGE_H__
#define __DXT_IMAGE_H__


#include <array>
#include <vector>
#include <set>
#include <memory>
#include <string>
#include <cstring>

#include "image.h"

//Forward Declaration

union PhysicalDXTBlock {
  struct {
    uint16_t ep1;
    uint16_t ep2;
    uint32_t interp;
  };
  uint64_t dxt_block;
};

struct LogicalDXTBlock {
  uint8_t ep1[4];
  uint8_t ep2[4];
  uint8_t palette[4][4];
  uint8_t indices[16];
  LogicalDXTBlock &operator=(const LogicalDXTBlock &other) {
    memcpy(this, &other, sizeof(*this));
    return *this;
  }

  bool operator==(const LogicalDXTBlock &other) const {
    return memcmp(this, &other, sizeof(*this)) == 0;
  }

};


namespace MPTC {
struct CompressedBlock;
PhysicalDXTBlock LogicalToPhysical(const LogicalDXTBlock &b);
LogicalDXTBlock PhysicalToLogical(const PhysicalDXTBlock &b);
  class DXTImage {

   public:
     DXTImage(const std::string orig_file, bool is_first, int search_area, 
	      int32_t err_threshold);
     DXTImage();
     DXTImage(int width, int height, uint8_t *rgb);
     DXTImage(int widht, int height, bool is_intra, const std::vector<uint32_t> unique_indices);
     
     int Width() const { return _width; }
     int Height() const { return _height; }

     int BlocksWide() const { return _blocks_width; }
     int BlocksHigh() const { return _blocks_height;}

     std::unique_ptr<RGBImage> EndpointOneImage() const;
     std::unique_ptr<RGBImage> EndpointTwoImage() const;
     std::unique_ptr<RGBImage> DecompressedImage() const;
     std::unique_ptr<GreyImage> InterpolationImage() const;
     
     static std::vector<uint8_t> TwoBitValuesToImage(const std::vector<uint8_t> &v);
     std::vector<uint8_t> InterpolationValues() const;
     void GetColorAt(int x, int y, uint8_t out[4]) const; 
     uint8_t InterpolationValueAt(int x, int y) const;

     inline uint32_t ErrorReencode();
     int IntraSearch(int32_t block_idx, int32_t &min_err_x, int32_t &min_err_y, 
	             uint32_t &index, CompressedBlock &blk, bool &re_assigned, uint8_t search_space);

     int InterBlockSearch(std::unique_ptr<DXTImage> &reference, int32_t block_idx, 
	                  int32_t &min_err_x, int32_t &min_err_y, int ref_number, 
			  uint32_t &index, CompressedBlock &blk, bool &re_assigned);

     int InterPixelSearch(std::unique_ptr<DXTImage> &reference, int32_t block_idx, 
	                  int32_t &min_err_x, int32_t &min_err_y, int ref_number, 
			  uint32_t &index, CompressedBlock &blk, bool &re_assigned);

     void SearchForInterpolation(std::unique_ptr<DXTImage> &refrence, int ref_number);
     std::unique_ptr<GreyImage> FoundMaskImage() const;
     void SetLogicalBlocks();
     uint32_t Get4X4InterpolationBlock(uint32_t x, uint32_t y);
     std::vector<uint8_t> Get4X4ColorsBlock(uint32_t x, uint32_t y); 

     void ReencodeAndAnalyze(std::unique_ptr<DXTImage> &reference, int ref_number, std::vector<std::tuple<uint8_t, uint8_t> > &motion_indices, uint8_t search_space);
     void Reencode(std::unique_ptr<DXTImage> &reference, int ref_number);
     double PSNR() const;     
     void SetInterpolationData(std::vector<uint32_t> &interpolation_data);
    inline uint32_t PixelAt(int x, int y) const {
       return  y * _width + x;
     }

     inline uint32_t BlockAt(int x, int y) const {
      return (y/4) * _blocks_width + (x/4);
     }

     inline size_t DictSize() const {
       return _dict_reminder.size();
     }

     inline size_t FullDictSize() const {
       return _interp_dict.size();
     }

     inline void MakeDict() {
       for( auto e : _physical_blocks) {
         _interp_dict.insert(e.interp);
       }
     }

     inline std::vector<uint8_t> Get8BitPalette() {
       std::vector<uint8_t> result;
       result.resize(_unique_palette.size()*4, 0);
       memcpy(result.data(), _unique_palette.data(), result.size());
       return std::move(result);
     }

     uint32_t FindCommonEntries(std::unique_ptr<DXTImage> &reference) {
       uint32_t count_common = 0; 
       for(auto e : _interp_dict) {
	 if(reference->_interp_dict.find(e) != reference->_interp_dict.end()) 
	   count_common++;
       }
       
       return count_common;
     }
     
     uint32_t Error(std::unique_ptr<DXTImage> &other);
     static void SetPattern(int search_area) {

       _search_pattern.clear();
       _global_palette.clear();
       _global_palette_dict.clear();

       std::cout << "called once!" << std::endl;
       _search_pattern.push_back(std::make_tuple(0,0));

       for(int level_y = 1; level_y < search_area; level_y++) {
	 for( int curr_level = level_y; curr_level >= -1*level_y; curr_level--) {


	    if(curr_level == level_y) {
              int y = curr_level;
	      for(int x = curr_level; x >= -1*curr_level; x--) 
		_search_pattern.push_back(std::make_tuple(x,y));
	    } else if(curr_level == -1*level_y) {
	      int y = curr_level;
	      for(int x = curr_level; x <= -1*curr_level; x++)
		_search_pattern.push_back(std::make_tuple(x,y));
	    }
	    else {
	      _search_pattern.push_back(std::make_tuple(level_y, curr_level));
	      _search_pattern.push_back(std::make_tuple(-1*level_y, curr_level));
	    }

	 }
       }
     }

     int32_t _width;
     int32_t _height;
     int32_t _blocks_width;
     int32_t _blocks_height;
     int32_t _num_blocks;

     bool _is_intra;
     int _search_area;
     std::vector<int> _found_at;
     std::vector<bool> _found;
     std::set<uint32_t> _dict_reminder; // dict for reminder blocks which can't be found 
     std::set<uint32_t> _interp_dict;

     static std::vector<std::tuple<uint32_t, uint32_t> > _global_palette;
     static std::set<uint32_t> _global_palette_dict;
     std::vector<uint32_t> _unique_palette;
     std::vector<std::tuple<uint8_t, uint8_t> > _motion_indices;
     std::vector<std::tuple<uint8_t, uint8_t> > _intra_motion_indices;
     std::vector<std::tuple<uint8_t, uint8_t> > _inter_block_motion_indices;
     std::vector<std::tuple<uint8_t, uint8_t> > _inter_pixel_motion_indices;
     std::vector<uint8_t> _index_mask;

     std::vector<PhysicalDXTBlock> _physical_blocks;
     std::vector<LogicalDXTBlock> _logical_blocks;

     static std::vector<std::tuple<int,int> > _search_pattern;
     static int32_t _err_threshold;
     
     std::vector<uint8_t> _src_img;

     

  }; // end DXTImage

} // namespace MPTC
#endif
