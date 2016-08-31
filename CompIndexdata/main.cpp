#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cassert>
#include <cstddef>
#include <map>
#include <set>
#include <tuple>
#include <sstream>
#include <iomanip>
#include <squish/squish.h>


#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_DXT_IMPLEMENTATION
#include "stb_dxt.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


int search_area;
void createImage(){
  
  uint32_t W = 512, H = 512;
  std::vector<uint32_t> img(W*H);
  for(uint32_t h = 0; h < H; h+=2){
    for(uint32_t w = 0; w < W; w+=2){        

      uint32_t color;
      if( (w/2 + h/2) % 2 == 0)   
        color = 0xFF000000;
      else color = 0xFFFFFFFF;
      
      for(int j = 0 ; j < 2; j++){
        for(int i = 0; i < 2; i++){
          uint32_t pix_idx = (w + i) * W + (h +j);
          img[pix_idx] = color;
        }
      }

    }
  }
  stbi_write_png("test.png", W, H, 4, reinterpret_cast<uint8_t*>(img.data()), 4*W);
  
}

typedef struct _logical_block{
  uint16_t ep1;
  uint16_t ep2;
  uint32_t interpolation;
} logical_block;

typedef union _block_data {
  logical_block B;
  uint64_t data;
} block_data;

std::vector<uint8_t> GetIndexFrames(std::string imgFile, std::vector<uint32_t> &Inter){

  int W, H, n;
  uint8_t *img = stbi_load(imgFile.c_str(), &W, &H, &n, 4);
  if( img == NULL) assert(true && "image pointer null");
  uint32_t dim_x = (W + 3) >> 2;
  uint32_t dim_y = (H + 3) >> 2;
  uint32_t num_blocks = dim_x * dim_y;

  uint32_t block[16];
  std::vector<uint64_t> dxt_blocks;
  uint32_t *img_data = reinterpret_cast<uint32_t*> (img);

  for(int block_y = 0; block_y < H/4; block_y++){
    for(int block_x = 0; block_x < W/4; block_x++) {

       int block_idx = block_y * W/4 + block_x; 

       for( int j = 0; j < 4; j++){
         for(int i = 0; i < 4; i++){

           int pix_idx = (block_y * 4 + j) * W + (block_x * 4 + i); 
	   block[j*4+i] = img_data[pix_idx];
           //block[4*(j*4+i)+0] = img[4*pix_idx+0];
	   //block[4*(j*4+i)+1] = img[4*pix_idx+1];
           //block[4*(j*4+i)+2] = img[4*pix_idx+2];
	   //block[4*(j*4+i)+3] = img[4*pix_idx+3];

         }
       }
       uint8_t cmp[8];
       stb_compress_dxt_block(reinterpret_cast<uint8_t*>(cmp), reinterpret_cast<uint8_t*>(block), 1, 0);
       //squish::Compress((block),(cmp), squish::kDxt1);
       dxt_blocks.push_back(cmp[1]); 

    }
  }
  /// Get the index data back;
  std::vector<uint8_t> ep1_img, ep2_img;
  std::vector<uint8_t> interpolation_img(W*H);

  std::vector<uint8_t> interpolation_img1, interpolation_img2, interpolation_img3, interpolation_img4;
  block_data D;
  

  for(uint32_t block_idx  = 0; block_idx < num_blocks ; block_idx++){

    D.data = dxt_blocks[block_idx];
    uint16_t ep1 = D.B.ep1;
    uint16_t ep2 = D.B.ep2;
    uint8_t r,g,b,a = 255;
    uint16_t dummy = 0;  
    
    
    dummy = (ep1 & 0xF800) | (( ep1 & 0xE000) >> 5);   
    r = (dummy >> 8);

    dummy = (ep1 & 0x07E0) << 5;
    dummy = dummy| ((dummy & 0xC000) >> 6);
    g = dummy >> 8;

    dummy = (ep1 & 0x001F) << 11;
    dummy = dummy | ((dummy & 0xE000) >> 5);
    b = dummy >> 8;

    ep1_img.push_back(r);ep1_img.push_back(g);ep1_img.push_back(b);ep1_img.push_back(a);

    dummy = (ep2 & 0xF800) | (( ep2 & 0xE000) >> 5); 
    r = (dummy >> 8);

    dummy = (ep2 & 0x07E0) << 5;
    dummy = dummy | ((dummy & 0xC000) >> 6);
    g = dummy >> 8;

    dummy = (ep2 & 0x001F) << 11;
    dummy = dummy | ((dummy & 0xE000) >> 5);
    b = dummy >> 8;
    ep2_img.push_back(r);ep2_img.push_back(g);ep2_img.push_back(b);ep2_img.push_back(a);
 
    uint32_t interpolation = D.B.interpolation;
    Inter.push_back(interpolation);
 
    int block_x = block_idx % (W/4);
    int block_y = block_idx / (W/4);
    uint32_t gray = interpolation & ( 0xFF000000 ); 
    uint8_t p;
    p = gray >> 24;
    interpolation_img1.push_back(p);

    gray = interpolation & (0x00FF0000);
    p = gray >> 16;
    interpolation_img2.push_back(p);
 
    gray = interpolation & (0x0000FF00);
    p = gray >> 8;
    interpolation_img3.push_back(p);
   
    gray = interpolation & (0x000000FF);
    p = gray;
    interpolation_img4.push_back(gray);

      for(int i = 15; i >= 0; i--){

        int pix_idx = (block_y * 4 + i/4) * W + (block_x * 4 + i%4); 
        uint8_t pix_val = (interpolation & 3) * 85; 
        interpolation_img[pix_idx] = pix_val;
        interpolation = interpolation >> 2;

      }
  } 
  std::string outFile = imgFile.substr(0, imgFile.find_last_of(".")) + "_Index.png"; 

  stbi_write_png(outFile.c_str(), W, H, 1, interpolation_img.data(), W);

  outFile = imgFile.substr(0, imgFile.find_last_of(".")) + "_ep1.png";
  stbi_write_png(outFile.c_str(), W/4, H/4, 4, ep1_img.data(), W);

  outFile = imgFile.substr(0, imgFile.find_last_of(".")) + "_ep2.png";
  stbi_write_png(outFile.c_str(), W/4, H/4, 4, ep2_img.data(), W);

  return interpolation_img;
   
}

std::vector<uint8_t> Get4X4Block(std::vector<uint8_t> &img, uint32_t W, uint32_t H, uint32_t x, uint32_t y){
  std::vector<uint8_t> block;
  for(uint32_t j = 0; j < 4; j++){
    for(uint32_t i = 0; i < 4; i++){
      uint32_t pix_idx = (y + j) * W + (x + i);
      block.push_back(img[pix_idx]);
    }
  }

  return block;
}

void TotalDictSize(std::string imgPath, uint32_t frame_idx){

  std::set<uint32_t> total_dict,current_dict;
  block_data D;
  std::vector<uint8_t> interpolation;
  std::vector<uint32_t> interpolation32;
  uint32_t num_blocks = ( 1920/4 * 1080/4);
  uint32_t W = 1920;
  uint32_t H = 1080;
  std::pair<std::set<uint32_t>::iterator,bool> ret_total;
  std::pair<std::set<uint32_t>::iterator,bool> ret_curr;

  //Frame Index from which dictionary is calculated 
  for(uint32_t frame = frame_idx; frame <= frame_idx + 1; frame++){

    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << frame;
    std::string frame_str = ss.str();
    std::string file_name = imgPath + "/" + frame_str + ".png";
    interpolation.clear(); interpolation32.clear(); 
    interpolation = GetIndexFrames(file_name, interpolation32);
    std::vector<uint8_t> blocks_found_img(W/4 * H/4);

    for(int i = 0; i < num_blocks; i++){

      current_dict.insert(interpolation32[i]);
      ret_total = total_dict.insert(interpolation32[i]);
      if(ret_total.second == false) blocks_found_img[i] = 255;
      else blocks_found_img[i] = 0;

    }
    	
    std::cout << "Frame:" << frame_str << "--" << current_dict.size() << "--" << total_dict.size() << std::endl;
    file_name = frame_str + "_found.png";
    stbi_write_png(file_name.c_str(), W/4, H/4, 1, blocks_found_img.data(), W/4);
      

    current_dict.clear();
  }


}


void Prediction(std::string imgPath, uint32_t frame_idx) {

  uint32_t W = 1920;
  uint32_t H = 1080;

  std::vector<int32_t> errs;
  std::vector< std::tuple<int32_t, int32_t> > index;
  std::vector<uint8_t> interpolationPrev, interpolationCurr;
  std::vector<uint32_t> inter32Prev, inter32Curr;
  std::set<uint32_t> dict_rest;
  
  
  std::stringstream ss;
  ss << std::setw(5) << std::setfill('0') << frame_idx;
  std::string frame_str = ss.str();
  std::string file_name = imgPath + "/" + frame_str + ".png";
  
  interpolationPrev = GetIndexFrames(file_name, inter32Prev);

  ss.str("");
  ss << std::setw(5) << std::setfill('0') << frame_idx + 1;
  frame_str = ss.str();
  file_name = imgPath + "/" + frame_str + ".png";

  interpolationCurr = GetIndexFrames(file_name, inter32Curr);
 
  std::vector<uint8_t> blocks_found_img(W/4 * H/4);

  uint32_t countBlocks = 0;
  for(int h = 0; h < H; h+=4){
    for(int w = 0; w < W; w+=4){

      std::vector<uint8_t> blockCurr = Get4X4Block(interpolationCurr, 1920, 1080, w, h);
      int32_t x_idx = -1, y_idx = -1, ERR = 100;

      for(int i = -1 * search_area; i <= search_area; i++){
        for(int j = -1 * search_area; j <= search_area; j++){

	  if( w+i >= 0 && w+i <= W-4 && h+j >= 0 && h+j <= H-4){ 

            std::vector<uint8_t> blockPrev = Get4X4Block(interpolationPrev, 1920, 1080, w+i, h+j);
            uint64_t MSE  = 0;
            for( uint32_t idx = 0; idx < 16; idx++){

              int err = std::abs(blockCurr[idx]/85 - blockPrev[idx]/85);
	      MSE += err;

            }

	    if(MSE < ERR ) {
	      ERR = MSE; x_idx = w+i; y_idx = h+j; 
	      break;
	    }

	}
      }
      if(ERR == 0) break;
    }

      //std::cout << "X:" << x_idx << "," << "Y:" << y_idx << "--ERR:" << ERR << std::endl;
      uint32_t block_idx = h/4 * W/4 + w/4;
      if( ERR == 0){
	countBlocks++; 
	blocks_found_img[block_idx] = 255;
      }
      else{
	blocks_found_img[block_idx] = 0;
	dict_rest.insert(inter32Curr[block_idx]);
      }
      errs.push_back(ERR);
    }
  }
  std::cout<< "Num of blocks found:" << countBlocks << std::endl;
  std::cout << "Size of dict for rest:" << dict_rest.size() << std::endl;
  std::string blocks_found = "Blocks_found_motion" + frame_str + ".png";
  stbi_write_png(blocks_found.c_str(), W/4, H/4, 1, blocks_found_img.data(), W/4);


}

int main(int argc, char *argv[]) {

  // Read two conseqcutive frames 
  // convert the first to DXT1 using stb_img file
  // Seperate the index data and other stuff from the first DXT1 
  // Now write a DXT1 compressor function for the next frame using the ** Here Ideas ** 
  // May be open CV. Don't know, please keep it as cpp as possible. 

  if(argc < 2 ){
    std::cout<< "Image file argument required" << std::endl;
    exit(-1);
  }

  std::vector<uint8_t> interpolationPrev, interpolationCurr;
  
  std::string imgPath(argv[1]);
  uint32_t frame_idx = std::stoi(argv[2], nullptr, 10);
  search_area = std::stoi(argv[3], nullptr, 10);
  //std::string imgFile3(argv[3]);

  //interpolationPrev = GetIndexFrames(imgFile1, inter32Prev); 
  //interpolationCurr = GetIndexFrames(imgFile2, inter32Curr);
 
  //dxt_blocksFut = GetIndexFrames(imgFile3);
  //TotalDictSize(imgPath, frame_idx);
  //Prediction(imgPath, frame_idx);

  return 0;  
}
