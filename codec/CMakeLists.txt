include_directories("${MPTC_SOURCE_DIR}/codec")
include_directories("${MPTC_BINARY_DIR}/codec")
include_directories("${MPTC_SOURCE_DIR}/entropy")
include_directories("${MPTC_SOURCE_DIR}/Include")

set( HEADERS 
  "dxt_image.h"
  "pixel_traits.h"
  "image.h"
  "codec.h"
  "image_utils.h"
  "pipeline.h"
  "image_processing.h"
  "wavelet.h"
  "entropy.h"
)

set( SOURCES
  "dxt_image.cpp"
  "image.cpp"
  "codec.cpp"
  "wavelet.cpp"
  "image_processing.cpp"
)

add_library(mptc_codec ${HEADERS} ${SOURCES})
target_link_libraries(mptc_codec arith_codec)
add_executable(mptc_encoder encoder.cpp)
target_link_libraries(mptc_encoder mptc_codec arith_codec)
