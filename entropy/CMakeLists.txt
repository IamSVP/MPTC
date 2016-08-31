INCLUDE_DIRECTORIES("${MPTC_SOURCE_DIR}/entropy")
INCLUDE_DIRECTORIES("${MPTC_SOURCE_DIR}/googletest/include")

SET( HEADERS 
  "arithmetic_codec.h"
  )
SET( SOURCES
  "arithmetic_codec.cpp"
  )

ADD_LIBRARY(arith_codec ${HEADERS} ${SOURCES})

ADD_EXECUTABLE(bit_stream_test "bit_stream_test.cpp")
TARGET_LINK_LIBRARIES(bit_stream_test gtest_main)
ADD_TEST(Test_bit_stream bit_stream_test)
