INCLUDE_DIRECTORIES("${MPTC_SOURCE_DIR}/entropy")
INCLUDE_DIRECTORIES("${MPTC_SOURCE_DIR}/googletest/include")

SET( HEADERS 
  "arithmetic_codec.h"
  )
SET( SOURCES
  "arithmetic_codec.cpp"
  )

ADD_LIBRARY(arith_codec ${HEADERS} ${SOURCES})


