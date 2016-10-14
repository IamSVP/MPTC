set( SOURCES
  "mpeg_codec.c"
)

add_executable(mpeg_codec mpeg_codec.c)
target_link_libraries(mpeg_codec avutil avformat avcodec z avutil m swscale)