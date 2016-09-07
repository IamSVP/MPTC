# First add alll the include directories from other projects and folders
# Second add all external link libraries, set their paths and properties 
# Set Headers and Sources from it's own files, use glob or add each file 
# ADD_LIBRARY, ADD_EXECUTABLE with the library name 
# Target Link libraries, add libraries on which this is depended on 

INCLUDE_DIRECTORIES("${MPTC_SOURCE_DIR}/Include")
set( SOURCES "main.cpp")

ADD_EXECUTABLE( mptc_index ${SOURCES})
TARGET_LINK_LIBRARIES(mptc_index squish)

