#declare libraries
add_library(BufferHandler   BufferHandler.cpp)
add_library(CrossCorr       CrossCorr.cpp)

#add in subdirectories

#link libraries as needed
target_link_libraries(CrossCorr     PRIVATE     BufferHandler)