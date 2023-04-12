cmake_minimum_required(VERSION 3.0.0)
project(cross_corr VERSION 0.1.0)

include(CTest)
enable_testing()


#add libraries and subdirectories
add_subdirectory(src)

#add executables
add_executable(cross_corr main.cpp)

#perform linking
target_link_libraries(cross_corr    PRIVATE BufferHandler
                                    PRIVATE CrossCorr)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
