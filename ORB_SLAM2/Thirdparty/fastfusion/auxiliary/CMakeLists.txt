add_library(auxiliary  SHARED ${CPU_LIBRARY_TYPE}
debug.cpp memory.cpp threadpool.cpp plywriter.cpp ocv_tools.cpp)
#cuda_add_library(auxiliaryGPU ${GPU_LIBRARY_TYPE}
#cuda_basic.cu
#OPTIONS ${CUDAOPTIONS})
target_link_libraries(auxiliary 
${OpenCV_LIBS})
