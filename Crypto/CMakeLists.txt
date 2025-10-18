cmake_minimum_required(VERSION 3.10)
project(CRYPTO_TOOL VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(crypto_tool 
    src/Crypto_tool.cpp
)

target_include_directories(crypto_tool PUBLIC include/)
target_compile_options(crypto_tool PRIVATE -O2 -Wall)

add_executable(main src/main.cpp)

target_compile_options(main PRIVATE -O2 -Wall)
target_link_libraries(main PRIVATE crypto_tool)