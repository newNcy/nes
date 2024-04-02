project (nes)

add_subdirectory(3rd)

add_executable(nes 
    mapper.c
    rom.c
    nes.c
)
add_definitions(-g)
set(CMAKE_BUILD_TYPE Debug)

target_link_libraries(nes SDL3_Headers SDL3-shared)
