project (nes)


include(FetchContent)


FetchContent_Declare(
  SDL3
  GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
  GIT_TAG release-3.2.14
)
FetchContent_MakeAvailable(SDL3)


add_executable(nes 
    mapper.c
    rom.c
    main.c
)
add_definitions(-g)
set(CMAKE_BUILD_TYPE Debug)

target_link_libraries(nes PRIVATE SDL3::SDL3 )
