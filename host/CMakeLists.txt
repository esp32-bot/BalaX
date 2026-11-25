# Incremental laptop builds of the status screen renderer, for `bx frames` and the live preview.
#
#   cmake -S host -B host/build/live-native -G Ninja            frames CLI
#   emcmake cmake -S host -B host/build/live-wasm -G Ninja      preview.mjs + preview.wasm
#
# The flags match tools/bx's host builds. The firmware itself builds with ESP-IDF, not this.
cmake_minimum_required(VERSION 3.22)
project(balax_host CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(REPO "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
# Shared components come from the esp32-tools/tools checkout, like the firmware's (see ../CMakeLists.txt).
if(EXISTS "${REPO}/../../esp32-tools/tools/components")
    get_filename_component(SHARED_COMP "${REPO}/../../esp32-tools/tools/components" ABSOLUTE)
elseif(EXISTS "${REPO}/../tools/components")
    get_filename_component(SHARED_COMP "${REPO}/../tools/components" ABSOLUTE)
else()
    set(SHARED_COMP "${REPO}/components")
endif()

# An object library, so an edit relinks without re-archiving.
add_library(status_render OBJECT "${REPO}/main/status_view.cpp" "${SHARED_COMP}/gfx/canvas.cpp")
target_include_directories(status_render PUBLIC
    "${SHARED_COMP}/gfx/include" "${SHARED_COMP}/m5stickc_plus/include" "${REPO}/main" "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_options(status_render PUBLIC -O2 -Wall -Wextra -Werror)

if(EMSCRIPTEN)
    add_executable(preview_wasm preview_wasm.cpp)
    target_link_libraries(preview_wasm PRIVATE status_render)
    set_target_properties(preview_wasm PROPERTIES OUTPUT_NAME preview SUFFIX ".mjs")
    # -O1 at link: the -O2 link's extra wasm-opt pass costs about 0.4 s per save for no visible gain.
    target_link_options(preview_wasm PRIVATE -O1
        -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createPreview -sENVIRONMENT=web,worker,node
        -sFILESYSTEM=0 -sEXPORTED_RUNTIME_METHODS=HEAPU16,UTF8ToString)
else()
    add_executable(frames frames.cpp)
    target_link_libraries(frames PRIVATE status_render)
    target_compile_definitions(frames PRIVATE FRAMES_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/goldens")
endif()
