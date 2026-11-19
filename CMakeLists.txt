cmake_minimum_required(VERSION 3.20)
project(FkToIkBake LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Engine: plain C++17, no Maya dependency.
add_library(fk2ik_engine STATIC
    engine/BakeEngine.cpp
    engine/ControlSolver.cpp
    engine/KeyReducer.cpp
    engine/LimbMatcher.cpp
    engine/RigMap.cpp)
set_target_properties(fk2ik_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MSVC)
    target_compile_options(fk2ik_engine PRIVATE /O2 /fp:precise /W4)
else()
    target_compile_options(fk2ik_engine PRIVATE -O2 -Wall -Wextra)
endif()

# Engine validation suite (no Maya needed):  cmake --build . --target fk2ik_tests
option(FK2IK_BUILD_TESTS "Build the Maya-independent engine tests" ON)
if(FK2IK_BUILD_TESTS)
    add_executable(fk2ik_tests tests/test_main.cpp tests/SyntheticRig.cpp)
    target_link_libraries(fk2ik_tests PRIVATE fk2ik_engine)
endif()

# ---------------------------------------------------------------------------
# Maya plugin.  Point MAYA_DEVKIT at the Maya 2027 devkit root (the folder
# containing include/ and lib/), or MAYA_LOCATION at the Maya install.
set(MAYA_DEVKIT "" CACHE PATH "Maya 2027 devkit root (contains include/maya and lib)")
set(MAYA_LOCATION "" CACHE PATH "Maya 2027 install root (fallback for lib/)")

set(_maya_inc "")
set(_maya_lib "")
foreach(_root IN ITEMS "${MAYA_DEVKIT}" "${MAYA_LOCATION}")
    if(_root AND NOT _maya_inc AND EXISTS "${_root}/include/maya/MFnPlugin.h")
        set(_maya_inc "${_root}/include")
    endif()
    if(_root AND NOT _maya_lib AND (EXISTS "${_root}/lib/OpenMaya.lib" OR EXISTS "${_root}/lib/libOpenMaya.so"
                                     OR EXISTS "${_root}/Maya.app/Contents/MacOS/libOpenMaya.dylib"))
        if(EXISTS "${_root}/Maya.app/Contents/MacOS")
            set(_maya_lib "${_root}/Maya.app/Contents/MacOS")
        else()
            set(_maya_lib "${_root}/lib")
        endif()
    endif()
endforeach()

if(NOT _maya_inc OR NOT _maya_lib)
    message(WARNING "Maya devkit not found (set -DMAYA_DEVKIT=... or -DMAYA_LOCATION=...). "
                    "Only the engine and its tests will be built.")
    return()
endif()
message(STATUS "Maya include: ${_maya_inc}")
message(STATUS "Maya libs   : ${_maya_lib}")

add_library(fkToIkBake SHARED
    maya/PluginMain.cpp
    maya/FkToIkBakeCmd.cpp
    maya/MayaRig.cpp)
target_include_directories(fkToIkBake PRIVATE "${_maya_inc}")
target_link_libraries(fkToIkBake PRIVATE fk2ik_engine)

if(WIN32)
    target_link_libraries(fkToIkBake PRIVATE
        "${_maya_lib}/OpenMaya.lib" "${_maya_lib}/OpenMayaAnim.lib" "${_maya_lib}/Foundation.lib")
    target_compile_definitions(fkToIkBake PRIVATE NT_PLUGIN REQUIRE_IOSTREAM _USE_MATH_DEFINES WIN32 _WINDOWS NOMINMAX)
    target_compile_options(fkToIkBake PRIVATE /O2 /fp:precise /W3 /MD)
    set_target_properties(fkToIkBake PROPERTIES SUFFIX ".mll" PREFIX "")
elseif(APPLE)
    target_link_libraries(fkToIkBake PRIVATE
        "${_maya_lib}/libOpenMaya.dylib" "${_maya_lib}/libOpenMayaAnim.dylib" "${_maya_lib}/libFoundation.dylib")
    target_compile_definitions(fkToIkBake PRIVATE OSMac_ MAC_PLUGIN REQUIRE_IOSTREAM)
    set_target_properties(fkToIkBake PROPERTIES SUFFIX ".bundle" PREFIX "")
else()
    target_link_libraries(fkToIkBake PRIVATE
        "${_maya_lib}/libOpenMaya.so" "${_maya_lib}/libOpenMayaAnim.so" "${_maya_lib}/libFoundation.so")
    target_compile_definitions(fkToIkBake PRIVATE LINUX REQUIRE_IOSTREAM _BOOL)
    set_target_properties(fkToIkBake PROPERTIES SUFFIX ".so" PREFIX "")
endif()

# Copy the built plugin into the installable package layout.
if(WIN32)
    set(_plat win64)
elseif(APPLE)
    set(_plat mac)
else()
    set(_plat linux)
endif()
add_custom_command(TARGET fkToIkBake POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/package/FkToIkBake/plug-ins/${_plat}/2027"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:fkToIkBake>"
            "${CMAKE_SOURCE_DIR}/package/FkToIkBake/plug-ins/${_plat}/2027/")
