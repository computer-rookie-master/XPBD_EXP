cmake_minimum_required(VERSION 3.10)
project(XPBD_EXP)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# OPENGL
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
set(GLEW_LIBRARIES "/opt/homebrew/opt/glew/lib/libGLEW.dylib")
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
set(GLM_LIBRARIES "/opt/homebrew/opt/glm/lib/libglm.dylib")

# 添加头文件路径
include_directories(include ${OPENGL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIRS} ${GLM_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)

# 添加源文件
add_executable(XPBD_EXP
    src/main.cpp
    src/physics/xpbd.cpp
    src/physics/entity.cpp
    src/physics/collider.cpp
    src/physics/collision_broad_phase.cpp
    src/physics/collision_narrow_phase.cpp
    src/render/mesh.cpp
    src/render/mesh_renderer.cpp
    src/render/sphere_mesh.cpp
    src/render/cube_mesh.cpp
)

#链接库
target_link_libraries(XPBD_EXP ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw ${GLM_LIBRARIES})
