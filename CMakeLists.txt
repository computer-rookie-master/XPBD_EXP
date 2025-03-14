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
    src/physics/collision_broad_phase.cpp
    src/physics/collision_narrow_phase.cpp
    src/physics/physics_util.cpp
    src/render/mesh.cpp
    src/render/mesh_renderer.cpp
    src/render/sphere_mesh.cpp
    src/render/cube_mesh.cpp
)


#链接库
target_link_libraries(XPBD_EXP ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw ${GLM_LIBRARIES})


# # 添加 GoogleTest
# add_subdirectory(thirdparty/googletest)
# include_directories(thirdparty/googletest/googletest/include)
# include_directories(thirdparty/googletest/googlemock/include)

# # 添加测试目标
# enable_testing()
# add_executable(XPBD_EXP_Tests
#     tests/test_collision_broad_phase.cpp  # 单元测试文件，稍后创建
#     src/physics/collision_broad_phase.cpp  # 测试目标文件
#     src/physics/entity.cpp                 # 依赖 Entity 类
#     src/physics/physics_util.cpp           # 依赖工具函数
#     src/render/mesh.cpp                    # 依赖 Mesh 类
#     src/render/sphere_mesh.cpp             # 用于创建测试网格
# )

# # 链接测试目标库
# target_link_libraries(XPBD_EXP_Tests 
#     gtest 
#     gtest_main 
#     ${GLM_LIBRARIES}  # 需要 GLM 支持
# )

# # 添加测试到 CTest
# add_test(NAME CollisionBroadPhaseTest COMMAND XPBD_EXP_Tests)
