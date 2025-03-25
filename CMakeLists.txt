cmake_minimum_required(VERSION 3.8)
project(p4b-follow-wall-pepineros)

# Definimos algunas opciones de compilación si el compilador es GCC o Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontramos los paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Establecemos las dependencias
set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Añadimos el directorio de encabezados (include)
include_directories(include)

# Definimos la librería de comportamiento del robot
add_library(follow_wall_behavior SHARED
  src/FollowWall/FollowWall.cpp
)

# Añadimos las dependencias a la librería
ament_target_dependencies(follow_wall_behavior ${dependencies})

# Definimos el nodo principal
add_executable(follow_wall_main src/FollowWallMain.cpp)

# Vinculamos las dependencias al nodo principal
ament_target_dependencies(follow_wall_main ${dependencies})
target_link_libraries(follow_wall_main follow_wall_behavior)

# Instalamos el nodo y la librería
install(TARGETS
  follow_wall_main
  follow_wall_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Configuración de pruebas si se requiere
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Exportamos las dependencias para otras partes del proyecto
ament_export_dependencies(${dependencies})
ament_package()
