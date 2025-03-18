cmake_minimum_required(VERSION 3.8)
project(p4-bump-and-go-pepineros)

# Definimos algunas opciones de compilación si el compilador es GCC o Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontramos los paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

# Establecemos las dependencias
set(dependencies
  rclcpp
  geometry_msgs
  kobuki_ros_interfaces
)

# Añadimos el directorio de encabezados (include)
include_directories(include)

# Definimos la librería de comportamiento del robot
add_library(bumpgo_behavior SHARED
  src/bumpgo/bumpgo_behavior.cpp
)

# Añadimos las dependencias a la librería
ament_target_dependencies(bumpgo_behavior ${dependencies})

# Definimos el nodo principal
add_executable(bumpgo_main src/bumpgo_main.cpp)

# Vinculamos las dependencias al nodo principal
ament_target_dependencies(bumpgo_main ${dependencies})
target_link_libraries(bumpgo_main bumpgo_behavior)

# Instalamos el nodo y la librería
install(TARGETS
  bumpgo_main
  bumpgo_behavior
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
