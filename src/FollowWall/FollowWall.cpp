#include <memory> 
#include <algorithm>
#include "FollowWall/FollowWall.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp" 

namespace FollowWall
{

using std::placeholders::_1;  

// Constructor de la clase FollowWallNode
FollowWallNode::FollowWallNode()
: Node("follow_wall_node")  // Inicializamos el nodo con el nombre "follow_wall_node"
{
  // Declaramos y obtenemos los parámetros
  declare_parameter("min_distance", min_distance_);
  declare_parameter("distance_to_wall", distance_to_wall_);
  get_parameter("min_distance", min_distance_);
  get_parameter("distance_to_wall", distance_to_wall_);

  // Suscripción al topic "scan_raw" (Gazebo) o scan_filtered (Kobuki)
  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_filtered", rclcpp::SensorDataQoS().reliable(),
    std::bind(&FollowWallNode::laser_callback, this, _1));

  // Publicador para indicar si hay un obstáculo en el topic "obstacle" (en google pone que el Kobuki lo tiene)
  obstacle_pub_ = create_publisher<std_msgs::msg::Bool>("obstacle", 100);

  // Publicador para enviar comandos de velocidad al robot en el topic "cmd_vel"
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

// Callback que procesa los datos del Lidar
void 
FollowWallNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  RCLCPP_INFO(get_logger(), "Obstacle in (%li)", scan->ranges.size());

  // Encontramos la distancia mínima detectada por el Lidar
  int min_idx = std::min_element(scan->ranges.begin(), scan->ranges.end()) - scan->ranges.begin();
  float distance_min = scan->ranges[min_idx];

  // Zona de delante del Lidar
  float min_dist = scan->range_max;
  for(int idx=0; idx<19; idx++)
  {
    float dist = scan->ranges[idx];
    if (dist < min_dist) min_dist = dist;
  }

  for(int idx=340; idx<359; idx++)
  {
    float dist = scan->ranges[idx];
    if (dist < min_dist) min_dist = dist;

    RCLCPP_INFO(get_logger(), "min_dist in (%f)", min_dist);
  }


  // Zona izquierda del Lidar



  // Mensajes que se publicarán
  auto obstacle_msg = std_msgs::msg::Bool();  // Para indicar si hay un obstáculo
  auto cmd_vel_msg = geometry_msgs::msg::Twist();  // Para controlar el movimiento del robot

  // Máquina de estados para decidir la acción del robot
  switch (estado_actual) {
    case Estado::SIGUIENDO_PARED:
      if (distance_min > distance_to_wall_ + 0.2) {  // Si está demasiado lejos de la pared
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = -0.1;  // Corrige ligeramente hacia la pared
      } else if (distance_min < distance_to_wall_ - 0.2) {  // Si está demasiado cerca de la pared
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.2;  // Se aleja ligeramente de la pared
      } else {  // Si está a la distancia correcta
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.0;  // Sigue recto
      }
      break;

    case Estado::BUSCANDO_PARED:
      if (distance_min > 2.0) {  // Si no hay pared cerca, sigue avanzando recto
        cmd_vel_msg.linear.x = 0.2;
        cmd_vel_msg.angular.z = 0.0;
      } else {  // Si encuentra una pared, cambia al estado de seguir la pared
        estado_actual = Estado::SIGUIENDO_PARED;
      }
      break;

    case Estado::EVADE_OBSTACULO:
      if (distance_min < min_distance_) {  // Si hay un obstáculo muy cerca
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.5;  // Gira a la izquierda para evitarlo
      } else {  // Si el obstáculo ha sido evitado, vuelve a seguir la pared
        estado_actual = Estado::SIGUIENDO_PARED;
      }
      break;
  }

  // Publica el mensaje con los comandos de velocidad
  cmd_vel_pub_->publish(cmd_vel_msg);

  // Determina si hay un obstáculo demasiado cerca y lo publica
  obstacle_msg.data = (distance_min < min_distance_);
  if (obstacle_msg.data) {
    estado_actual = Estado::EVADE_OBSTACULO;  // Si hay obstáculo, cambia al estado de evasión
  }
  obstacle_pub_->publish(obstacle_msg);
}

}  // namespace FollowWall
