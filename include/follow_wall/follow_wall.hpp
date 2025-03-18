#ifndef BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_
#define BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_

// Incluimos las bibliotecas necesarias
#include "geometry_msgs/msg/twist.hpp" 
#include "rclcpp/rclcpp.hpp" 
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

namespace bumpgo
{

using namespace std::chrono_literals;

// Definimos la clase BumpGoBehavior que hereda de la clase Node
class BumpGoBehavior : public rclcpp::Node
{
public:
  // Constructor de la clase
  BumpGoBehavior();  

private:
  // Callback para cuando se detecta choque en el bumper
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);

  // Función principal del Kobuki, se ejecuta en bucle para actualizar su comportamiento
  void control_cycle();

  // Definimos los estados posibles del robot
  static const int FORWARD = 0;  
  static const int AVOID = 1;
  static const int TURN = 2;

  // Definimos una variable para almacenar en que estado esta el Kobuki
  int state_;

  // Guardamos el tiempo del último cambio de estado del Kobuki
  rclcpp::Time state_ts_;  

  // Función para cambiar el estado dentro de la maquina de estados
  void go_state(int new_state);

  // Funciones que verifican si es necesario cambiar de estado
  bool check_forward_2_avoid();  
  bool check_turn();   

  // Duracion para cambiar en cada estado
  const rclcpp::Duration TIME {1s};

  // Velocidades del robot
  static constexpr float SPEED_LINEAR = 0.3;
  static constexpr float SPEED_ANGULAR = 1.0;

  // Suscripción al bumper
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;

  // Publisher de velocidades en /cmd_vel
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // Temporizador para ejecutar "control_cycle()""
  rclcpp::TimerBase::SharedPtr timer_; 

  // Registramos el ultimo evento del bumper
  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bump_;
};

}  // namespace bumpgo

#endif  // BUMP_BUMPGO__BUMPGO_BEHAVIOR_HPP_
