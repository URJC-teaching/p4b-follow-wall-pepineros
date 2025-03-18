// Incluimos las bibliotecas necesarias
#include "bumpgo/bumpgo_behavior.hpp"  
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"  


namespace bumpgo
{
  using namespace std::chrono_literals;
  using std::placeholders::_1;


  // Constructor del nodo `BumpGoBehavior`
  BumpGoBehavior::BumpGoBehavior() : rclcpp::Node("bumpgo_behavior"),
    // Lo iniciamos en estado FORWARD (para que empiece avanzando)  
    state_(FORWARD)  
  {
    // Subscriber al bumper para detectar colisiones
    bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
      "/events/bumper", 10, std::bind(&BumpGoBehavior::bumper_callback, this, _1));

    // Publisher de velocidades en el topic "/cmd_vel"
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Creamos un temporizador que ejecuta "control_cycle()"" cada 50ms
    timer_ = create_wall_timer(50ms, std::bind(&BumpGoBehavior::control_cycle, this));

    // Guardamos el tiempo actual
    state_ts_ = now();

    // Iniciamos la variable del bumper sin detectar colisión
    last_bump_ = std::make_unique<kobuki_ros_interfaces::msg::BumperEvent>();
    last_bump_->state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;

    // Establecemos el estado inicial del robot como FORWARD (para que vaya para adelante)
    go_state(FORWARD);
  }

  // Creamos el Callback que se ejecuta cuando se detecta un evento del bumper
  void BumpGoBehavior::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
  {
    // Guardamos el nuevo estado del bumper
    last_bump_ = std::move(msg);  
  }

  // Funcion para decidir el comportamiento del robot
  void BumpGoBehavior::control_cycle()
  {
    // Inicializamos las velocidades a 0
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    // MAQUINA DE ESTADOS
    switch (state_) {  

        // Estado de avanzar
        case FORWARD:
            cmd_vel.linear.x = SPEED_LINEAR;  // Velocidad definida en el .hpp

            // Si detectamos un obstáculo, cambiamos a AVOID
            if (check_forward_2_avoid()) {
                go_state(AVOID);
            }
            break;

        // Estado de retroceder
        case AVOID:
            // Retrocede (en negativo) a la misma velocidad
            cmd_vel.linear.x = -SPEED_LINEAR;  

            // Si ya ha retrocedido suficiente tiempo, pasa a girar
            if (check_turn()) {  
                go_state(TURN);
            }
            break;

        // Estado de girar
        case TURN:  
            cmd_vel.angular.z = SPEED_ANGULAR;  // Velocidad definida en el .hpp

            // Si ya ha girado suficiente tiempo, vuelve a avanzar
            if (check_turn()) {  
                go_state(FORWARD);
            }
            break;
    }

    // Publicamos el comando de velocidad en "/cmd_vel"
    vel_pub_->publish(cmd_vel);  
  }

  // Función para cambiar de estado
  void BumpGoBehavior::go_state(int new_state)
  {
    // Guardamos el nuevo estado
    state_ = new_state;
    
    // Actualizamos la marca de tiempo del cambio de estado
    state_ts_ = now();  
  }

  // Comprobamos si se detecta choque
  bool BumpGoBehavior::check_forward_2_avoid()
  {
    static bool bumper_was_pressed = false;

    if (last_bump_->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED) {
        if (!bumper_was_pressed) {  
            bumper_was_pressed = true;

            // Si el bumper se presionó por primera vez, cambiamos a AVOID
            return true;  
        }
    } else bumper_was_pressed = false;  // Si no se presiona el bumpper seguimos en False
    
    return false;
  }

  // Comprobamos que el giro del Kobuki
  bool BumpGoBehavior::check_turn()
  {
    // Si ha pasado TIME (2s), pasamos de estado
    return (now() - state_ts_) > TIME;  
  }

}

