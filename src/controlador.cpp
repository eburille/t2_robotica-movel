#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <GeographicLib/UTMUPS.hpp>

class Controlador: public rclcpp::Node{
  public:
    Controlador(void);

    void angVelPublisher(void);
  
  private:
    double x_odom = 0.0;
    double y_odom = 0.0;
    double theta_odom = 0.0;
    double vel_x_odom = 0.0;
    double vel_y_odom = 0.0;
    double vel_lin_odom = 0.0;

    double theta_imu = 0.0;
    double ang_vel_imu = 0.0;
    double accel_x_imu = 0.0;    
    double accel_y_imu = 0.0;

    bool is_origin_set_ = false;
    double x_gps = 0.0;
    double y_gps = 0.0;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr satSub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub);
    void imuSubCB(const sensor_msgs::msg::Imu::SharedPtr imuSub);
    void satSubCB(const sensor_msgs::msg::NavSatFix::SharedPtr satSub);
};

Controlador::Controlador(void): Node ("controlador"){
  using std::placeholders::_1;

  odomSub_=create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&Controlador::odomSubCB,this,_1));
  imuSub_ =create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&Controlador::imuSubCB,this,_1));
  satSub_ =create_subscription<sensor_msgs::msg::NavSatFix>("gnss/fix", 1, std::bind(&Controlador::satSubCB,this,_1));

  velPub_=create_publisher<geometry_msgs::msg::Twist>("/twist_mrac_linearizing_controller/command", 100);

  int rate = 100;
  timer_=rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0/rate),std::bind(&Controlador::angVelPublisher,this));

}

void Controlador::angVelPublisher(void)
{   
    geometry_msgs::msg::Twist msg;

    geometry_msgs::msg::Vector3 lin;
    geometry_msgs::msg::Vector3 ang;
  
    msg.linear = lin;
    msg.angular = ang;

    RCLCPP_INFO(this->get_logger(), "\n Dados coletados");

    

    RCLCPP_INFO(this->get_logger(), " x_odom  | x_gps     -      y_odom  | y_gps     -    Theta_odom   | Theta_IMU ");
    RCLCPP_INFO(this->get_logger(), "  %.4f       %.4f            %.4f     %.4f             %.4f             %.4f", x_odom , x_gps, y_odom , y_gps, theta_odom, theta_imu);
}

void Controlador::odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub)
{
    x_odom = odomSub->pose.pose.position.x;
    y_odom = odomSub->pose.pose.position.y;

    double z_orientation = odomSub->pose.pose.orientation.z;
    double w_orientation = odomSub->pose.pose.orientation.w;
    theta_odom = 2*atan2(z_orientation,w_orientation);

    vel_x_odom = odomSub->twist.twist.linear.x; 
    vel_y_odom = odomSub->twist.twist.linear.x; 

    vel_lin_odom = std::sqrt(vel_x_odom*vel_x_odom + vel_y_odom*vel_y_odom);

    // RCLCPP_INFO(this->get_logger(), "Odometria recebida");
    // RCLCPP_INFO(this->get_logger(), "--- Posição Atual do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "X:     %.4f metros", x);
    // RCLCPP_INFO(this->get_logger(), "Y:     %.4f metros", y);
    // RCLCPP_INFO(this->get_logger(), "Theta: %.4f radianos (%.2f°)", theta, theta * (180.0 / M_PI));

    // RCLCPP_INFO(this->get_logger(), "--- Posição Ref do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "X:     %.4f metros", xRef);
    // RCLCPP_INFO(this->get_logger(), "Y:     %.4f metros", yRef);
    // RCLCPP_INFO(this->get_logger(), "Theta: %.4f radianos (%.2f°)\n", thetaRef, thetaRef * (180.0 / M_PI));
}

void Controlador::imuSubCB(const sensor_msgs::msg::Imu::SharedPtr imuSub){
  double z_orientation =  imuSub->orientation.z;
  double w_orientation =  imuSub->orientation.w;
  theta_imu = 2*atan2(z_orientation,w_orientation);

  ang_vel_imu = imuSub->angular_velocity.z;

  accel_x_imu = imuSub->linear_acceleration.x;
  accel_y_imu = imuSub->linear_acceleration.y;

  // RCLCPP_INFO(this->get_logger(), "\n IMU recebido\n");
  // RCLCPP_INFO(this->get_logger(), "--------------------------");
  // RCLCPP_INFO(this->get_logger(), "theta_imu:     %.4f m/s", theta_imu);
  // RCLCPP_INFO(this->get_logger(), "ang_vel_imu:     %.4f m/s", ang_vel_imu);
  // RCLCPP_INFO(this->get_logger(), "accel_x_imu:     %.4f m/s^2", accel_x_imu);
  // RCLCPP_INFO(this->get_logger(), "accel_y_imu:     %.4f m/s^2", accel_y_imu);

}

void Controlador::satSubCB(const sensor_msgs::msg::NavSatFix::SharedPtr satSub){
  // double latitude = satSub->latitude;
  // double longitude = satSub->longitude;

  // RCLCPP_INFO(this->get_logger(), "     GPS recebido");
  // RCLCPP_INFO(this->get_logger(), "--------------------------");
  // RCLCPP_INFO(this->get_logger(), "latitude:     %f", latitude);
  // RCLCPP_INFO(this->get_logger(), "longitude:     %f\n", longitude);

  // Ignora se o GPS não tiver sinal válido
  if (satSub->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      RCLCPP_WARN(this->get_logger(), "Aguardando sinal válido de GPS...");
      return;
  }

  int zone;
  bool northp;
  double utm_x, utm_y;
  
  // Coordenadas no ponto 0
  static double origin_x_ = 0.0;
  static double origin_y_ = 0.0;


  try {
    // Converte Lat/Lon atuais para UTM usando GeographicLib
    GeographicLib::UTMUPS::Forward(satSub->latitude, satSub->longitude, zone, northp, utm_x, utm_y);
            
    // Define a origem local com a primeira coordenada válida recebida
    if (!is_origin_set_) {
      origin_x_ = utm_x;
      origin_y_ = utm_y;
      is_origin_set_ = true;
      RCLCPP_INFO(this->get_logger(), "Origem definida! Zona UTM: %d%s", zone, (northp ? "N" : "S"));
      RCLCPP_INFO(this->get_logger(), "X0: %.3f m, Y0: %.3f m", origin_x_, origin_y_);
    }

    // Calcula a posição cartesiana local relativa à origem (0,0)
    x_gps = -(utm_x - origin_x_);
    y_gps = utm_y - origin_y_;

    // Aqui você tem suas coordenadas cartesianas prontas para usar no ROS
    RCLCPP_INFO(this->get_logger(), "X (Leste): %.3f m | Y (Norte): %.3f m", x_gps, y_gps);

    // Dica: Aqui você poderia publicar um Odometry msg ou um TF para o RViz
            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Erro na conversão: %s", e.what());
    }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  printf("hello world t2 package\n");

  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Controlador>());
  rclcpp::shutdown();

  return 0;
}
