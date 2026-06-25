#include <cstdio>
#include <memory>
#include <numbers>
#include <iostream>
#include <eigen3/Eigen/Dense>

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

    double x_gps = 0.0;
    double y_gps = 0.0;

    double dt;
    int rate;

    // Vetor de estados: [X, Y, vel_lin, theta, vel_ang]
    Eigen::VectorXd state = Eigen::VectorXd::Zero(5);

    // Matriz de Covariância do Erro (Incerteza do estado
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(5, 5) * 1.0; // Incerteza inicial
        
    // Configuração de ruído do processo (Ajustar baseado no robô)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr satSub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub);
    void imuSubCB(const sensor_msgs::msg::Imu::SharedPtr imuSub);
    void satSubCB(const sensor_msgs::msg::NavSatFix::SharedPtr satSub);

    void predictKalmanState();
};

Controlador::Controlador(void): Node ("controlador"){
  rate = 100;
  dt = 1/rate;

  Q(0,0) = 0.01; // Ruído de posição X
  Q(1,1) = 0.01; // Ruído de posição Y
  Q(2,2) = 0.02; // Ruído de ângulo
  Q(3,3) = 0.1;  // Ruído de velocidade linear
  Q(4,4) = 0.1;  // Ruído de velocidade angular

  using std::placeholders::_1;

  odomSub_=create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&Controlador::odomSubCB,this,_1));
  imuSub_ =create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&Controlador::imuSubCB,this,_1));
  satSub_ =create_subscription<sensor_msgs::msg::NavSatFix>("gnss/fix", 1, std::bind(&Controlador::satSubCB,this,_1));

  velPub_=create_publisher<geometry_msgs::msg::Twist>("/twist_mrac_linearizing_controller/command", 100);

  timer_=rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0/rate),std::bind(&Controlador::angVelPublisher,this));

}

void Controlador::angVelPublisher(void)
{   
    geometry_msgs::msg::Twist msg;

    geometry_msgs::msg::Vector3 lin;
    geometry_msgs::msg::Vector3 ang;
  
    msg.linear = lin;
    msg.angular = ang;

    Controlador::predictKalmanState();

    // RCLCPP_INFO(this->get_logger(), "Dados coletados");
    // RCLCPP_INFO(this->get_logger(), "x_odom | x_gps  -  y_odom | y_gps  -  ang_odom | ang_IMU ");
    // RCLCPP_INFO(this->get_logger(), "%.4f  %.4f   %.4f  %.4f    %.4f   %.4f\n", x_odom , x_gps, y_odom , y_gps, theta_odom, theta_imu);

    // RCLCPP_INFO(this->get_logger(), "z_ori_odom | z_ori_IMU  -  w_ori_odom | w_ori_IMU");
    // RCLCPP_INFO(this->get_logger(), "%.4f  %.4f            %.4f  %.4f \n", (z_orientation_odom) , (z_orientation_imu), (w_orientation_odom) , (w_orientation_imu));
}

void Controlador::predictKalmanState(){
  double theta = state(2);
  double v = state(3);
  double omega = state(4);

  // Atualiza o estado usando o modelo não-linear f(x)
  state(0) += v * std::cos(theta) * dt;
  state(1) += v * std::sin(theta) * dt;
  state(2) += omega * dt;
  // v e omega mantêm-se constantes na predição pura (modelo de velocidade constante)

  // Normaliza theta entre -PI e +PI
  state(2) = std::atan2(std::sin(state(2)), std::cos(state(2)));

  // Calcula a matriz Jacobiana A baseada no estado atual
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(5, 5);
  A(0, 2) = -v * std::sin(theta) * dt;
  A(0, 3) = std::cos(theta) * dt;
  A(1, 2) = v * std::cos(theta) * dt;
  A(1, 3) = std::sin(theta) * dt;
  A(2, 4) = dt;

  // Atualiza a covariância: P = A*P*A^T + Q
  P = A * P * A.transpose() + Q;
}

void Controlador::odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub)
{
    x_odom = odomSub->pose.pose.position.x;
    y_odom = odomSub->pose.pose.position.y;

    double y_orientation =  odomSub->pose.pose.orientation.y;
    double x_orientation =  odomSub->pose.pose.orientation.x;

    double z_orientation = odomSub->pose.pose.orientation.z;
    double w_orientation = odomSub->pose.pose.orientation.w;
    
    double yaw = atan2(2 * (w_orientation * z_orientation + x_orientation * y_orientation), 1 - 2 * (y_orientation*y_orientation + z_orientation*z_orientation));
    theta_odom = yaw;

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

  double y_orientation =  imuSub->orientation.y;
  double x_orientation =  imuSub->orientation.x;

  double yaw = atan2(2 * (w_orientation * z_orientation + x_orientation * y_orientation), 1 - 2 * (y_orientation*y_orientation + z_orientation*z_orientation));
  theta_imu = yaw + M_PI_2;

  if (theta_imu > M_PI){
    theta_imu -= 2*M_PI;
  }

  ang_vel_imu = imuSub->angular_velocity.z;

  accel_x_imu = imuSub->linear_acceleration.x;
  accel_y_imu = imuSub->linear_acceleration.y;
}

void Controlador::satSubCB(const sensor_msgs::msg::NavSatFix::SharedPtr satSub){
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
  static bool is_origin_set_ = false;

  try {
    // Converte Lat/Lon atuais para UTM usando GeographicLib
    GeographicLib::UTMUPS::Forward(satSub->latitude, satSub->longitude, zone, northp, utm_x, utm_y);
            
    // Define a origem local com a primeira coordenada válida recebida
    if (!is_origin_set_) {
      origin_x_ = utm_x;
      origin_y_ = utm_y;
      is_origin_set_ = true;
    }

    // Calcula a posição cartesiana local relativa à origem (0,0)
    x_gps = -(utm_x - origin_x_);
    y_gps = utm_y - origin_y_;

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
