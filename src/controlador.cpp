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

double lowPassFilter(double dt, double in, double oldIn, double oldOut){
  double p = 1.0 * 2.0 * M_PI;  // Polo / Frequência de corte (50 Hz)

  // Coeficientes do Filtro Passa-Baixas Discreto (Tustin)
  double a0 = (p * dt - 2.0) / (2.0 + p * dt);
  double b0 = (p * dt) / (2.0 + p * dt);
  double b1 = (p * dt) / (2.0 + p * dt); // No passa-baixas, b1 tem o mesmo sinal de b0

  // Equação de diferenças 
  double out = -a0 * oldOut + b0 * in + b1 * oldIn;

  return out;
}

class Controlador: public rclcpp::Node{
  
  public:
    Controlador(void);

    void posePublisher(void);
  
  private:
    // double dt;
    int rate;
    
    int num_estados = 4;
    int num_sensores = 4;

    // Vetor dos sensores [x_odom, y_odom, theta_odom, vel_odom, theta_imu, ax_imu, ay_imu, x_gps, y_gps]
    Eigen::VectorXd Y = Eigen::VectorXd::Zero(num_sensores);

    // Vetor de estados: [X, Y, theta, vel_lin]
    Eigen::VectorXd state = Eigen::VectorXd::Zero(num_estados);
    double theta;

    // Entrada do sistema: [w a]
    Eigen::VectorXd u = Eigen::VectorXd::Zero(2);
        
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angVelPub_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr satSub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub);
    void imuSubCB(const sensor_msgs::msg::Imu::SharedPtr imuSub);
    void satSubCB(const sensor_msgs::msg::NavSatFix::SharedPtr satSub);
    void velSubCB(const geometry_msgs::msg::Twist velSub);

    void predictKalmanState();
};

Controlador::Controlador(void): Node ("controlador"){
  rate = 100;
  // dt = 1/rate;

  this->declare_parameter<std::vector<double>>("process_noise_diagonal", {1e-8, 0.01, 0.01});
  this->declare_parameter<std::vector<double>>("measurement_noise_diagonal", {0.001, 1e-7, 0.0024, 1.5e-5, 1.5e-5});

  using std::placeholders::_1;

  odomSub_=create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&Controlador::odomSubCB,this,_1));
  imuSub_ =create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&Controlador::imuSubCB,this,_1));
  satSub_ =create_subscription<sensor_msgs::msg::NavSatFix>("gnss/fix", 1, std::bind(&Controlador::satSubCB,this,_1));
  velSub_=create_subscription<geometry_msgs::msg::Twist>("twist_mrac_linearizing_controller/command", 1, std::bind(&Controlador::velSubCB,this,_1));

  posePub_=create_publisher<nav_msgs::msg::Odometry>("/pose_ekf", 100);
  angVelPub_=create_publisher<geometry_msgs::msg::Twist>("/vel_debug",100);


  timer_=rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0/rate),std::bind(&Controlador::posePublisher,this));

}

void Controlador::posePublisher(void)
{   
    nav_msgs::msg::Odometry msg;

    Controlador::predictKalmanState();

    // RCLCPP_INFO(this->get_logger(), "Dados coletados");
    // RCLCPP_INFO(this->get_logger(), "x    y   ang  ");
    // RCLCPP_INFO(this->get_logger(), "%.4f  %.4f  %.4f\n", state(0) , state(1), state(2));

    Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q(rotation_vector);

    // 2. Garante que o quaternion está normalizado (evita erros numéricos)
    q.normalize();

    msg.pose.pose.position.x = state(2);
    msg.pose.pose.position.y = state(3);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    posePub_->publish(msg);


    // RCLCPP_INFO(this->get_logger(), "x_odom | x_gps  -  y_odom | y_gps  -  ang_odom | ang_IMU ");
    // RCLCPP_INFO(this->get_logger(), "%.4f  %.4f   %.4f  %.4f    %.4f   %.4f\n", x_odom , x_gps, y_odom , y_gps, theta_odom, theta_imu);

    // RCLCPP_INFO(this->get_logger(), "z_ori_odom | z_ori_IMU  -  w_ori_odom | w_ori_IMU");
    // RCLCPP_INFO(this->get_logger(), "%.4f  %.4f            %.4f  %.4f \n", (z_orientation_odom) , (z_orientation_imu), (w_orientation_odom) , (w_orientation_imu));
}

void Controlador::predictKalmanState(){
  static rclcpp::Time oldTime = this->get_clock()->now();

  rclcpp::Duration duration = this->get_clock()->now() - oldTime;
  double dt = duration.seconds();
  printf("duration: %f\n", dt);
  oldTime = this->get_clock()->now();

  static auto q_diag = this->get_parameter("process_noise_diagonal").as_double_array();
  static auto r_diag = this->get_parameter("measurement_noise_diagonal").as_double_array();

  // printf("controlador \n");
  static Eigen::VectorXd state_1 = Eigen::VectorXd::Zero(num_estados);

  static Eigen::MatrixXd K = Eigen::MatrixXd::Zero(num_estados, num_sensores);
  static Eigen::MatrixXd P = Eigen::MatrixXd::Identity(num_estados, num_estados)*1000;
  static Eigen::MatrixXd P_1 = Eigen::MatrixXd::Identity(num_estados, num_estados)*1000;

  static Eigen::VectorXd f = Eigen::VectorXd::Zero(num_estados);
  static Eigen::MatrixXd F = Eigen::MatrixXd::Identity(num_estados, num_estados);

  static Eigen::VectorXd h = Eigen::VectorXd::Zero(num_sensores);
  static Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_sensores, num_estados);

  // Configuração de ruído do processo (Ajustar baseado no robô)
  static Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_estados, num_estados);
  static Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_sensores, num_sensores);

  double vel_x   = state(0); //u(1);
  double vel_y   = state(1); //u(1);
  double x       = state(2);
  double y       = state(3);


  f(0) = vel_x;
  f(1) = vel_y;
  f(2) = x + vel_x * dt;
  f(3) = y + vel_y * dt;

  F(2,0) = dt;
  F(3,1) = dt;

  h(0) = vel_x;
  h(1) = vel_y;
  h(2) = x;
  h(3) = y;

  H(0,0) = 1;
  H(1,1) = 1;
  H(2,2) = 1;
  H(3,3) = 1;

  Q(0,0) = q_diag[0];
  Q(1,1) = q_diag[1];
  Q(2,2) = q_diag[2];
  Q(3,3) = q_diag[3];

  R(0,0) = r_diag[0];
  R(1,1) = r_diag[1];
  R(2,2) = r_diag[2];
  R(3,3) = r_diag[3];

  printf("inicialização\n");
  std::cout << "Matriz f:\n" << f << std::endl;
  std::cout << "Matriz F:\n" << F << std::endl;
  std::cout << "Matriz h:\n" << h << std::endl;
  std::cout << "Matriz H:\n" << H << std::endl;
  std::cout << "Matriz Q:\n" << Q << std::endl;
  std::cout << "Matriz R:\n" << R << std::endl;
  std::cout << "Matriz Y:\n" << Y << std::endl;

  state_1 = f;
  P = F * P * F.transpose() + Q;

  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  state = state_1 + K * (Y - h);
  P = (Eigen::MatrixXd::Identity(num_estados, num_estados) - K * H) * P;
}

void Controlador::velSubCB(const geometry_msgs::msg::Twist velSub){  
  u[0] = velSub.angular.z;
  u[1] = velSub.linear.x;
}

void Controlador::odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub)
//[x_odom, y_odom, theta_odom, vel_odom, theta_imu, ax_imu, ay_imu, x_gps, y_gps]
{ 
    static rclcpp::Time oldTime = this->get_clock()->now();
    static double vel_lin_odom = 0;
    static double old_measured_vel = 0;

    rclcpp::Duration duration = this->get_clock()->now() - oldTime;
    double dt = duration.seconds();
    oldTime = this->get_clock()->now();

    double vel_x_odom = odomSub->twist.twist.linear.x; 
    double vel_y_odom = odomSub->twist.twist.linear.y; 
    double measured_vel = std::sqrt(vel_x_odom*vel_x_odom + vel_y_odom*vel_y_odom);

    vel_lin_odom = lowPassFilter(dt, measured_vel, old_measured_vel, vel_lin_odom);// -a0 * vel_lin_odom + b0 * measured_vel + b1 * old_measured_vel;
    
    old_measured_vel = measured_vel;

    Y(0) = vel_x_odom;
    Y(1) = vel_y_odom;

    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Vector3 lin;
    geometry_msgs::msg::Vector3 ang;

    lin.x = vel_lin_odom;
    // lin.y = measured_vel;
    // ang.z = w_odom;
    msg.angular = ang;
    msg.linear = lin;
    // angVelPub_->publish(msg);

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
  //[x_odom, y_odom, theta_odom, vel_odom, theta_imu, ax_imu, ay_imu, x_gps, y_gps]
  double z_orientation =  imuSub->orientation.z;
  double w_orientation =  imuSub->orientation.w;
  double y_orientation =  imuSub->orientation.y;
  double x_orientation =  imuSub->orientation.x;

  double yaw = atan2(2 * (w_orientation * z_orientation + x_orientation * y_orientation), 1 - 2 * (y_orientation*y_orientation + z_orientation*z_orientation));
  double theta_imu = yaw + M_PI_2;

  if (theta_imu > M_PI){
    theta_imu -= 2*M_PI;
  }else if (theta_imu < -M_PI){
    theta_imu += 2*M_PI;
  }

  theta = theta_imu;
}

void Controlador::satSubCB(const sensor_msgs::msg::NavSatFix::SharedPtr satSub){
  static rclcpp::Time oldTime = this->get_clock()->now();
  static double vel_lin_odom = 0;
  static double old_measured_vel = 0;

  rclcpp::Duration duration = this->get_clock()->now() - oldTime;
  double dt = duration.seconds();
  oldTime = this->get_clock()->now();

  //[x_odom, y_odom, theta_odom, vel_odom, theta_imu, ax_imu, ay_imu, x_gps, y_gps]
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
    double x_gps = -(utm_x - origin_x_);
    double y_gps = -(utm_y - origin_y_);

    Y(2) = x_gps;
    Y(3) = y_gps;

    geometry_msgs::msg::Twist msg;
    geometry_msgs::msg::Vector3 lin;
    geometry_msgs::msg::Vector3 ang;

    lin.x = x_gps;
    lin.y = y_gps;
    // lin.y = measured_vel;
    // ang.z = w_odom;
    msg.angular = ang;
    msg.linear = lin;
    angVelPub_->publish(msg);

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
