#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>

class Controlador: public rclcpp::Node
{
  public:
      Controlador(void);
      void angVelPublisher(void);

  private:
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double xRef = 0.0;
        double yRef = 0.0;
        double thetaRef = 0.0;
        double vel_x = 0.0;
        double vel_y = 0.0;
        double vel_lin = 0.0;
        double alpha = 0.0;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angVelPub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPose_;

        rclcpp::TimerBase::SharedPtr timer_;

        void odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub);

        void goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalPose);

        void torqueCTRL();
      
};

Controlador::Controlador(void): Node ("controlaor")
{   
    printf("inicio construtor");
    using std::placeholders::_1;

    odomSub_=create_subscription<nav_msgs::msg::Odometry>("odom", 1, std::bind(&Controlador::odomSubCB,this,_1));
    goalPose_=create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&Controlador::goalPoseCB,this,_1));

    angVelPub_=create_publisher<geometry_msgs::msg::Twist>("/twist_mrac_linearizing_controller/command",100);
    printf("Antes int");
    int rate = 100;
    timer_=rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0/rate),std::bind(&Controlador::angVelPublisher,this));

    printf("fim construtor");
}

void Controlador::angVelPublisher(void)
{   
    // RCLCPP_INFO(this->get_logger(), "\n ang vel publicada\n");

    static double xAlvo = 0.0;    
    static double yAlvo = 0.0;
    static double thetaAlvo = 0.0; 
    
    static double xAntigo = 0.0;    
    static double yAntigo = 0.0;
    static double thetaAntigo = 0.0;  
    
    static double integralX = 0.0;
    static double integralY = 0.0;

    // 1. Calcula o erro atual (Ação Proporcional)
    double erroX = xRef - x;
    double erroY = yRef - y;

    // 2. Acumula o erro no tempo (Ação Integral Discreta a 100Hz -> Ts = 0.01)
    // ATENÇÃO: 'integralX' e 'integralY' devem ser variáveis globais ou membros da classe!
    integralX += erroX * 0.01;
    integralY += erroY * 0.01;

    // Antirewind / Anti-windup (Opcional, mas altamente recomendado):
    // Impede que a integral cresça infinitamente se o robô colidir ou travar
    integralX = std::max(std::min(integralX, 1.0), -1.0);
    integralY = std::max(std::min(integralY, 1.0), -1.0);

    // 3. Define os Ganhos do Controlador (Ajuste esses valores no seu teste!)
    double Kp = 1.0; 
    double Ki = 0.05;

    // 4. Monta o novo vetor v (y_dot) com a estrutura PI
    double y_dot[2] = {
        Kp * erroX + Ki * integralX,
        Kp * erroY + Ki * integralY
    };

    // Atualização de histórico (Seu código original)
    xAntigo = xAlvo;
    yAntigo = yAlvo;
    thetaAntigo = thetaAlvo;

    // 5. Matriz E^-1(x) com ponto deslocado b = 0.1m
    double b = 0.1; 
    double Einv[2][2] = {
        { cos(theta),           sin(theta)          },
        {-sin(theta) / b,       cos(theta) / b      }
    };

    // 6. Multiplicação Matriz x Vetor PI
    double u[2] = {
        Einv[0][0] * y_dot[0] + Einv[0][1] * y_dot[1],
        Einv[1][0] * y_dot[0] + Einv[1][1] * y_dot[1]
    };

    // 7. Cinemática Inversa das Rodas (L = 0.322, R = 0.075)
    double w1 = (u[0] + (u[1] * 0.322 / 2.0)) / 0.075; // Roda Direita
    double w2 = (u[0] - (u[1] * 0.322 / 2.0)) / 0.075; // Roda Esquerda

    geometry_msgs::msg::Twist msg;

    geometry_msgs::msg::Vector3 lin;
    geometry_msgs::msg::Vector3 ang;
  
    lin.x = u[0];
    ang.z = u[1];

    msg.linear = lin;
    msg.angular = ang;

    angVelPub_->publish(msg);
}


void Controlador::odomSubCB(const nav_msgs::msg::Odometry::SharedPtr odomSub)
{
    x = odomSub->pose.pose.position.x;
    y = odomSub->pose.pose.position.y;

    double z_orientation = odomSub->pose.pose.orientation.z;
    double w_orientation = odomSub->pose.pose.orientation.w;
    theta = 2*atan2(z_orientation,w_orientation);

    vel_x = odomSub->twist.twist.linear.x; 
    vel_y = odomSub->twist.twist.linear.x; 

    vel_lin = std::sqrt(vel_x*vel_x + vel_y*vel_y);

    // RCLCPP_INFO(this->get_logger(), "\nOdometria recebida\n");
    // RCLCPP_INFO(this->get_logger(), "--- Posição Atual do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "X:     %.4f metros", x);
    // RCLCPP_INFO(this->get_logger(), "Y:     %.4f metros", y);
    // RCLCPP_INFO(this->get_logger(), "Theta: %.4f radianos (%.2f°)", theta, theta * (180.0 / M_PI));

    // RCLCPP_INFO(this->get_logger(), "--- Posição Ref do Robô (Odom) ---");
    // RCLCPP_INFO(this->get_logger(), "X:     %.4f metros", xRef);
    // RCLCPP_INFO(this->get_logger(), "Y:     %.4f metros", yRef);
    // RCLCPP_INFO(this->get_logger(), "Theta: %.4f radianos (%.2f°)", thetaRef, thetaRef * (180.0 / M_PI));

}

void Controlador::goalPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr goalPose)
{   
    xRef = goalPose->pose.position.x;
    yRef = goalPose->pose.position.y;

    double z_orientation = goalPose->pose.orientation.z;
    double w_orientation = goalPose->pose.orientation.w;
    
    double dRef = std::sqrt(xRef*xRef + yRef*yRef);
    double d = std::sqrt(x*x + y*y);

    thetaRef = 2*atan2(z_orientation,w_orientation);

    double ts = (0.1 + (dRef - d) * 17) / 4;
    alpha = std::exp(-0.01 / ts);
}

void Controlador::torqueCTRL()
{

}

int main(int argc, char ** argv)
{
  printf("hello world T1 package hehehe\n");
  rclcpp::init(argc,argv);
  printf("aqui");
  rclcpp::spin(std::make_shared<Controlador>());
  rclcpp::shutdown();
  return 0;
}
