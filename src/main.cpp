#include "solver.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Solver>(std::string("solver_node")));
    rclcpp::shutdown();
    return 0;
}