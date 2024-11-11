#ifndef ACTUATORS_BOARD_HPP__
#define ACTUATORS_BOARD_HPP__

#include "rclcpp/rclcpp.hpp"

class ActuatorsBoardNode : public rclcpp::Node
{
    public :
        ActuatorsBoardNode();
        ~ActuatorsBoardNode();
};

#endif // ACTUATORS_BOARD_HPP__