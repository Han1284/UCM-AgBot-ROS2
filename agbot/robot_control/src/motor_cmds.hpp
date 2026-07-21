#include <array>
#include <memory>
#include <boost/asio.hpp>
#include <iostream> 

#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP

class RobotCommands
{
public:
  RobotCommands()
    : socket_(io_context_),
      server_("192.168.1.7"),
      port_(20001),
      lift_speed_(0),
      lift_ctrl_(0),
      left_motor_speed_(0),
      right_motor_speed_(0),
      motor_rotation_ctrl_(0),
      data_({0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 
             0x00, 0x00, 
             0x00, 0x38})

  // {
  //     data_[6] = lift_speed_;
  //     data_[7] = lift_ctrl_;
  //     data_[9] = left_motor_speed_;
  //     data_[10] = right_motor_speed_;
  //     data_[11] = motor_rotation_ctrl_;
  // }

  {}

  bool is_connected()
  {
      return socket_.is_open();
  }

  void connection()
  {
    while (!is_connected()) {
      try {
        socket_.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(server_), port_));
        std::cout << "Connection established\n";
      } catch (const boost::system::system_error& e) {
        std::cout << "Failed to connect: " << e.what() << ". Retrying...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));  
      }
    }
  }

  void set_lift_speed(int speed)
  {
    data_[6] = speed;
  }

  void set_lift_ctrl(int ctrl)
  {
    data_[7] = ctrl;
  }

  void set_left_speed(int speed)
  {
    data_[9] = speed;
  }

  void set_right_speed(int speed)
  {
    data_[10] = speed;
  }

  void set_direction_ctrl(int state)
  {
    data_[11] = state;
  }

  void set_neutral()
  {
    data_={0x08, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 
      0x00, 0x00, 
      0x00, 0x38};
  }

  int rotation_check()
  {
    return data_[11];
  }

  void sending_data()
  {
    while (!is_connected()) 
    {
      std::cout << "Connection lost. Attempting to reconnect...\n";
      connection();  
    }
    boost::asio::write(socket_, boost::asio::buffer(data_));
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));

    std::stringstream ss;
    ss << "Sending data: ";
    for (const auto& value : data_) {
        ss << static_cast<int>(value) << ' ';
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotCommands"), "%s", ss.str().c_str());
  }

private:
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;
  std::string server_;
  int port_;
  int lift_speed_;
  int lift_ctrl_;
  int left_motor_speed_;
  int right_motor_speed_;
  int motor_rotation_ctrl_;
  std::array<unsigned char, 13> data_;
};

#endif
