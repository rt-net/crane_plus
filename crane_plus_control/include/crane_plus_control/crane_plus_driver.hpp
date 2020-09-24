
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <memory>
#include <string>
#include <vector>

class CranePlusDriver
{
public:
  CranePlusDriver(const std::string port_name, const int baudrate, std::vector<uint8_t> id_list);
  ~CranePlusDriver();

  bool open_port(void);
  void close_port(void);
  std::string get_last_error_log(void);

  bool torque_enable(const bool enable);
  bool write_goal_joint_positions(const std::vector<double> & goal_positions);
  bool write_moving_speed_rpm(const uint8_t dxl_id, const double speed_rpm);
  bool write_moving_speed_rpm_all(const double speed_rpm);
  bool read_present_joint_positions(std::vector<double> * joint_positions);

private:
  std::shared_ptr<dynamixel::PortHandler> dxl_port_handler_;
  std::shared_ptr<dynamixel::PacketHandler> dxl_packet_handler_;
  int baudrate_;
  std::vector<uint8_t> id_list_;
  std::string last_error_log_;

  bool parse_dxl_error(const std::string func_name, const uint8_t dxl_id,
    const int dxl_comm_result, const uint8_t dxl_packet_error);
  double dxl_pos_to_radian(const uint16_t position);
  uint16_t radian_to_dxl_pos(const double position);

};
