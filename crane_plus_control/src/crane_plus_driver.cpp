
#include "crane_plus_control/crane_plus_driver.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("crane_plus_driver");

CranePlusDriver::CranePlusDriver()
{
}

CranePlusDriver::~CranePlusDriver()
{
  dxl_port_handler_->closePort();
}

hardware_interface::return_type CranePlusDriver::init()
{
  dxl_port_handler_ = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0");
  dxl_packet_handler_ = dynamixel::PacketHandler::getPacketHandler(1.0);

  if(!dxl_port_handler_->openPort()){
    throw std::runtime_error("unable to open dynamixel port");
  }

  if(!dxl_port_handler_->setBaudRate(1000000)){
    throw std::runtime_error("unable to set baudrate");
  }

  joint_names_.push_back("crane_plus_joint1");
  joint_names_.push_back("crane_plus_joint2");
  joint_names_.push_back("crane_plus_joint3");
  joint_names_.push_back("crane_plus_joint4");
  joint_names_.push_back("crane_plus_joint_hand");

  // Resize members
  pos_.resize(joint_names_.size());
  vel_.resize(joint_names_.size());
  eff_.resize(joint_names_.size());
  cmd_.resize(joint_names_.size());
  op_mode_.resize(joint_names_.size());
  joint_state_handles_.resize(joint_names_.size());
  joint_command_handles_.resize(joint_names_.size());
  joint_mode_handles_.resize(joint_names_.size());

  size_t i = 0;
  for (auto & joint_name : joint_names_) {

    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    joint_state_handles_[i] = state_handle;

    if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::return_type::OK) {
      throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
    }

    hardware_interface::JointCommandHandle command_handle(joint_name, &cmd_[i]);
    joint_command_handles_[i] = command_handle;
    if (register_joint_command_handle(&joint_command_handles_[i]) !=
      hardware_interface::return_type::OK)
    {
      throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
    }

    joint_mode_handles_[i] = hardware_interface::OperationModeHandle(joint_name, &op_mode_[i]);
    if (register_operation_mode_handle(&joint_mode_handles_[i]) != hardware_interface::return_type::OK)
    {
      throw std::runtime_error("unable to register " + joint_mode_handles_[i].get_name());
    }
    ++i;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusDriver::read()
{
  const int ADDR_MX_PRESENT_POSITION = 36;
  const double TO_RADIANS = (300.0 / 1023.0) * M_PI / 180.0;
  static int count=0;

  for(size_t i=0; i < joint_names_.size(); ++i){
    int dxl_id = i + 1;  // サーボのIDは1 ~ 5なので
    uint16_t dxl_present_position = 0;
    uint8_t dxl_error = 0;     
    int dxl_comm_result = dxl_packet_handler_->read2ByteTxRx(dxl_port_handler_, 
      dxl_id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "TxRxResult: %d", dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      RCLCPP_ERROR(LOGGER, "RxPAcketError: %d", dxl_error);
    }

    // RCLCPP_INFO(LOGGER, "[ID:%03d] PresPos:%f", dxl_id, dxl_present_position * TO_RADIANS);

    pos_[i] = (dxl_present_position - 511) * TO_RADIANS;
  }

  count++;

  if(count > 200){
    count = 0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusDriver::write()
{
  return hardware_interface::return_type::OK;
}