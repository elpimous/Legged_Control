
//
// Created by qiayuan on 1/24/22.
// Adapted for Ylo2 robot - vincent foucault - 2023-03

#include "legged_ylo2_hw/Ylo2HW.h"
#include "sensor_msgs/Imu.h"


namespace legged {

void Ylo2HW::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_message){

  imuData_.ori_[0]        = imu_message->orientation.x;
  imuData_.ori_[1]        = imu_message->orientation.y;
  imuData_.ori_[2]        = imu_message->orientation.z;
  imuData_.ori_[3]        = imu_message->orientation.w;

  imuData_.angularVel_[0] = imu_message->angular_velocity.x;
  imuData_.angularVel_[1] = imu_message->angular_velocity.y;
  imuData_.angularVel_[2] = imu_message->angular_velocity.z;

  imuData_.linearAcc_[0]  = imu_message->linear_acceleration.x;
  imuData_.linearAcc_[1]  = imu_message->linear_acceleration.y;
  imuData_.linearAcc_[2]  = imu_message->linear_acceleration.z;

  imuData_.oriCov_[0] = imu_message->orientation_covariance[0];
  imuData_.oriCov_[4] = imu_message->orientation_covariance[4];
  imuData_.oriCov_[8] = imu_message->orientation_covariance[8];

  imuData_.angularVelCov_[0] = imu_message->linear_acceleration_covariance[0];
  imuData_.angularVelCov_[4] = imu_message->linear_acceleration_covariance[4];
  imuData_.angularVelCov_[8] = imu_message->linear_acceleration_covariance[8];
}


bool Ylo2HW::startup_routine()
{
  /* initialize GPIO pin, for security switch button */
  command_.btnPin = mraa_gpio_init(BTN_PIN);
  mraa_gpio_dir(command_.btnPin, MRAA_GPIO_IN);

  command_.peak_fdcan_board_initialization();
  usleep(100);

  command_.check_initial_ground_pose();
  std::cout << "startup_routine Done." << std::endl;
  usleep(3000000);
  return true;
}


bool Ylo2HW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  robot_hw_nh.getParam("power_limit", powerLimit_); // 4

  while (!Ylo2HW::startup_routine()){}; // run startup routine until success

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  std::string robot_type;
  robot_type = "a1";
  /*
  root_nh.getParam("robot_type", robot_type);
  if (robot_type == "a1") {
    std::cout << "robot is : " << robot_type << std::endl;
  } else if (robot_type == "aliengo") {
    std::cout << "robot is : " << robot_type << std::endl;
  } else {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }
  */
  return true;
}

void Ylo2HW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  // read the 12 joints, and store values into legged controller
  for (int i = 0; i < 12; ++i) {

    // Reset values
    RX_pos = 0.0;
    RX_vel = 0.0;
    RX_tor = 0.0;
    RX_volt = 0.0;
    RX_temp = 0.0;
    RX_fault = 0.0;

    auto ids  = command_.motor_adapters_[i].getIdx(); // moteus controller id
    int port  = command_.motor_adapters_[i].getPort(); // select correct port on Peak canfd board
    auto sign = command_.motor_adapters_[i].getSign(); // in case of joint reverse rotation

    // call ylo2 moteus lib
    command_.read_moteus_RX_queue(ids, port, 
                                  RX_pos, RX_vel, RX_tor, 
                                  RX_volt, RX_temp, RX_fault);      // query values;

    jointData_[i].pos_ = static_cast<double>(sign*(RX_pos*2*M_PI)); // conversion turns -> radians
    jointData_[i].vel_ = static_cast<double>(RX_vel*2*M_PI);
    jointData_[i].tau_ = static_cast<double>(RX_tor);               // measured in N*m


    // The imu variable is actualized into callback !

    // TODO read volt, temp, faults for Diagnostics

    usleep(150); // needed
  }

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
}

void Ylo2HW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  // ask security switch status
  // if pressed, it directly set maxtorque to 0
  command_.security_switch();

  //ROS_INFO("write function");
  for (int i = 0; i < 12; ++i) {

    auto ids  = command_.motor_adapters_[i].getIdx(); // moteus controller id
    int port  = command_.motor_adapters_[i].getPort(); // select correct port on Peak canfd board
    auto sign = command_.motor_adapters_[i].getSign(); // in case of joint reverse rotation
    
    joint_position = static_cast<float>(sign*(jointData_[i].posDes_/(2*M_PI))); // conversion radians -> turns
    joint_velocity = static_cast<float>(jointData_[i].velDes_/(2*M_PI));
    joint_fftorque = static_cast<float>(jointData_[i].ff_);
    joint_kp       = static_cast<float>(jointData_[i].kp_);
    joint_kd       = static_cast<float>(jointData_[i].kd_);
    
    // call ylo2 moteus lib
    command_.send_moteus_TX_frame(ids, port, joint_position, joint_velocity, joint_fftorque, joint_kp, joint_kd); 
    usleep(150); // needed

  }
}

bool Ylo2HW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0; // TODO CHECK LEGS INDEX
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = 0;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = 1;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = 2;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = 3;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0; // ABAD
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1; // UPPER LEG
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2; // LOWER LEG
    } else {
      continue;
    }

    // FOR LEGGED HARDWARE
    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  ROS_INFO("setupJoints() OK.");
  return true;
}


bool Ylo2HW::setupImu() {
   imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("ylo2_imu", "ylo2_imu", imuData_.ori_, imuData_.oriCov_,
                                                                          imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                          imuData_.linearAccCov_));

   ROS_INFO("setupImu() OK.");
   return true;
}


bool Ylo2HW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  ROS_INFO("setupContactSensor() OK.");
  return true;
}

}  // namespace legged
