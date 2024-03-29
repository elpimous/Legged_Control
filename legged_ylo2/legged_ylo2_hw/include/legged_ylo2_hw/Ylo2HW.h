
//
// Created by qiayuan on 1/24/22.
// Modified elpimous12 for ylo2 robot
//

#pragma once // directive de préprocesseur -> inclure le fichier d'en-tête une seule fois lors de la compilation
#include <legged_hw/LeggedHW.h>
#include <sensor_msgs/Imu.h>
#include "moteus_driver/YloTwoPcanToMoteus.hpp" // ylo2-robot library

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct Ylo2MotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct Ylo2ImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class Ylo2HW : public LeggedHW {
 public:

  Ylo2HW() = default;

  /** \brief Get necessary params from param server. Init hardware_interface.
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed. */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;


  /** \brief Communicate with hardware. Get data, status of robot.
   * Call Recv() to get robot's state.
   * @param time Current time
   * @param period Current time - last time */
  void read(const ros::Time& time, const ros::Duration& period) override;


  /** \brief Comunicate with hardware. Publish command to robot.
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call Recv(). Publish actuator
   * current state.
   * @param time Current time
   * @param period Current time - last time */
  void write(const ros::Time& time, const ros::Duration& period) override;


  // params for Mjbots QDD100 motors

  // receive variables
  float RX_pos =   0;
  float RX_vel =   0;
  float RX_tor =   0;
  float RX_volt =  0;
  float RX_temp =  0;
  float RX_fault = 0;

  // send orders variables
  float joint_position;
  float joint_velocity;
  float joint_fftorque;
  float joint_kp;
  float joint_kd;
    

  Ylo2ImuData imuData_{}; // Imu datas structure

  /** @brief The imu callback */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_message);
  
 private:
  
  /** @brief Executes the robot's startup routine */
  bool startup_routine();

  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  Ylo2MotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};

  YloTwoPcanToMoteus command_; // instance of class YloTwoPcanToMoteus, Ylo2 robot lib
};

}  // namespace legged
