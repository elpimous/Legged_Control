//
// Created by qiayuan on 1/24/22.
// Adapted for Ylo2 robot - vincent foucault - 2023-02
//

#pragma once // directive de  préprocesseur : le fichier source actuel n'est inclus qu'une seule fois lors de la compilation

#include <legged_hw/LeggedHW.h>
#include "lib/moteus_driver/YloTwoPcanToMoteus.hpp" // ylo2 library

namespace legged {
  
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"}; 

// moteus controllers variables initialization
struct Ylo2MotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

// imu variables initialization
struct Ylo2ImuData {
  double ori_[4];
  double oriCov_[9];          //= (4.592449e-06, 0.0, 0.0, 
                              // 0.0, 4.592449e-06, 0.0, 
                              // 0.0, 0.0, 4.592449e-06);
  double angularVel_[3];      // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];   // = (5.895184e-06, 0.0, 0.0, 
                              // 0.0, 5.895184e-06, 0.0, 
                              // 0.0, 0.0, 5.895184e-06);
  double linearAcc_[3];       // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];    // = (0.0007199025610000001, 0.0, 0.0, 
                              // 0.0, 0.0007199025610000001, 0.0, 
                              // 0.0, 0.0, 0.0007199025610000001);
};


class Ylo2HW : public LeggedHW {

 public:
  Ylo2HW() = default;

  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed. */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;


  /** \brief Communicate with hardware. Get data, status of robot.
   * @param time Current time
   * @param period Current time - last time
   * @param read each pos, vel, torque, for all 12 joints controllers (moteus) 
   * read imu message values */
  void read(const ros::Time& time, const ros::Duration& period) override;


  /** \brief Comunicate with hardware. Publish command to robot.
   * @param time Current time
   * @param period Current time - last time
   * @param write pos, vel, torque, kp, kd, for all 12 joints controllers (moteus) */
  void write(const ros::Time& time, const ros::Duration& period) override;

  float RX_pos =   0;
  float RX_vel =   0;
  float RX_tor =   0;
  float RX_volt =  0;
  float RX_temp =  0;
  float RX_fault = 0;


 private:

  /** @brief Sends a zero command to the robot */
  bool send_zero_command();

  /** @brief Executes the robot's startup routine */
  bool startup_routine();



  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  Ylo2MotorData jointData_[12]{};
  Ylo2ImuData imuData_{};
  bool contactState_[4]{};

  //int contactThreshold_{};
};

}  // namespace legged
