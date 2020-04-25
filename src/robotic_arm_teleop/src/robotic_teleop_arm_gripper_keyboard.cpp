/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, myyerrol
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: myyerrol

#include <robotic_arm_teleop/robotic_teleop_arm_gripper_keyboard.h>

int            g_kfd = 0;
struct termios g_cooked;
struct termios g_raw;

TeleopArmGripperKeyboard::TeleopArmGripperKeyboard()
{
    arm_name_.push_back("Rev1");
    arm_name_.push_back("Rev2");
    arm_name_.push_back("Rev3");
    arm_name_.push_back("Rev4");
    arm_name_.push_back("Rev5");
    arm_name_.push_back("Rev6");
    arm_name_.push_back("Rev7");

    arm_goal_.trajectory.joint_names.push_back("Rev1");
    arm_goal_.trajectory.joint_names.push_back("Rev2");
    arm_goal_.trajectory.joint_names.push_back("Rev3");
    arm_goal_.trajectory.joint_names.push_back("Rev4");
    arm_goal_.trajectory.joint_names.push_back("Rev5");
    arm_goal_.trajectory.joint_names.push_back("Rev6");
    arm_goal_.trajectory.joint_names.push_back("Rev7");

    arm_goal_.trajectory.points.resize(1);
    arm_goal_.trajectory.points[0].positions.resize(arm_name_.size());
    arm_goal_.trajectory.points[0].velocities.resize(arm_name_.size());
    arm_goal_.trajectory.points[0].accelerations.resize(arm_name_.size());

    for (int i = 0; i < arm_name_.size(); i++) {
        arm_index_[arm_name_[i]] = i;
        arm_goal_.trajectory.points[0].positions[i]     = 0.0;
        arm_goal_.trajectory.points[0].velocities[i]    = 0.0;
        arm_goal_.trajectory.points[0].accelerations[i] = 0.0;
    }

    gripper_goal_.trajectory.joint_names.push_back("Rev8");
    // gripper_goal_.trajectory.joint_names.push_back("joint_finger_r");

    gripper_goal_.trajectory.points.resize(1);
    gripper_goal_.trajectory.points[0].positions.resize(2);
    gripper_goal_.trajectory.points[0].velocities.resize(2);
    gripper_goal_.trajectory.points[0].accelerations.resize(2);

    for (int i = 0; i < 2; i++) {
        gripper_goal_.trajectory.points[0].positions[i]     = 0.0;
        gripper_goal_.trajectory.points[0].velocities[i]    = 0.0;
        gripper_goal_.trajectory.points[0].accelerations[i] = 0.0;
    }

    // gripper_goal_.command.position   = 0.0;
    // gripper_goal_.command.max_effort = 0.0;

    ros::NodeHandle n_private("~");
    n_private.param("arm_position_step", arm_position_step_, 0.0174);
    n_private.param("gripper_position_step", gripper_position_step_, 0.001);

    arm_trajectory_client_ =  boost::make_shared<ArmTrajectoryClient>(
        "robotic_arm_controller/follow_joint_trajectory", true);

    gripper_trajectory_client_ = boost::make_shared<gripperTrajectoryClient>(
        "robotic_gripper_controller/follow_joint_trajectory", true);

    // gripper_command_client_ = boost::make_shared<gripperCommandClient>(
    //     "gripper_controller/gripper_cmd", true);

    while (!arm_trajectory_client_->waitForServer(ros::Duration(5)) ) {
        ROS_INFO_STREAM(
            "Waiting for the arm joint_trajectory_action server...");
    }

    while (!gripper_trajectory_client_->waitForServer(ros::Duration(5)) ) {
        ROS_INFO_STREAM(
            "Waiting for the gripper joint_trajectory_action server...");
    }

    // while (!gripper_command_client_->waitForServer(ros::Duration(5))) {
    //     ROS_INFO_STREAM("Waiting for the grippper_command_action server...");
    // }
}

TeleopArmGripperKeyboard::~TeleopArmGripperKeyboard()
{
    robotic_arm_nh_.shutdown();
}

void TeleopArmGripperKeyboard::spinTeleopArmGripper()
{
    char   keyboard_cmd;
    double arm_position[6];
    double gripper_position = 0;
    bool   flag = false;

    memset(arm_position, 0, sizeof(arm_position));

    tcgetattr(g_kfd, &g_cooked);
    memcpy(&g_raw, &g_cooked, sizeof(struct termios));
    g_raw.c_lflag &=~ (ICANON | ECHO);
    g_raw.c_cc[VEOL] = 1;
    g_raw.c_cc[VEOF] = 2;
    tcsetattr(g_kfd, TCSANOW, &g_raw);

    puts("-----------------------------------------");
    puts("    Teleop Trajectory Arm By Keyboard    ");
    puts("                Version 1                ");
    puts("-----------------------------------------");
    puts("Q                   W                   E");
    puts("A                   S                   D");
    puts("Z                   X                    ");
    puts("                                         ");
    puts("-----------------------------------------");
    puts("Q:Joint1-UP  W:Joint2-UP  E:Joint3-UP    ");
    puts("A:Joint4-UP  S:Joint5-UP  D:Joint6-UP    ");
    puts("Z:Joint7-UP  X:gripper-Open                 ");
    puts("-----------------------------------------");
    puts("Shift+Q:Joint1-DOWN  Shift+W:Joint2-DOWN ");
    puts("Shift+E:Joint3-DOWN  Shift+A:Joint4-DOWN ");
    puts("Shift+S:Joint5-DOWN  Shift+D:Joint6-DOWN ");
    puts("Shift+Z:Joint7-DOWN                      ");
    puts("Shift+X:gripper-Close"                       );
    puts("-----------------------------------------");
    puts("PRESS CTRL-C TO QUIT                     ");

    struct pollfd ufd;
    ufd.fd = g_kfd;
    ufd.events = POLLIN;

    while (true) {
        boost::this_thread::interruption_point();
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("Function poll():");
            return;
        }
        else if (num > 0) {
            if(read(g_kfd, &keyboard_cmd, 1) < 0) {
                perror("Function read():");
                return;
            }
        }
        else {
            if(flag == true) {
                continue;
            }
        }

        switch(keyboard_cmd) {
            case KEYCODE_Q: {
                arm_position[arm_index_["Rev1"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev1"]] >= 2.90) {
                    arm_position[arm_index_["Rev1"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_W: {
                arm_position[arm_index_["Rev2"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev2"]] >= 1.76) {
                    arm_position[arm_index_["Rev2"]] = 1.76;
                }
                flag = true;
                break;
            }
            case KEYCODE_E: {
                arm_position[arm_index_["Rev3"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev3"]] >= 2.90) {
                    arm_position[arm_index_["Rev3"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_A: {
                arm_position[arm_index_["Rev4"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev4"]] >= 0.0) {
                    arm_position[arm_index_["Rev4"]] = 0.0;
                }
                break;
            }
            case KEYCODE_S: {
                arm_position[arm_index_["Rev5"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev5"]] >= 2.90) {
                    arm_position[arm_index_["Rev5"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_D: {
                arm_position[arm_index_["Rev6"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev6"]] >= 3.75) {
                    arm_position[arm_index_["Rev6"]] = 3.75;
                }
                flag = true;
                break;
            }
            case KEYCODE_Z: {
                arm_position[arm_index_["Rev7"]] += arm_position_step_;
                if (arm_position[arm_index_["Rev7"]] >= 2.90) {
                    arm_position[arm_index_["Rev7"]] = 2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_X: {
                gripper_position += gripper_position_step_;
                if (gripper_position >= 0.04) {
                    gripper_position = 0.04;
                }
                gripper_goal_.trajectory.points[0].positions[0] = gripper_position;
                gripper_goal_.trajectory.points[0].positions[1] = gripper_position;
                gripper_goal_.trajectory.points[0].time_from_start =
                    ros::Duration(5);
                gripper_goal_.goal_time_tolerance = ros::Duration(0);
                gripper_trajectory_client_->sendGoal(gripper_goal_);
                // gripper_goal_.command.position = gripper_position;
                // gripper_command_client_->sendGoal(gripper_goal_);
                flag = true;
                break;
            }
            case KEYCODE_Q_CAP: {
                arm_position[arm_index_["Rev1"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev1"]] <= -2.90) {
                    arm_position[arm_index_["Rev1"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_W_CAP: {
                arm_position[arm_index_["Rev2"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev2"]] <= -1.76) {
                    arm_position[arm_index_["Rev2"]] = -1.76;
                }
                flag = true;
                break;
            }
            case KEYCODE_E_CAP: {
                arm_position[arm_index_["Rev3"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev3"]] <= -2.90) {
                    arm_position[arm_index_["Rev3"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_A_CAP: {
                arm_position[arm_index_["Rev4"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev4"]] <= -3.07) {
                    arm_position[arm_index_["Rev4"]] = -3.07;
                }
                flag = true;
                break;
            }
            case KEYCODE_S_CAP: {
                arm_position[arm_index_["Rev5"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev5"]] <= -2.90) {
                    arm_position[arm_index_["Rev5"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_D_CAP: {
                arm_position[arm_index_["Rev6"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev6"]] <= 0.0) {
                    arm_position[arm_index_["Rev6"]] = 0.0;
                }
                break;
            }
            case KEYCODE_Z_CAP: {
                arm_position[arm_index_["Rev7"]] -= arm_position_step_;
                if (arm_position[arm_index_["Rev7"]] <= -2.90) {
                    arm_position[arm_index_["Rev7"]] = -2.90;
                }
                flag = true;
                break;
            }
            case KEYCODE_X_CAP: {
                gripper_position -= gripper_position_step_;
                if (gripper_position <= 0.0) {
                    gripper_position = 0.0;
                }
                gripper_goal_.trajectory.points[0].positions[0] = gripper_position;
                gripper_goal_.trajectory.points[0].positions[1] = gripper_position;
                gripper_goal_.trajectory.points[0].time_from_start =
                    ros::Duration(5);
                gripper_goal_.goal_time_tolerance = ros::Duration(0);
                gripper_trajectory_client_->sendGoal(gripper_goal_);
                // gripper_goal_.command.position = gripper_position;
                // gripper_command_client_->sendGoal(gripper_goal_);
                flag = true;
                break;
            }
            default: {
                flag = false;
            }
        }

        arm_goal_.trajectory.points[0].positions[arm_index_["Rev1"]] =
            arm_position[arm_index_["Rev1"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["Rev2"]] =
            arm_position[arm_index_["Rev2"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["Rev3"]] = arm_position[arm_index_["Rev3"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["Rev4"]] = arm_position[arm_index_["Rev4"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["Rev5"]] =
            arm_position[arm_index_["Rev5"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["Rev6"]] = arm_position[arm_index_["Rev6"]];
        arm_goal_.trajectory.points[0].positions[arm_index_["Rev7"]] = arm_position[arm_index_["Rev7"]];

        arm_goal_.trajectory.points[0].time_from_start = ros::Duration(5);
        arm_goal_.goal_time_tolerance = ros::Duration(0);
        arm_trajectory_client_->sendGoal(arm_goal_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotic_teleop_arm_gripper_keyboard",
              ros::init_options::NoSigintHandler);

    TeleopArmGripperKeyboard teleop_arm_gripper_keyboard;

    boost::thread make_thread = boost::thread(
        boost::bind(&TeleopArmGripperKeyboard::spinTeleopArmGripper,
                    &teleop_arm_gripper_keyboard));

    ros::spin();

    make_thread.interrupt();
    make_thread.join();
    tcsetattr(g_kfd, TCSANOW, &g_cooked);

    return 0;
}
