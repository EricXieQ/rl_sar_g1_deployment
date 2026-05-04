/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef G1_FSM_HPP
#define G1_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <fstream>
#include <chrono>

namespace g1_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float percent_getup = 0.0f;

    void Enter() override
    {
        percent_getup = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
    }

    void Run() override
    {
        Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLRoboMimicLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRLRoboMimicLocomotion : public RLFSMState
{
public:
RLFSMStateRLRoboMimicLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLRoboMimicLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/locomotion";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num2 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateRLRoboMimicCharleston";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num3 || rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft)
        {
            return "RLFSMStateRLWholeBodyTrackingDance102";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num4 || rl.control.current_gamepad == Input::Gamepad::RB_DPadRight)
        {
            return "RLFSMStateRLWholeBodyTrackingGangnamStyle";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num5)
        {
            // Smooth path: pre-position the body in dab frame-0 pose first,
            // then auto-transition into the policy. Use this with the hoist.
            return "RLFSMStateASAPDabGetReady";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num6)
        {
            // Direct path: jump straight into the policy. Has a residual
            // first-action jolt but the policy is alive immediately, so the
            // robot can actively balance. Use for quick ground tests.
            return "RLFSMStateRLASAPDab";
        }
        return state_name_;
    }
};

class RLFSMStateRLRoboMimicCharleston : public RLFSMState
{
public:
    RLFSMStateRLRoboMimicCharleston(RL *rl) : RLFSMState(*rl, "RLFSMStateRLRoboMimicCharleston") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robomimic/charleston";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

        rl.motion_length = 18.0;
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        return state_name_;
    }
};

class RLFSMStateRLWholeBodyTrackingDance102 : public RLFSMState
{
public:
    RLFSMStateRLWholeBodyTrackingDance102(RL *rl) : RLFSMState(*rl, "RLFSMStateRLWholeBodyTrackingDance102") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "whole_body_tracking/dance_102";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);

            // Initialize motion loader
            std::string motion_file_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + rl.params.Get<std::string>("motion_file");
            float fps = 1.0f / (rl.params.Get<float>("dt") * rl.params.Get<int>("decimation"));
            rl.motion_loader = std::make_unique<MotionLoader>(motion_file_path, fps);
            rl.motion_length = rl.motion_loader->GetDuration();

            auto waist_sdk_indices = rl.params.Get<std::vector<int>>("waist_joint_indices");
            std::vector<float> waist_angles = {
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[0])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[1])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[2])]
            };
            rl.motion_loader->Reset(fsm_state->imu.quaternion, waist_angles);

            std::cout << LOGGER::INFO << "Motion duration: " << rl.motion_length << "s" << std::endl;

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        // Calculate motion time and progress
        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = std::fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        rl.motion_loader->Update(motion_time);

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

class RLFSMStateRLWholeBodyTrackingGangnamStyle : public RLFSMState
{
public:
RLFSMStateRLWholeBodyTrackingGangnamStyle(RL *rl) : RLFSMState(*rl, "RLFSMStateRLWholeBodyTrackingGangnamStyle") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "whole_body_tracking/gangnam_style";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);

            // Initialize motion loader
            std::string motion_file_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + rl.params.Get<std::string>("motion_file");
            float fps = 1.0f / (rl.params.Get<float>("dt") * rl.params.Get<int>("decimation"));
            rl.motion_loader = std::make_unique<MotionLoader>(motion_file_path, fps);
            rl.motion_length = rl.motion_loader->GetDuration();

            auto waist_sdk_indices = rl.params.Get<std::vector<int>>("waist_joint_indices");
            std::vector<float> waist_angles = {
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[0])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[1])],
                fsm_state->motor_state.q[rl.InverseJointMapping(waist_sdk_indices[2])]
            };
            rl.motion_loader->Reset(fsm_state->imu.quaternion, waist_angles);

            std::cout << LOGGER::INFO << "Motion duration: " << rl.motion_length << "s" << std::endl;

            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        // Calculate motion time and progress
        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = std::fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        rl.motion_loader->Update(motion_time);

        RLControl();

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        return state_name_;
    }
};

// ============================================================================
// Pre-stage for the ASAP dab. Uses fixed-PD interpolation (no policy) to bring
// the body from wherever it is right now to the dab's frame-0 pose
// (elbows pre-bent ~68°, waist pitched -27°, hips tucked, etc.) over a few
// seconds, then auto-transitions to RLFSMStateRLASAPDab. Without this
// pre-stage, ASAP's first action sees a body that's far from where the
// trained trajectory expects it, producing a violent corrective torque
// (the transition jolt). With this pre-stage, ASAP starts from on-trajectory.
// ============================================================================
class RLFSMStateASAPDabGetReady : public RLFSMState
{
public:
    RLFSMStateASAPDabGetReady(RL *rl) : RLFSMState(*rl, "RLFSMStateASAPDabGetReady") {}

    // Frame-0 pose of the wall-dab motion in 29-DOF SDK order. Computed
    // offline from the .pkl (see scripts/check_dab_start_pose.py). Wrist
    // joints (19-21, 26-28) aren't in the motion, so they stay at 0.
    static constexpr float kDabFrame0[29] = {
        -0.383f,  0.067f, -0.232f,  0.021f,  0.001f,  0.0f,  // L leg
        -0.517f, -0.061f, -0.200f,  0.199f,  0.008f,  0.0f,  // R leg
         0.178f,  0.015f, -0.468f,                           // waist
         0.145f,  0.158f, -0.449f,  1.195f,  0.0f, 0.0f, 0.0f,  // L arm
         0.149f, -0.144f,  0.164f,  1.191f,  0.0f, 0.0f, 0.0f,  // R arm
    };

    // Soft-but-firm fixed PD gains. Mirrors ASAP's fixed_kp/fixed_kd from
    // policy/g1/asap_dab/config.yaml — gentle enough to ease into the lean
    // without slamming, firm enough to actually reach the target.
    static constexpr float kInterpKp[29] = {
        100, 100, 100, 150,  40,  40,
        100, 100, 100, 150,  40,  40,
        300, 300, 300,
        100, 100,  50,  50,  20,  20,  20,
        100, 100,  50,  50,  20,  20,  20,
    };
    static constexpr float kInterpKd[29] = {
        2, 2, 2, 4, 2, 2,
        2, 2, 2, 4, 2, 2,
        3, 3, 3,
        2, 2, 2, 2, 1, 1, 1,
        2, 2, 2, 2, 1, 1, 1,
    };

    static constexpr float kInterpDurationSec = 3.0f;

    float percent = 0.0f;
    std::vector<float> start_pose;

    void Enter() override
    {
        std::cout << LOGGER::INFO
                  << "[ASAPDabGetReady] Easing into dab frame-0 pose over "
                  << kInterpDurationSec << "s. Hold still — robot is leaning forward.\n";
        percent = 0.0f;
        start_pose.assign(29, 0.0f);
        for (int i = 0; i < 29; ++i) start_pose[i] = fsm_state->motor_state.q[i];
    }

    void Run() override
    {
        // Advance percent by one FSM tick. dt is the physics tick (0.005s
        // = 200 Hz), set in the loaded config.
        float dt = rl.params.Get<float>("dt");
        percent += dt / kInterpDurationSec;
        if (percent > 1.0f) percent = 1.0f;

        for (int i = 0; i < 29; ++i)
        {
            float target = (1.0f - percent) * start_pose[i] + percent * kDabFrame0[i];
            fsm_command->motor_command.q[i]   = target;
            fsm_command->motor_command.dq[i]  = 0.0f;
            fsm_command->motor_command.kp[i]  = kInterpKp[i];
            fsm_command->motor_command.kd[i]  = kInterpKd[i];
            fsm_command->motor_command.tau[i] = 0.0f;
        }
        LOGGER::PrintProgress(percent, "DabGetReady");
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        // Emergency / abort routes — always available.
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        // Auto-transition into the actual dab once we've reached frame-0.
        if (percent >= 1.0f)
        {
            return "RLFSMStateRLASAPDab";
        }
        return state_name_;
    }
};

// ============================================================================
// ASAP wall-dab motion tracking policy. Loaded as a TorchScript wrapper at
// policy/g1/asap_dab/policy.pt that converts rl_sar's term-priority history
// layout into the ASAP actor's expected 380-dim layout. Phase advances over
// 5.93s (the dab motion duration). Entered from RLFSMStateASAPDabGetReady,
// which has already placed the body in frame-0 pose.
// ============================================================================
class RLFSMStateRLASAPDab : public RLFSMState
{
public:
    RLFSMStateRLASAPDab(RL *rl) : RLFSMState(*rl, "RLFSMStateRLASAPDab") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;

        // DIAGNOSTIC: snapshot pre-switch state BEFORE InitRL() runs. InitRL
        // loads the TorchScript model from disk (~50-100ms) and the simulator
        // keeps stepping during that window — so reading motor_state.q AFTER
        // InitRL gives a corrupted "post-load drift" value, not the true
        // moment-of-switch state. Save these on the stack first, log them
        // later once the file is open.
        int ndof_entry = rl.params.Get<int>("num_of_dofs");
        std::vector<float> pre_switch_q(ndof_entry);
        std::vector<float> pre_switch_tgt(ndof_entry);
        std::vector<float> pre_switch_kp(ndof_entry);
        for (int i = 0; i < ndof_entry; ++i) {
            pre_switch_q[i]   = fsm_state->motor_state.q[i];
            pre_switch_tgt[i] = fsm_command->motor_command.q[i];
            pre_switch_kp[i]  = fsm_command->motor_command.kp[i];
        }
        // Wall-clock time of the switch — anchor for the time axis.
        wallclock_anchor_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        rl.config_name = "asap_dab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }

        // Wall-dab motion duration in seconds (5.93s, from the .pkl file).
        rl.motion_length = 5.93f;

        entry_pose = pre_switch_q;  // for the existing first-tick diff print
        logged_first_tick = false;
        ramp_in_ticks = 0;
        ramp_in_active = (kEntryBlendSec > 0.0f);

        // DIAGNOSTIC: open the CSV log and write the pre-switch row using
        // values captured BEFORE InitRL. wallclock_ms = 0 marks the switch.
        diag_log_file.open("/tmp/rl_sar_dab_log.csv");
        if (diag_log_file.is_open())
        {
            diag_log_file << "time_ms,wallclock_ms,tag";
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << ",q" << i;
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << ",tgt" << i;
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << ",kp" << i;
            diag_log_file << "\n";
            diag_log_file << "0,0,loco_last";
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << "," << pre_switch_q[i];
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << "," << pre_switch_tgt[i];
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << "," << pre_switch_kp[i];
            diag_log_file << "\n";
            diag_log_file.flush();
        }

        // JOLT FIX (1/2): seed motor_command.q with the robot's *current*
        // joint positions (in ASAP/SDK order, since InitRL has applied the
        // new mapping). PD now starts with zero error, which neutralizes the
        // "leftover-locomotion bytes reinterpreted under new mapping" jolt
        // we'd otherwise see at the gap edge.
        for (int i = 0; i < ndof_entry; ++i)
        {
            fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
        }
    }

    // Fall-detection thresholds (degrees). If pitch or roll exceeds these
    // during the dab, abort and hand control to the locomotion policy which
    // is robust enough to stabilize from most non-fully-fallen states.
    static constexpr float kFallPitchDeg = 30.0f;
    static constexpr float kFallRollDeg  = 30.0f;
    bool fall_detected = false;

    // Entry-blend duration. ASAP is a motion-tracking policy trained to be
    // on-trajectory at every phase, so any blending of its output target
    // with an off-trajectory pose triggers violent corrective actions
    // (flips, falls). Keep this at 0 — the leftover-buffer seed in Enter()
    // is a separate fix that's safe to keep regardless. The residual
    // first-action jolt is the policy's natural cost of operating from a
    // standing entry, and only fully goes away with proper trajectory
    // pre-positioning (out of scope here).
    static constexpr float kEntryBlendSec = 0.0f;
    int ramp_in_ticks = 0;
    bool ramp_in_active = true;

    // DIAGNOSTIC: entry-pose vs first-policy-target logging. Compares where
    // the robot actually is at press-time vs what the policy's first PD
    // target demands, to quantify the jolt on tick 1. Remove after checking.
    std::vector<float> entry_pose;
    bool logged_first_tick = false;

    // DIAGNOSTIC: CSV time-series log of the locomotion → ASAP-dab transition.
    // First row captures locomotion's last command (entry snapshot, taken
    // BEFORE the slow InitRL() call so it's not corrupted by sim drift).
    // Every subsequent row captures one ASAP tick with both the simulated
    // time (episode_length_buf-derived) and the wall-clock time (so the
    // model-load gap is measurable and excludable from plots).
    std::ofstream diag_log_file;
    long long wallclock_anchor_ms = 0;  // wall-clock millis at FSM-switch moment

    void Run() override
    {
        if (!rl.rl_init_done) rl.rl_init_done = true;

        float motion_time = rl.episode_length_buf * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
        motion_time = fmin(motion_time, rl.motion_length);
        float percent = motion_time / rl.motion_length;
        LOGGER::PrintProgress(percent, rl.config_name);

        RLControl();

        // JOLT FIX: ease the PD target from entry pose toward the policy's
        // commanded target over kEntryBlendSec. Phase advances normally
        // (no freeze), so the policy stays fully active and can balance —
        // we just dampen the first-action step in PD-target space.
        if (ramp_in_active)
        {
            float t_sec = ramp_in_ticks * rl.params.Get<float>("dt") * rl.params.Get<int>("decimation");
            float alpha = (kEntryBlendSec > 0.0f) ? std::fmin(t_sec / kEntryBlendSec, 1.0f) : 1.0f;
            if ((int)entry_pose.size() == rl.params.Get<int>("num_of_dofs"))
            {
                int n = (int)entry_pose.size();
                for (int i = 0; i < n; ++i)
                {
                    float policy_tgt = fsm_command->motor_command.q[i];
                    fsm_command->motor_command.q[i] =
                        alpha * policy_tgt + (1.0f - alpha) * entry_pose[i];
                }
            }
            ramp_in_ticks++;
            if (alpha >= 1.0f) ramp_in_active = false;
        }

        // DIAGNOSTIC: on the first tick only, print the first PD target the
        // policy produced vs the pose the robot was in at press-time. A large
        // delta here is exactly the jolt hypothesis — it proves the policy is
        // asking the robot to snap to the motion's frame-0 pose instantly.
        if (!logged_first_tick)
        {
            static const char* joint_names[29] = {
                "L_hip_p","L_hip_r","L_hip_y","L_knee","L_ank_p","L_ank_r",
                "R_hip_p","R_hip_r","R_hip_y","R_knee","R_ank_p","R_ank_r",
                "waist_y","waist_r","waist_p",
                "L_sh_p","L_sh_r","L_sh_y","L_elbow","L_wr_r","L_wr_p","L_wr_y",
                "R_sh_p","R_sh_r","R_sh_y","R_elbow","R_wr_r","R_wr_p","R_wr_y"
            };
            int ndof_log = rl.params.Get<int>("num_of_dofs");
            std::printf("\n[ASAPDab DIAG] === First-tick jolt analysis ===\n");
            std::printf("[ASAPDab DIAG] %3s  %-8s  %8s  %8s  %10s\n",
                        "idx", "joint", "entry_q", "target", "delta_deg");
            float max_abs_delta = 0.0f; int max_i = 0;
            double sum_sq = 0.0;
            for (int i = 0; i < ndof_log && i < 29; ++i)
            {
                float target = fsm_command->motor_command.q[i];
                float entry  = entry_pose[i];
                float delta_deg = (target - entry) * 57.2958f;
                sum_sq += (target - entry) * (target - entry);
                if (std::fabs(delta_deg) > std::fabs(max_abs_delta)) { max_abs_delta = delta_deg; max_i = i; }
                std::printf("[ASAPDab DIAG] %3d  %-8s  %8.3f  %8.3f  %+10.1f\n",
                            i, joint_names[i], entry, target, delta_deg);
            }
            std::printf("[ASAPDab DIAG] Largest delta: %s = %+.1f deg\n",
                        joint_names[max_i], max_abs_delta);
            std::printf("[ASAPDab DIAG] L2 norm of delta: %.3f rad\n\n", std::sqrt(sum_sq));
            logged_first_tick = true;
        }

        // DIAGNOSTIC: append one CSV row per ASAP tick. Logs both the
        // simulated time (from episode_length_buf) and wall-clock time
        // (relative to FSM-switch moment), so the plotter can detect and
        // exclude the InitRL model-load gap.
        if (diag_log_file.is_open())
        {
            float dt = rl.params.Get<float>("dt");
            int decim = rl.params.Get<int>("decimation");
            float time_ms = rl.episode_length_buf * dt * decim * 1000.0f;
            long long wc_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            long long wallclock_ms = wc_now_ms - wallclock_anchor_ms;
            int ndof_csv = rl.params.Get<int>("num_of_dofs");
            diag_log_file << time_ms << "," << wallclock_ms << ",asap";
            for (int i = 0; i < ndof_csv; ++i) diag_log_file << "," << fsm_state->motor_state.q[i];
            for (int i = 0; i < ndof_csv; ++i) diag_log_file << "," << fsm_command->motor_command.q[i];
            for (int i = 0; i < ndof_csv; ++i) diag_log_file << "," << fsm_command->motor_command.kp[i];
            diag_log_file << "\n";
        }

        // Fall detection: read IMU quaternion (w,x,y,z) and check euler angles.
        // QuaternionToEuler returns [roll, pitch, yaw] in radians.
        std::vector<float> euler = QuaternionToEuler(fsm_state->imu.quaternion);
        float roll_deg = euler[0] * 57.2958f;
        float pitch_deg = euler[1] * 57.2958f;

        if (!fall_detected && (std::fabs(roll_deg) > kFallRollDeg || std::fabs(pitch_deg) > kFallPitchDeg))
        {
            std::cout << LOGGER::WARNING << "[ASAPDab] Fall detected (roll=" << roll_deg
                      << "deg, pitch=" << pitch_deg << "deg). Aborting to locomotion policy." << std::endl;
            fall_detected = true;
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
            return;
        }

        if (motion_time / rl.motion_length == 1)
        {
            rl.fsm.RequestStateChange("RLFSMStateRLRoboMimicLocomotion");
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
        fall_detected = false;

        // DIAGNOSTIC: close CSV log so the data is flushed and visible.
        if (diag_log_file.is_open())
        {
            diag_log_file.close();
            std::cout << LOGGER::INFO << "[ASAPDab DIAG] CSV log written to /tmp/rl_sar_dab_log.csv" << std::endl;
        }
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLRoboMimicLocomotion";
        }
        return state_name_;
    }
};

} // namespace g1_fsm

class G1FSMFactory : public FSMFactory
{
public:
    G1FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<g1_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<g1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<g1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLRoboMimicLocomotion")
            return std::make_shared<g1_fsm::RLFSMStateRLRoboMimicLocomotion>(rl);
        else if (state_name == "RLFSMStateRLRoboMimicCharleston")
            return std::make_shared<g1_fsm::RLFSMStateRLRoboMimicCharleston>(rl);
        else if (state_name == "RLFSMStateRLWholeBodyTrackingDance102")
            return std::make_shared<g1_fsm::RLFSMStateRLWholeBodyTrackingDance102>(rl);
        else if (state_name == "RLFSMStateRLWholeBodyTrackingGangnamStyle")
            return std::make_shared<g1_fsm::RLFSMStateRLWholeBodyTrackingGangnamStyle>(rl);
        else if (state_name == "RLFSMStateASAPDabGetReady")
            return std::make_shared<g1_fsm::RLFSMStateASAPDabGetReady>(rl);
        else if (state_name == "RLFSMStateRLASAPDab")
            return std::make_shared<g1_fsm::RLFSMStateRLASAPDab>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "g1"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLRoboMimicLocomotion",
            "RLFSMStateRLRoboMimicCharleston",
            "RLFSMStateRLWholeBodyTrackingDance102",
            "RLFSMStateRLWholeBodyTrackingGangnamStyle",
            "RLFSMStateASAPDabGetReady",
            "RLFSMStateRLASAPDab"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(G1FSMFactory, "RLFSMStatePassive")

#endif // G1_FSM_HPP
