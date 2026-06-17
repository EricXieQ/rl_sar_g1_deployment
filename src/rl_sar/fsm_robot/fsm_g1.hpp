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

        // HANDOFF RE-SEED (mirror of the dab's "JOLT FIX"): capture the robot's
        // current pose BEFORE InitRL swaps the joint_mapping. motor_state.q is in
        // the OUTGOING policy's order (e.g. the dab's identity map); convert it to
        // physical SDK order via that map. After InitRL we convert SDK -> this
        // policy's order, so the entry-interp ramp holds the TRUE pose instead of
        // a scrambled leftover command (joints getting each other's targets).
        int ndof_entry = rl.params.Get<int>("num_of_dofs");
        std::vector<int> prev_mapping = rl.params.Get<std::vector<int>>("joint_mapping");
        std::vector<float> sdk_hold(ndof_entry, 0.0f);
        for (int slot = 0; slot < ndof_entry && slot < (int)prev_mapping.size(); ++slot)
        {
            int sdk_idx = prev_mapping[slot];
            if (sdk_idx >= 0 && sdk_idx < ndof_entry)
                sdk_hold[sdk_idx] = fsm_state->motor_state.q[slot];
        }

        // read params from yaml
        rl.config_name = "robomimic/locomotion";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;

            // SDK order -> locomotion policy order, then seed the held command so
            // the entry ramp starts from the correct pose (no joint scramble).
            std::vector<int> loco_mapping = rl.params.Get<std::vector<int>>("joint_mapping");
            for (int slot = 0; slot < ndof_entry && slot < (int)loco_mapping.size(); ++slot)
            {
                int sdk_idx = loco_mapping[slot];
                if (sdk_idx >= 0 && sdk_idx < ndof_entry)
                    fsm_command->motor_command.q[slot] = sdk_hold[sdk_idx];
            }
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

        // DIAGNOSTIC: push current frame into the pre-switch ring buffer so
        // ASAPDab::Enter() can prepend it to the CSV log when the operator
        // presses key 5. Note: motor_command.q/kp are in *locomotion's*
        // policy-order at this point — the Python plotter remaps them.
        {
            int ndof = rl.params.Get<int>("num_of_dofs");
            std::vector<float> qv(ndof), tv(ndof), kv(ndof);
            for (int i = 0; i < ndof; ++i)
            {
                qv[i] = fsm_state->motor_state.q[i];
                tv[i] = fsm_command->motor_command.q[i];
                kv[i] = fsm_command->motor_command.kp[i];
            }
            rl.PushDiagRingFrame("Locomotion", qv, tv, kv);
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
            // Direct entry into the ASAP dab. We tried a fixed-PD pre-stage
            // (RLFSMStateASAPDabGetReady, removed) to pre-position the body
            // in frame-0 pose, but ASAP is a motion-tracking policy and
            // needs to be actively running to balance — any window where
            // it's not commanding lets the body drift, and the policy can't
            // recover from that. Direct switch keeps the residual first-
            // action jolt but the policy is alive immediately, which is
            // safer overall.
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
// ASAP wall-dab motion tracking policy. Loaded as a TorchScript wrapper at
// policy/g1/asap_dab/policy.pt that converts rl_sar's term-priority history
// layout into the ASAP actor's expected 380-dim layout. Phase advances over
// 5.93s (the dab motion duration). Entered directly from locomotion (key 5).
// We tried inserting a fixed-PD pre-stage to pre-position the body in
// frame-0 pose, but motion-tracking policies need to be actively running to
// balance — any window without policy commands lets the body drift, and the
// policy can't recover. So direct entry it is, with the residual first-
// action jolt as the cost. Future fix: retrain ASAP with wider initial-
// state distribution, or migrate to a motion-loader-based architecture
// (whole_body_tracking style) that re-anchors the trajectory at deploy time.
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
        // Capture locomotion's joint_mapping BEFORE InitRL swaps in the dab
        // config. pre_switch_q is in locomotion's policy order; we need this
        // mapping to convert it to SDK order for a correct hold-seed below.
        std::vector<int> loco_mapping = rl.params.Get<std::vector<int>>("joint_mapping");
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

        // entry_pose is seeded below in SDK order (see JOLT FIX 1/2).
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

            // DIAGNOSTIC: dump the locomotion ring buffer first. Each frame
            // gets a relative timestamp = wallclock - anchor (negative, since
            // these all happened BEFORE the FSM-switch moment captured by
            // wallclock_anchor_ms). Tags become "pre_<state_name>" so the
            // plotter can distinguish.
            {
                std::lock_guard<std::mutex> lock(rl.diag_ring_mutex);
                for (const auto& f : rl.diag_ring_buf)
                {
                    // Both time_ms and wallclock_ms are "ms since FSM switch"
                    // for consistency with asap rows. They'll be negative for
                    // pre-switch frames (which is what we want — they plot to
                    // the left of t=0).
                    long long rel_ms = f.wallclock_ms - wallclock_anchor_ms;
                    diag_log_file << rel_ms << "," << rel_ms
                                  << ",pre_" << f.state_name;
                    int n = (int)f.q.size();
                    for (int i = 0; i < ndof_entry; ++i)
                        diag_log_file << "," << (i < n ? f.q[i] : 0.0f);
                    for (int i = 0; i < ndof_entry; ++i)
                        diag_log_file << "," << (i < (int)f.tgt.size() ? f.tgt[i] : 0.0f);
                    for (int i = 0; i < ndof_entry; ++i)
                        diag_log_file << "," << (i < (int)f.kp.size() ? f.kp[i] : 0.0f);
                    diag_log_file << "\n";
                }
            }

            // Then the loco_last row (the snapshot taken BEFORE InitRL ran).
            diag_log_file << "0,0,loco_last";
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << "," << pre_switch_q[i];
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << "," << pre_switch_tgt[i];
            for (int i = 0; i < ndof_entry; ++i) diag_log_file << "," << pre_switch_kp[i];
            diag_log_file << "\n";
            diag_log_file.flush();
        }

        // JOLT FIX (1/2): seed motor_command.q to HOLD the robot's pre-switch
        // pose, in SDK order. pre_switch_q was captured in locomotion's policy
        // order, and motor_state.q here is still stale loco-order (GetState
        // hasn't run under the dab config yet) — so seeding directly from
        // either scrambles joints once the dab applies its identity mapping.
        // Remap via locomotion's joint_mapping: physical joint loco_mapping[slot]
        // held the value pre_switch_q[slot]. PD then holds the true entry pose
        // through the model-load / history-fill window instead of a scrambled
        // leftover command — killing the handoff jolt and the phantom plot jump.
        std::vector<float> sdk_entry_pose(ndof_entry, 0.0f);
        for (int slot = 0; slot < ndof_entry && slot < (int)loco_mapping.size(); ++slot)
        {
            int sdk_idx = loco_mapping[slot];
            if (sdk_idx >= 0 && sdk_idx < ndof_entry)
                sdk_entry_pose[sdk_idx] = pre_switch_q[slot];
        }
        for (int i = 0; i < ndof_entry; ++i)
            fsm_command->motor_command.q[i] = sdk_entry_pose[i];
        entry_pose = sdk_entry_pose;  // SDK order — correct first-tick diag & CSV
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
            "RLFSMStateRLASAPDab"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(G1FSMFactory, "RLFSMStatePassive")

#endif // G1_FSM_HPP
