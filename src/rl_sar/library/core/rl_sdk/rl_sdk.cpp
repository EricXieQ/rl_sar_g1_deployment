/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sdk.hpp"
#include <filesystem>

void RL::StateController(const RobotState<float>* state, RobotCommand<float>* command)
{
    auto updateState = [&](std::shared_ptr<FSMState> statePtr)
    {
        if (auto rl_fsm_state = std::dynamic_pointer_cast<RLFSMState>(statePtr))
        {
            rl_fsm_state->fsm_state = state;
            rl_fsm_state->fsm_command = command;
        }
    };
    for (auto& pair : fsm.states_)
    {
        updateState(pair.second);
    }

    // DIVERGENCE SAFEGUARD: the RL loop sets safeguard_trip_ when a policy output
    // runs away (e.g. a joint winding up against an external constraint). Consume
    // it here on the FSM thread and bail to Passive damping (one-shot, so the
    // operator can re-engage a policy afterward without a restart).
    if (this->safeguard_trip_.exchange(false))
    {
        std::cout << std::endl << LOGGER::WARNING << "[SAFEGUARD] policy output diverged -> bailing to Passive (damping)" << std::endl;
        this->fsm.RequestStateChange("RLFSMStatePassive");
    }

    fsm.Run();

    this->motiontime++;

    if (this->control.current_keyboard == Input::Keyboard::W)
    {
        this->control.x += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::S)
    {
        this->control.x -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::A)
    {
        this->control.y += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::D)
    {
        this->control.y -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Q)
    {
        this->control.yaw += 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::E)
    {
        this->control.yaw -= 0.1f;
    }
    if (this->control.current_keyboard == Input::Keyboard::Space)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
    {
        this->control.navigation_mode = !this->control.navigation_mode;
        std::cout << std::endl << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
    }
}

std::vector<float> RL::ComputeObservation()
{
    std::vector<std::vector<float>> obs_list;

    for (const std::string &observation : this->params.Get<std::vector<std::string>>("observations"))
    {
        // ============= Base Observations =============
        if (observation == "lin_vel")
        {
            obs_list.push_back(this->obs.lin_vel * this->params.Get<float>("lin_vel_scale"));
        }
        else if (observation == "ang_vel")
        {
            // In ROS1 Gazebo, the coordinate system for angular velocity is in the world coordinate system.
            // In ROS2 Gazebo, mujoco and real robot, the coordinate system for angular velocity is in the body coordinate system.
            if (this->ang_vel_axis == "body")
            {
                obs_list.push_back(this->obs.ang_vel * this->params.Get<float>("ang_vel_scale"));
            }
            else if (this->ang_vel_axis == "world")
            {
                obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.ang_vel) * this->params.Get<float>("ang_vel_scale"));
            }
        }
        else if (observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(this->obs.base_quat, this->obs.gravity_vec));
        }
        else if (observation == "commands")
        {
            obs_list.push_back(this->obs.commands * this->params.Get<std::vector<float>>("commands_scale"));
        }
        else if (observation == "dof_pos")
        {
            std::vector<float> dof_pos_rel = this->obs.dof_pos - this->params.Get<std::vector<float>>("default_dof_pos");
            for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
            {
                dof_pos_rel[i] = 0.0f;
            }
            obs_list.push_back(dof_pos_rel * this->params.Get<float>("dof_pos_scale"));
        }
        else if (observation == "dof_vel")
        {
            obs_list.push_back(this->obs.dof_vel * this->params.Get<float>("dof_vel_scale"));
        }
        else if (observation == "actions")
        {
            obs_list.push_back(this->obs.actions);
        }
        // ============= Other Observations =============
        else if (observation == "whole_body_tracking/motion_command")
        {
            std::vector<float> motion_cmd;
            if (this->motion_loader)
            {
                auto joint_pos_sdk = this->motion_loader->GetJointPos();
                auto joint_vel_sdk = this->motion_loader->GetJointVel();
                auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
                std::vector<float> joint_pos_training(joint_mapping.size());
                std::vector<float> joint_vel_training(joint_mapping.size());
                for (size_t i = 0; i < joint_mapping.size(); ++i)
                {
                    joint_pos_training[i] = joint_pos_sdk[joint_mapping[i]];
                    joint_vel_training[i] = joint_vel_sdk[joint_mapping[i]];
                }
                motion_cmd.insert(motion_cmd.end(), joint_pos_training.begin(), joint_pos_training.end());
                motion_cmd.insert(motion_cmd.end(), joint_vel_training.begin(), joint_vel_training.end());
            }
            else
            {
                motion_cmd.resize(this->params.Get<int>("num_of_dofs") * 2, 0.0f);
            }
            obs_list.push_back(motion_cmd);
        }
        else if (observation == "whole_body_tracking/motion_anchor_ori_b")
        {
            std::vector<float> anchor_ori(6, 0.0f);
            if (this->motion_loader)
            {
                auto waist_sdk_indices = this->params.Get<std::vector<int>>("waist_joint_indices");
                std::vector<float> waist_angles = {
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[0])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[1])],
                    this->obs.dof_pos[InverseJointMapping(waist_sdk_indices[2])]
                };
                std::vector<float> robot_torso_quat_w = MotionLoader::ComputeTorsoQuat(this->obs.base_quat, waist_angles);
                std::vector<float> ref_torso_quat_w = this->motion_loader->GetAnchorQuat();
                std::vector<float> init_quat = this->motion_loader->GetInitQuat();
                std::vector<float> motion_anchor_quat_w = QuaternionMultiply(init_quat, ref_torso_quat_w);
                std::vector<float> robot_quat_inv = QuaternionConjugate(robot_torso_quat_w);
                std::vector<float> relative_quat = QuaternionMultiply(robot_quat_inv, motion_anchor_quat_w);
                std::vector<float> rot_matrix = QuaternionToRotationMatrix(relative_quat);
                anchor_ori = MatrixFirstTwoColumns(rot_matrix);
            }
            obs_list.push_back(anchor_ori);
        }
        else if (observation == "RoboMimic_Deploy/phase")
        {
            float motion_time = this->episode_length_buf * this->params.Get<float>("dt") * this->params.Get<int>("decimation");
            float count = motion_time;
            float phase = count / this->motion_length;
            std::vector<float> phase_vec = {phase};
            obs_list.push_back(phase_vec);
        }
    }

    this->obs_dims.clear();
    for (const auto& obs : obs_list)
    {
       this->obs_dims.push_back(obs.size());
    }

    std::vector<float> obs;
    for (const auto& obs_vec : obs_list)
    {
        obs.insert(obs.end(), obs_vec.begin(), obs_vec.end());
    }
    std::vector<float> clamped_obs = clamp(obs, -this->params.Get<float>("clip_obs"), this->params.Get<float>("clip_obs"));
    return clamped_obs;
}

void RL::InitObservations()
{
    this->obs.lin_vel = {0.0f, 0.0f, 0.0f};
    this->obs.ang_vel = {0.0f, 0.0f, 0.0f};
    this->obs.gravity_vec = {0.0f, 0.0f, -1.0f};
    this->obs.commands = {0.0f, 0.0f, 0.0f};
    this->obs.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    this->obs.dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->obs.dof_vel.clear();
    this->obs.dof_vel.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->obs.actions.clear();
    this->obs.actions.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    // Reset the action smoothing filter so it doesn't carry stale values
    // from a previous policy state.
    this->last_action_smoothed.clear();
    this->last_action_smoothed.resize(this->params.Get<int>("num_of_dofs"), 0.0f);
    this->ComputeObservation();
}

void RL::InitOutputs()
{
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->output_dof_tau.clear();
    this->output_dof_tau.resize(num_of_dofs, 0.0f);
    this->output_dof_pos = this->params.Get<std::vector<float>>("default_dof_pos");
    this->output_dof_vel.clear();
    this->output_dof_vel.resize(num_of_dofs, 0.0f);
}

void RL::InitControl()
{
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
}

void RL::InitJointNum(size_t num_joints)
{
    this->robot_state.motor_state.resize(num_joints);
    this->start_state.motor_state.resize(num_joints);
    this->now_state.motor_state.resize(num_joints);
    this->robot_command.motor_command.resize(num_joints);
}

void RL::InitRL(std::string robot_config_path)
{
    std::lock_guard<std::mutex> lock(this->model_mutex);

    this->ReadYaml(robot_config_path, "config.yaml");

    // init joint num first
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));

    // init rl
    this->InitObservations();
    this->InitOutputs();
    this->InitControl();

    // init obs history
    const auto& observations_history = this->params.Get<std::vector<int>>("observations_history");  // avoid dangling reference
    if (!observations_history.empty())
    {
        int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;
        this->history_obs_buf = ObservationBuffer(1, this->obs_dims, history_length, this->params.Get<std::string>("observations_history_priority"));
        // Buffer is empty after recreation -> seed it on the first Forward()
        // so the policy's first inference uses a full, consistent history
        // instead of a partial/wrong-order one (the handoff-jolt fix).
        this->history_needs_seed = true;
    }

    // Entry-only command interpolation: ramp the FIRST action over
    // interp_entry_steps PD ticks (default 2x decimation), then run direct so
    // the dab is crisp. Arm the countdown only when the config enables it.
    this->interp_entry_steps = std::max(1, this->params.Get<int>("interp_entry_steps", 2 * std::max(1, this->params.Get<int>("decimation"))));
    const bool interp_on = this->params.Has("interpolate_commands") && this->params.Get<bool>("interpolate_commands");
    this->interp_entry_remaining = interp_on ? this->interp_entry_steps : 0;
    this->interp_entry_captured = false;
    this->interp_q_target.clear();

    // init model — prefer a preloaded (cached) model so the FSM switch does
    // NOT read from disk mid-control. Only fall back to a disk load if this
    // config wasn't preloaded. Timed + logged so the cache-hit (no disk I/O)
    // vs disk-load path is directly measurable evidence for the preload fix.
    auto t_model0 = std::chrono::high_resolution_clock::now();
    bool model_from_cache = false;
    auto cached = this->preloaded_models_.find(robot_config_path);
    if (cached != this->preloaded_models_.end() && cached->second)
    {
        this->model = cached->second;  // instant alias, no disk I/O
        model_from_cache = true;
    }
    else
    {
        std::string model_path = std::string(POLICY_DIR) + "/" + robot_config_path + "/" + this->params.Get<std::string>("model_name");
        this->model = InferenceRuntime::ModelFactory::load_model(model_path);
    }
    if (!this->model)
    {
        throw std::runtime_error("Failed to load model for: " + robot_config_path);
    }
    auto t_model1 = std::chrono::high_resolution_clock::now();
    double model_acquire_ms = std::chrono::duration<double, std::milli>(t_model1 - t_model0).count();
    std::cout << LOGGER::INFO << "[INITRL] model for " << robot_config_path
              << (model_from_cache ? " from CACHE (preloaded, no disk I/O)" : " from DISK")
              << " | acquire = " << model_acquire_ms << " ms" << std::endl;
}

void RL::PreloadModels(const std::string& robot_name)
{
    // Load every policy model under POLICY_DIR/<robot_name>/ into the cache,
    // once, at startup (off the time-critical path). Keyed by the config path
    // relative to POLICY_DIR (e.g. "g1/asap_dab") so InitRL can alias it.
    namespace fs = std::filesystem;
    // A/B toggle for evidence: RL_DISABLE_PRELOAD=1 skips preloading so the
    // FSM switch falls back to the disk-load path (InitRL then logs "from DISK
    // | acquire = ~7 ms"). With preload on, the same switch logs "from CACHE
    // | acquire = ~0 ms". Running both gives a direct before/after timing.
    if (const char* dis = std::getenv("RL_DISABLE_PRELOAD"); dis && std::string(dis) != "0")
    {
        std::cout << LOGGER::WARNING << "[PRELOAD] disabled via RL_DISABLE_PRELOAD "
                  << "(FSM switch will load the model from disk)" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lock(this->model_mutex);
    const std::string policy_root = std::string(POLICY_DIR);
    fs::path robot_dir = fs::path(policy_root) / robot_name;
    if (!fs::exists(robot_dir))
    {
        std::cout << LOGGER::WARNING << "[PRELOAD] no policy dir: " << robot_dir << std::endl;
        return;
    }
    for (const auto& entry : fs::recursive_directory_iterator(robot_dir))
    {
        if (entry.path().filename() != "config.yaml") continue;
        const std::string cfg_path = fs::relative(entry.path().parent_path(), policy_root).string();
        if (this->preloaded_models_.count(cfg_path)) continue;
        try
        {
            YAML::Node node = YAML::LoadFile(entry.path().string())[cfg_path];
            if (!node || !node["model_name"]) continue;  // fixed-PD states (passive/getup) have no model
            const std::string model_name = node["model_name"].as<std::string>();
            const std::string model_path = policy_root + "/" + cfg_path + "/" + model_name;
            if (!fs::exists(model_path)) continue;
            std::shared_ptr<InferenceRuntime::Model> m = InferenceRuntime::ModelFactory::load_model(model_path);
            if (m)
            {
                this->preloaded_models_[cfg_path] = m;
                std::cout << LOGGER::INFO << "[PRELOAD] cached " << cfg_path << " (" << model_name << ")" << std::endl;
            }
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::WARNING << "[PRELOAD] skipped " << cfg_path << ": " << e.what() << std::endl;
        }
    }
}

void RL::ComputeOutput(const std::vector<float> &actions, std::vector<float> &output_dof_pos, std::vector<float> &output_dof_vel, std::vector<float> &output_dof_tau)
{
    auto action_scale_vec = this->params.Get<std::vector<float>>("action_scale");
    const float as_frac = this->action_scale_percent.load() / 100.0f;
    for (auto& v : action_scale_vec) v *= as_frac;

    auto rl_kp_vec = this->params.Get<std::vector<float>>("rl_kp");
    const float kp_frac = this->rl_kp_percent.load() / 100.0f;
    for (auto& v : rl_kp_vec) v *= kp_frac;

    std::vector<float> actions_scaled = actions * action_scale_vec;
    std::vector<float> pos_actions_scaled = actions_scaled;
    std::vector<float> vel_actions_scaled(actions.size(), 0.0f);
    for (int i : this->params.Get<std::vector<int>>("wheel_indices"))
    {
        pos_actions_scaled[i] = 0.0f;
        vel_actions_scaled[i] = actions_scaled[i];
    }
    std::vector<float> all_actions_scaled = pos_actions_scaled + vel_actions_scaled;
    output_dof_pos = pos_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos");
    output_dof_vel = vel_actions_scaled;
    output_dof_tau = rl_kp_vec * (all_actions_scaled + this->params.Get<std::vector<float>>("default_dof_pos") - this->obs.dof_pos) - this->params.Get<std::vector<float>>("rl_kd") * this->obs.dof_vel;
    output_dof_tau = clamp(output_dof_tau, -this->params.Get<std::vector<float>>("torque_limits"), this->params.Get<std::vector<float>>("torque_limits"));
}

int RL::InverseJointMapping(int idx) const
{
    auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    for (size_t i = 0; i < joint_mapping.size(); ++i) {
        if (joint_mapping[i] == idx) return (int)i;
    }
    return -1;
}

void RL::TorqueProtect(const std::vector<float>& origin_output_dof_tau)
{
    std::vector<int> out_of_range_indices;
    std::vector<float> out_of_range_values;
    for (size_t i = 0; i < origin_output_dof_tau.size(); ++i)
    {
        float torque_value = origin_output_dof_tau[i];
        float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[i];
        float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[i];

        if (torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if (!out_of_range_indices.empty())
    {
        for (size_t i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            float value = out_of_range_values[i];
            float limit_lower = -this->params.Get<std::vector<float>>("torque_limits")[index];
            float limit_upper = this->params.Get<std::vector<float>>("torque_limits")[index];

            std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

void RL::AttitudeProtect(const std::vector<float> &quaternion, float pitch_threshold, float roll_threshold)
{
    // Use QuaternionToEuler from vector_math.hpp
    std::vector<float> euler = QuaternionToEuler(quaternion);
    float roll = euler[0] * 57.2958f;   // Convert to degrees
    float pitch = euler[1] * 57.2958f;

    if (std::fabs(roll) > roll_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Roll exceeds " << roll_threshold << " degrees. Current: " << roll << " degrees." << std::endl;
    }
    if (std::fabs(pitch) > pitch_threshold)
    {
        this->control.SetKeyboard(Input::Keyboard::P);
        std::cout << LOGGER::WARNING << "Pitch exceeds " << pitch_threshold << " degrees. Current: " << pitch << " degrees." << std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

static int kbhit()
{
    static bool initialized = false;
    static termios original_term;

    // Initialize terminal to non-canonical mode on first call
    if (!initialized)
    {
        tcgetattr(STDIN_FILENO, &original_term);

        termios new_term = original_term;
        new_term.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
        new_term.c_cc[VMIN] = 0;   // Non-blocking read
        new_term.c_cc[VTIME] = 0;  // No timeout

        tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

        // Register cleanup function to restore terminal on exit
        static bool cleanup_registered = false;
        if (!cleanup_registered)
        {
            std::atexit([]() {
                tcsetattr(STDIN_FILENO, TCSANOW, &original_term);
            });
            cleanup_registered = true;
        }

        initialized = true;
    }

    // Non-blocking read of a single character
    char c;
    int result = read(STDIN_FILENO, &c, 1);

    return (result == 1) ? (unsigned char)c : -1;
}

void RL::KeyboardInterface()
{
    auto scale = [this](const char* label, std::atomic<int>& field, int delta,
                        const std::vector<float>& baseline) {
        int pct = field.load() + delta;
        if (pct < 0)   pct = 0;
        if (pct > 100) pct = 100;
        field.store(pct);
        float frac = pct / 100.0f;
        std::cout << std::endl << "[TUNE] " << label
                  << "  scale = " << pct << "%";
        if (baseline.empty())
        {
            std::cout << "  (YAML baseline not loaded)" << std::endl;
            return;
        }
        float mn = baseline.front() * frac;
        float mx = mn;
        for (float b : baseline)
        {
            float r = b * frac;
            if (r < mn) mn = r;
            if (r > mx) mx = r;
        }
        std::cout << "  |  final per joint: ";
        if (mn == mx) std::cout << mn;
        else          std::cout << mn << " .. " << mx;
        std::cout << std::endl;
    };

    int c = kbhit();
    if (c > 0)
    {
        switch (c)
        {
        case '+': case '=': scale("action_scale", this->action_scale_percent, +1, this->tuning_baseline_action_scale); return;
        case '-': case '_': scale("action_scale", this->action_scale_percent, -1, this->tuning_baseline_action_scale); return;
        case ']':           scale("kp+kd gain",   this->rl_kp_percent,        +1, this->tuning_baseline_rl_kp);        return;
        case '[':           scale("kp+kd gain",   this->rl_kp_percent,        -1, this->tuning_baseline_rl_kp);        return;
        case '0': this->control.SetKeyboard(Input::Keyboard::Num0); break;
        case '1': this->control.SetKeyboard(Input::Keyboard::Num1); break;
        case '2': this->control.SetKeyboard(Input::Keyboard::Num2); break;
        case '3': this->control.SetKeyboard(Input::Keyboard::Num3); break;
        case '4': this->control.SetKeyboard(Input::Keyboard::Num4); break;
        case '5': this->control.SetKeyboard(Input::Keyboard::Num5); break;
        case '6': this->control.SetKeyboard(Input::Keyboard::Num6); break;
        case '7': this->control.SetKeyboard(Input::Keyboard::Num7); break;
        case '8': this->control.SetKeyboard(Input::Keyboard::Num8); break;
        case '9': this->control.SetKeyboard(Input::Keyboard::Num9); break;
        case 'a': case 'A': this->control.SetKeyboard(Input::Keyboard::A); break;
        case 'b': case 'B': this->control.SetKeyboard(Input::Keyboard::B); break;
        case 'c': case 'C': this->control.SetKeyboard(Input::Keyboard::C); break;
        case 'd': case 'D': this->control.SetKeyboard(Input::Keyboard::D); break;
        case 'e': case 'E': this->control.SetKeyboard(Input::Keyboard::E); break;
        case 'f': case 'F': this->control.SetKeyboard(Input::Keyboard::F); break;
        case 'g': case 'G':
        {
            const bool en = !this->safeguard_enabled_.load();
            this->safeguard_enabled_.store(en);
            std::cout << std::endl << LOGGER::WARNING << "[SAFEGUARD] divergence guard " << (en ? "ENABLED" : "DISABLED") << std::endl;
            return;
        }
        case 'h': case 'H': this->control.SetKeyboard(Input::Keyboard::H); break;
        case 'i': case 'I': this->control.SetKeyboard(Input::Keyboard::I); break;
        case 'j': case 'J': this->control.SetKeyboard(Input::Keyboard::J); break;
        case 'k': case 'K': this->control.SetKeyboard(Input::Keyboard::K); break;
        case 'l': case 'L': this->control.SetKeyboard(Input::Keyboard::L); break;
        case 'm': case 'M': this->control.SetKeyboard(Input::Keyboard::M); break;
        case 'n': case 'N': this->control.SetKeyboard(Input::Keyboard::N); break;
        case 'o': case 'O': this->control.SetKeyboard(Input::Keyboard::O); break;
        case 'p': case 'P': this->control.SetKeyboard(Input::Keyboard::P); break;
        case 'q': case 'Q': this->control.SetKeyboard(Input::Keyboard::Q); break;
        case 'r': case 'R': this->control.SetKeyboard(Input::Keyboard::R); break;
        case 's': case 'S': this->control.SetKeyboard(Input::Keyboard::S); break;
        case 't': case 'T': this->control.SetKeyboard(Input::Keyboard::T); break;
        case 'u': case 'U': this->control.SetKeyboard(Input::Keyboard::U); break;
        case 'v': case 'V': this->control.SetKeyboard(Input::Keyboard::V); break;
        case 'w': case 'W': this->control.SetKeyboard(Input::Keyboard::W); break;
        case 'x': case 'X': this->control.SetKeyboard(Input::Keyboard::X); break;
        case 'y': case 'Y': this->control.SetKeyboard(Input::Keyboard::Y); break;
        case 'z': case 'Z': this->control.SetKeyboard(Input::Keyboard::Z); break;
        case ' ': this->control.SetKeyboard(Input::Keyboard::Space); break;
        case '\n': case '\r': this->control.SetKeyboard(Input::Keyboard::Enter); break;
        case 27:  // Escape sequence (for arrow keys on Unix/Linux/macOS)
        {
            char seq[2];
            // Try to read escape sequence non-blockingly
            if (read(STDIN_FILENO, &seq[0], 1) == 1)
            {
                if (seq[0] == '[')
                {
                    if (read(STDIN_FILENO, &seq[1], 1) == 1)
                    {
                        switch (seq[1])
                        {
                        case 'A': this->control.SetKeyboard(Input::Keyboard::Up); break;
                        case 'B': this->control.SetKeyboard(Input::Keyboard::Down); break;
                        case 'C': this->control.SetKeyboard(Input::Keyboard::Right); break;
                        case 'D': this->control.SetKeyboard(Input::Keyboard::Left); break;
                        default: break;
                        }
                    }
                }
                else
                {
                    // Plain escape key
                    this->control.SetKeyboard(Input::Keyboard::Escape);
                }
            }
            else
            {
                // Plain escape key
                this->control.SetKeyboard(Input::Keyboard::Escape);
            }
        } break;
        default:  break;
        }
    }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node &node)
{
    std::vector<T> values;
    for (const auto &val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

void RL::PushDiagRingFrame(const std::string& state_name,
                           const std::vector<float>& q,
                           const std::vector<float>& tgt,
                           const std::vector<float>& kp)
{
    std::lock_guard<std::mutex> lock(this->diag_ring_mutex);
    DiagRingFrame f;
    f.wallclock_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    f.state_name = state_name;
    f.q = q;
    f.tgt = tgt;
    f.kp = kp;
    this->diag_ring_buf.push_back(std::move(f));
    // Cap at ~1.5s of frames at 200Hz.
    static constexpr size_t kDiagRingMax = 300;
    while (this->diag_ring_buf.size() > kDiagRingMax)
        this->diag_ring_buf.pop_front();
}

void RL::LoadTuningBaseline(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(config_path)[file_path];
    }
    catch (const std::exception& e)
    {
        std::cout << LOGGER::WARNING << "[TUNE] failed to preload baseline from " << config_path
                  << " (" << e.what() << ")" << std::endl;
        return;
    }
    auto load_vec = [&](const char* key, std::vector<float>& dst) {
        auto node = root[key];
        if (!node || !node.IsSequence()) return;
        dst.clear();
        for (const auto& v : node) dst.push_back(v.as<float>());
    };
    load_vec("action_scale", this->tuning_baseline_action_scale);
    load_vec("rl_kp",        this->tuning_baseline_rl_kp);
    std::cout << LOGGER::INFO << "[TUNE] baseline loaded: action_scale(n="
              << this->tuning_baseline_action_scale.size()
              << ") rl_kp(n=" << this->tuning_baseline_rl_kp.size() << ")" << std::endl;
}

void RL::ReadYaml(const std::string& file_path, const std::string& file_name)
{
    std::string config_path = std::string(POLICY_DIR) + "/" + file_path + "/" + file_name;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path)[file_path];
    }
    catch (YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    for (auto it = config.begin(); it != config.end(); ++it)
    {
        std::string key = it->first.as<std::string>();
        this->params.config_node[key] = it->second;
    }
}

void RL::CSVInit(std::string robot_path)
{
    csv_filename = std::string(POLICY_DIR) + "/" + robot_path + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_cal_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "tau_est_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_pos_target_" << i << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << "joint_vel_" << i << ","; }

    file << std::endl;

    file.close();
}

void RL::CSVLogger(const std::vector<float>& torque, const std::vector<float>& tau_est, const std::vector<float>& joint_pos, const std::vector<float>& joint_pos_target, const std::vector<float>& joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << torque[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << tau_est[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_pos_target[i] << ","; }
    for(int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i) { file << joint_vel[i] << ","; }

    file << std::endl;

    file.close();
}

bool RLFSMState::Interpolate(
    float& percent,
    const std::vector<float>& start_pos,
    const std::vector<float>& target_pos,
    float duration_seconds,
    const std::string& description,
    bool use_fixed_gains)
{
    if (percent >= 1.0f)
    {
        return false;
    }

    if (percent == 0.0f)
    {
        float max_diff = 0.0f;
        for (size_t i = 0; i < start_pos.size() && i < target_pos.size(); ++i)
        {
            max_diff = std::max(max_diff, std::abs(start_pos[i] - target_pos[i]));
        }

        if (max_diff < 0.1f)
        {
            percent = 1.0f;
        }
    }

    int required_frames = std::max(1, static_cast<int>(std::ceil(duration_seconds / rl.params.Get<float>("dt"))));
    float step = 1.0f / required_frames;

    percent += step;
    percent = std::min(percent, 1.0f);

    auto kp = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kp") : rl.params.Get<std::vector<float>>("rl_kp");
    auto kd = use_fixed_gains ? rl.params.Get<std::vector<float>>("fixed_kd") : rl.params.Get<std::vector<float>>("rl_kd");

    for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
    {
        fsm_command->motor_command.q[i] = (1 - percent) * start_pos[i] + percent * target_pos[i];
        fsm_command->motor_command.dq[i] = 0;
        fsm_command->motor_command.kp[i] = kp[i];
        fsm_command->motor_command.kd[i] = kd[i];
        fsm_command->motor_command.tau[i] = 0;
    }

    if (!description.empty())
    {
        LOGGER::PrintProgress(percent, description);
    }

    if (percent >= 1.0f)
    {
        return false;
    }

    return true;
}

void RLFSMState::RLControl()
{
    const int ndof = rl.params.Get<int>("num_of_dofs");
    const bool interp = rl.params.Has("interpolate_commands") && rl.params.Get<bool>("interpolate_commands");

    std::vector<float> _output_dof_pos, _output_dof_vel;
    const bool got_new = rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel);
    if (got_new)
    {
        rl.interp_q_target = _output_dof_pos;   // latest policy target
        rl.interp_dq_target = _output_dof_vel;
    }

    // ENTRY RAMP: only for the first interp_entry_steps PD ticks after a switch,
    // ramp the command from the hold pose toward the live policy output. This
    // spreads the first-action step (hold pose -> dab opening pose) over ~tens
    // of ms instead of one hard tick. interp_q_target keeps updating as new
    // policy outputs arrive, so we ramp toward the *moving* reference (no phase
    // catch-up). Once the countdown hits 0 we fall through to direct application
    // and the dab runs at full crispness.
    if (interp && rl.interp_entry_remaining > 0 && (int)rl.interp_q_target.size() == ndof)
    {
        if (!rl.interp_entry_captured)
        {
            rl.interp_entry_pose.assign(ndof, 0.0f);
            for (int i = 0; i < ndof; ++i) rl.interp_entry_pose[i] = fsm_command->motor_command.q[i];
            rl.interp_entry_captured = true;
        }
        rl.interp_entry_remaining--;
        const int total = std::max(1, rl.interp_entry_steps);
        const float alpha = std::min(1.0f, (float)(total - rl.interp_entry_remaining) / (float)total);
        const auto kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto kd = rl.params.Get<std::vector<float>>("rl_kd");
        const float gain_frac = rl.rl_kp_percent.load() / 100.0f;  // single knob scales kp AND kd
        for (int i = 0; i < ndof; ++i)
        {
            fsm_command->motor_command.q[i] = (1.0f - alpha) * rl.interp_entry_pose[i] + alpha * rl.interp_q_target[i];
            fsm_command->motor_command.dq[i] = (i < (int)rl.interp_dq_target.size()) ? rl.interp_dq_target[i] : 0.0f;
            fsm_command->motor_command.kp[i] = kp[i] * gain_frac;
            fsm_command->motor_command.kd[i] = kd[i] * gain_frac;
            fsm_command->motor_command.tau[i] = 0;
        }
        return;
    }

    // DIRECT: post-entry (or interpolation off) -- apply the latest policy
    // target directly and hold it until the next one (zero-order hold).
    if (got_new)
    {
        const auto rl_kp = rl.params.Get<std::vector<float>>("rl_kp");
        const auto rl_kd = rl.params.Get<std::vector<float>>("rl_kd");
        const float gain_frac = rl.rl_kp_percent.load() / 100.0f;  // single knob scales kp AND kd
        for (int i = 0; i < ndof; ++i)
        {
            if (!_output_dof_pos.empty()) fsm_command->motor_command.q[i] = _output_dof_pos[i];
            if (!_output_dof_vel.empty()) fsm_command->motor_command.dq[i] = _output_dof_vel[i];
            fsm_command->motor_command.kp[i] = rl_kp[i] * gain_frac;
            fsm_command->motor_command.kd[i] = rl_kd[i] * gain_frac;
            fsm_command->motor_command.tau[i] = 0;
        }
    }
}
