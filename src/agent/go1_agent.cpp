#include "global_log.h"
#include "agent/go1_agent.h"

Go1Agent::Go1Agent(std::string model_type)
: safety_(UNITREE_LEGGED_SDK::LeggedType::Go1),
  udp_(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007)
{
    if(model_type == ModelType::GO1_ROUGH)
    {
        go1_rough_model_ = std::make_shared<Go1RoughModel>();
    }
    else if(model_type == ModelType::GO1_FLAT)
    {
        go1_flat_model_ = std::make_shared<Go1FlatModel>();
    }
    else
    {
        throw Go1AgentException("Invalid model type");
    }
    model_type_ = model_type;

    base_lin_vel_ = std::vector<float>(3, 0.0f);
    base_ang_vel_ = std::vector<float>(3, 0.0f);
    projected_gravity_ = std::vector<float>(3, 0.0f);
    velocity_commands_ = std::vector<float>(3, 0.0f);
    actions_ = std::vector<float>(kMotorAmount, 0.0f);
    last_actions_ = std::vector<float>(kMotorAmount, 0.0f);
    joint_positions_ = std::vector<float>(kMotorAmount, 0.0f);
    joint_velocities_ = std::vector<float>(kMotorAmount);
    height_scan_ = std::vector<float>(187, -0.1807f);

    udp_.InitCmdData(cmd_);

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].tau = 0; // 0.65
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].q = kProneAngles[0];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].tau = 0; // -0.65
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].q = kProneAngles[1];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].tau = 0;  // 0.65
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].q = kProneAngles[2];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].tau = 0; // -0.65
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].q = kProneAngles[3];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].tau = 0; 
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].q = kProneAngles[4];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].Kd = kMotorKd;
    
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].q = kProneAngles[5];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].q = kProneAngles[6];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].q = kProneAngles[7];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].q = kProneAngles[8];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].q = kProneAngles[9];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].Kd = kMotorKd;

    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].q = kProneAngles[10];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].Kd = kMotorKd;
    
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].q = kProneAngles[11];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].dq = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].Kp = kMotorKp;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].Kd = kMotorKd;

    sdk_send_thread_ = std::thread(&Go1Agent::SdkSend, this);
    sdk_recv_thread_ = std::thread(&Go1Agent::SdkReceive, this);
    sdk_send_thread_.detach();
    sdk_recv_thread_.detach();
}

Go1Agent::~Go1Agent()
{
    Go1CalibrateProne();
}

void Go1Agent::Run()
{
    if(model_type_ == ModelType::GO1_ROUGH)
    {
        Go1CalibrateStand();
        Go1RoughRun();
    }
    else if(model_type_ == ModelType::GO1_FLAT)
    {
        Go1CalibrateStand();
        Go1FlatRun();
    }
    else
    {
        throw Go1AgentException("Invalid model type");
    }
}

void Go1Agent::Go1RoughRun()
{
    while (true)
    {
        do
        {
            Go1RoughGetObs();
        }while(state_.motorState[UNITREE_LEGGED_SDK::FL_0].q == 0.0f);

        go1_rough_model_->SetBaseLinVel(base_lin_vel_);
        go1_rough_model_->SetBaseAngVel(base_ang_vel_);
        go1_rough_model_->SetProjectedGravity(projected_gravity_);
        go1_rough_model_->SetVelocityCommands(velocity_commands_);
        go1_rough_model_->SetJointPositions(joint_positions_);
        go1_rough_model_->SetJointVelocities(joint_velocities_);
        go1_rough_model_->SetActions(last_actions_);
        go1_rough_model_->SetHeightScan(height_scan_);

        actions_ = go1_rough_model_->RunModel();
        last_actions_ = actions_;
        Go1ProcessActions(actions_);
        Go1ExecuteActions(actions_);
        
        safety_.PositionLimit(cmd_);
        int safety_res = safety_.PowerProtect(cmd_, state_, 1);
        if(safety_res < 0)
        {
            exit(-1);
        }
        usleep(2000);
    }
}

void Go1Agent::Go1FlatRun()
{

    while (true)
    {
        do
        {
            Go1FlatGetObs();
        }while(joint_positions_[0] == 0.0f);

        // go1_flat_model_->SetBaseLinVel(base_lin_vel_);
        // go1_flat_model_->SetBaseAngVel(base_ang_vel_);
        // go1_flat_model_->SetProjectedGravity(projected_gravity_);
        // go1_flat_model_->SetVelocityCommands(velocity_commands_);
        // go1_flat_model_->SetJointPositions(joint_positions_);
        // go1_flat_model_->SetJointVelocities(joint_velocities_);
        // go1_flat_model_->SetActions(last_actions_);

        // actions_ = go1_flat_model_->RunModel();
        // last_actions_ = actions_;
        // Go1ProcessActions(actions_);
        // Go1ExecuteActions(actions_);
        
        spdlog::info("FL_hip   | target {:>7.4f} , {:>7.4f}",
            actions_[0], state_.motorState[UNITREE_LEGGED_SDK::FL_0].q);
        spdlog::info("FR_hip   | target {:>7.4f} , {:>7.4f}",
            actions_[1], state_.motorState[UNITREE_LEGGED_SDK::FR_0].q);
        spdlog::info("RL_hip   | target {:>7.4f} , {:>7.4f}",
            actions_[2], state_.motorState[UNITREE_LEGGED_SDK::RL_0].q);
        spdlog::info("RR_hip   | target {:>7.4f} , {:>7.4f}", 
            actions_[3], state_.motorState[UNITREE_LEGGED_SDK::RR_0].q);
        spdlog::info("FL_thigh | target {:>7.4f} , {:>7.4f}", 
            actions_[4], state_.motorState[UNITREE_LEGGED_SDK::FL_1].q);
        spdlog::info("FR_thigh | target {:>7.4f} , {:>7.4f}", 
            actions_[5], state_.motorState[UNITREE_LEGGED_SDK::FR_1].q);
        spdlog::info("RL_thigh | target {:>7.4f} , {:>7.4f}", 
            actions_[6], state_.motorState[UNITREE_LEGGED_SDK::RL_1].q);
        spdlog::info("RR_thigh | target {:>7.4f} , {:>7.4f}", 
            actions_[7], state_.motorState[UNITREE_LEGGED_SDK::RR_1].q);
        spdlog::info("FL_calf  | target {:>7.4f} , {:>7.4f}", 
            actions_[8], state_.motorState[UNITREE_LEGGED_SDK::FL_2].q);
        spdlog::info("FR_calf  | target {:>7.4f} , {:>7.4f}", 
            actions_[9], state_.motorState[UNITREE_LEGGED_SDK::FR_2].q);
        spdlog::info("RL_calf  | target {:>7.4f} , {:>7.4f}", 
            actions_[10], state_.motorState[UNITREE_LEGGED_SDK::RL_2].q);
        spdlog::info("RR_calf  | target {:>7.4f} , {:>7.4f}", 
            actions_[11], state_.motorState[UNITREE_LEGGED_SDK::RR_2].q);

        safety_.PositionLimit(cmd_);
        int safety_res = safety_.PowerProtect(cmd_, state_, 1);
        if(safety_res < 0)
        {
            exit(-1);
        }
        usleep(2000);
    }
}

void Go1Agent::Go1RoughGetObs()
{
    udp_.GetRecv(state_);

    spdlog::trace("sdk recv gyro: {:>7.4f} {:>7.4f} {:>7.4f}",
        state_.imu.gyroscope[0], state_.imu.gyroscope[1], state_.imu.gyroscope[2]
    );
    spdlog::trace("sdk recv accel: {:>7.4f} {:>7.4f} {:>7.4f}",
        state_.imu.accelerometer[0], state_.imu.accelerometer[1], state_.imu.accelerometer[2]
    );
    spdlog::trace("sdk recv eular: {:>7.4f} {:>7.4f} {:>7.4f}",
        state_.imu.rpy[0], state_.imu.rpy[1], state_.imu.rpy[2]
    );
    spdlog::debug("sdk recv motor pos: "
        "FL0:{:>7.4f} FR0:{:>7.4f} RL0:{:>7.4f} RR0:{:>7.4f} "
        "FL1:{:>7.4f} FR1:{:>7.4f} RL1:{:>7.4f} RR1:{:>7.4f} "
        "FL2:{:>7.4f} FR2:{:>7.4f} RL2:{:>7.4f} RR2:{:>7.4f}", 
        state_.motorState[UNITREE_LEGGED_SDK::FL_0].q, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_0].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_0].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_0].q,
        state_.motorState[UNITREE_LEGGED_SDK::FL_1].q, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_1].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_1].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_1].q,
        state_.motorState[UNITREE_LEGGED_SDK::FL_2].q, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_2].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_2].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_2].q
    );

    spdlog::trace("sdk recv motor vel: "
        "FL0:{:>7.4f} FR0:{:>7.4f} RL0:{:>7.4f} RR0:{:>7.4f} "
        "FL1:{:>7.4f} FR1:{:>7.4f} RL1:{:>7.4f} RR1:{:>7.4f} "
        "FL2:{:>7.4f} FR2:{:>7.4f} RL2:{:>7.4f} RR2:{:>7.4f}", 
        state_.motorState[UNITREE_LEGGED_SDK::FL_0].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_0].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_0].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_0].dq,
        state_.motorState[UNITREE_LEGGED_SDK::FL_1].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_1].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_1].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_1].dq,
        state_.motorState[UNITREE_LEGGED_SDK::FL_2].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_2].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_2].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_2].dq
    );

    joint_positions_[0] = state_.motorState[UNITREE_LEGGED_SDK::FL_0].q;
    joint_positions_[1] = state_.motorState[UNITREE_LEGGED_SDK::FR_0].q;
    joint_positions_[2] = state_.motorState[UNITREE_LEGGED_SDK::RL_0].q;
    joint_positions_[3] = state_.motorState[UNITREE_LEGGED_SDK::RR_0].q;
    joint_positions_[4] = state_.motorState[UNITREE_LEGGED_SDK::FL_1].q;
    joint_positions_[5] = state_.motorState[UNITREE_LEGGED_SDK::FR_1].q;
    joint_positions_[6] = state_.motorState[UNITREE_LEGGED_SDK::RL_1].q;
    joint_positions_[7] = state_.motorState[UNITREE_LEGGED_SDK::RR_1].q;
    joint_positions_[8] = state_.motorState[UNITREE_LEGGED_SDK::FL_2].q;
    joint_positions_[9] = state_.motorState[UNITREE_LEGGED_SDK::FR_2].q;
    joint_positions_[10] = state_.motorState[UNITREE_LEGGED_SDK::RL_2].q;
    joint_positions_[11] = state_.motorState[UNITREE_LEGGED_SDK::RR_2].q;

    joint_velocities_[0] = state_.motorState[UNITREE_LEGGED_SDK::FL_0].dq;
    joint_velocities_[1] = state_.motorState[UNITREE_LEGGED_SDK::FR_0].dq;
    joint_velocities_[2] = state_.motorState[UNITREE_LEGGED_SDK::RL_0].dq;
    joint_velocities_[3] = state_.motorState[UNITREE_LEGGED_SDK::RR_0].dq;
    joint_velocities_[4] = state_.motorState[UNITREE_LEGGED_SDK::FL_1].dq;
    joint_velocities_[5] = state_.motorState[UNITREE_LEGGED_SDK::FR_1].dq;
    joint_velocities_[6] = state_.motorState[UNITREE_LEGGED_SDK::RL_1].dq;
    joint_velocities_[7] = state_.motorState[UNITREE_LEGGED_SDK::RR_1].dq;
    joint_velocities_[8] = state_.motorState[UNITREE_LEGGED_SDK::FL_2].dq;
    joint_velocities_[9] = state_.motorState[UNITREE_LEGGED_SDK::FR_2].dq;
    joint_velocities_[10] = state_.motorState[UNITREE_LEGGED_SDK::RL_2].dq;
    joint_velocities_[11] = state_.motorState[UNITREE_LEGGED_SDK::RR_2].dq;

    base_ang_vel_[0] = state_.imu.gyroscope[0];
    base_ang_vel_[1] = state_.imu.gyroscope[1];
    base_ang_vel_[2] = state_.imu.gyroscope[2];

}

void Go1Agent::Go1FlatGetObs()
{
    udp_.GetRecv(state_);
    spdlog::trace("sdk recv gyro: {:>7.4f} {:>7.4f} {:>7.4f}",
        state_.imu.gyroscope[0], state_.imu.gyroscope[1], state_.imu.gyroscope[2]
    );
    spdlog::trace("sdk recv accel: {:>7.4f} {:>7.4f} {:>7.4f}",
        state_.imu.accelerometer[0], state_.imu.accelerometer[1], state_.imu.accelerometer[2]
    );
    spdlog::trace("sdk recv eular: {:>7.4f} {:>7.4f} {:>7.4f}",
        state_.imu.rpy[0], state_.imu.rpy[1], state_.imu.rpy[2]
    );
    spdlog::debug("sdk recv motor pos: "
        "FL0:{:>7.4f} FR0:{:>7.4f} RL0:{:>7.4f} RR0:{:>7.4f} "
        "FL1:{:>7.4f} FR1:{:>7.4f} RL1:{:>7.4f} RR1:{:>7.4f} "
        "FL2:{:>7.4f} FR2:{:>7.4f} RL2:{:>7.4f} RR2:{:>7.4f}", 
        state_.motorState[UNITREE_LEGGED_SDK::FL_0].q, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_0].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_0].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_0].q,
        state_.motorState[UNITREE_LEGGED_SDK::FL_1].q, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_1].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_1].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_1].q,
        state_.motorState[UNITREE_LEGGED_SDK::FL_2].q, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_2].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_2].q, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_2].q
    );

    spdlog::trace("sdk recv motor vel: "
        "FL0:{:>7.4f} FR0:{:>7.4f} RL0:{:>7.4f} RR0:{:>7.4f} "
        "FL1:{:>7.4f} FR1:{:>7.4f} RL1:{:>7.4f} RR1:{:>7.4f} "
        "FL2:{:>7.4f} FR2:{:>7.4f} RL2:{:>7.4f} RR2:{:>7.4f}", 
        state_.motorState[UNITREE_LEGGED_SDK::FL_0].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_0].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_0].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_0].dq,
        state_.motorState[UNITREE_LEGGED_SDK::FL_1].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_1].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_1].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_1].dq,
        state_.motorState[UNITREE_LEGGED_SDK::FL_2].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::FR_2].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RL_2].dq, 
        state_.motorState[UNITREE_LEGGED_SDK::RR_2].dq
    );

    joint_positions_[0] = state_.motorState[UNITREE_LEGGED_SDK::FL_0].q;
    joint_positions_[1] = state_.motorState[UNITREE_LEGGED_SDK::FR_0].q;
    joint_positions_[2] = state_.motorState[UNITREE_LEGGED_SDK::RL_0].q;
    joint_positions_[3] = state_.motorState[UNITREE_LEGGED_SDK::RR_0].q;
    joint_positions_[4] = state_.motorState[UNITREE_LEGGED_SDK::FL_1].q;
    joint_positions_[5] = state_.motorState[UNITREE_LEGGED_SDK::FR_1].q;
    joint_positions_[6] = state_.motorState[UNITREE_LEGGED_SDK::RL_1].q;
    joint_positions_[7] = state_.motorState[UNITREE_LEGGED_SDK::RR_1].q;
    joint_positions_[8] = state_.motorState[UNITREE_LEGGED_SDK::FL_2].q;
    joint_positions_[9] = state_.motorState[UNITREE_LEGGED_SDK::FR_2].q;
    joint_positions_[10] = state_.motorState[UNITREE_LEGGED_SDK::RL_2].q;
    joint_positions_[11] = state_.motorState[UNITREE_LEGGED_SDK::RR_2].q;

    joint_velocities_[0] = state_.motorState[UNITREE_LEGGED_SDK::FL_0].dq;
    joint_velocities_[1] = state_.motorState[UNITREE_LEGGED_SDK::FR_0].dq;
    joint_velocities_[2] = state_.motorState[UNITREE_LEGGED_SDK::RL_0].dq;
    joint_velocities_[3] = state_.motorState[UNITREE_LEGGED_SDK::RR_0].dq;
    joint_velocities_[4] = state_.motorState[UNITREE_LEGGED_SDK::FL_1].dq;
    joint_velocities_[5] = state_.motorState[UNITREE_LEGGED_SDK::FR_1].dq;
    joint_velocities_[6] = state_.motorState[UNITREE_LEGGED_SDK::RL_1].dq;
    joint_velocities_[7] = state_.motorState[UNITREE_LEGGED_SDK::RR_1].dq;
    joint_velocities_[8] = state_.motorState[UNITREE_LEGGED_SDK::FL_2].dq;
    joint_velocities_[9] = state_.motorState[UNITREE_LEGGED_SDK::FR_2].dq;
    joint_velocities_[10] = state_.motorState[UNITREE_LEGGED_SDK::RL_2].dq;
    joint_velocities_[11] = state_.motorState[UNITREE_LEGGED_SDK::RR_2].dq;

    base_ang_vel_[0] = state_.imu.gyroscope[0];
    base_ang_vel_[1] = state_.imu.gyroscope[1];
    base_ang_vel_[2] = state_.imu.gyroscope[2];
}

void Go1Agent::Go1CalibrateStand()
{

    size_t calibrate_steps = 100;
    float position_error[12];
    float calibration_action[12];
    spdlog::info("Calibrating stand...");
    do 
    {
        Go1RoughGetObs();
    }while(state_.motorState[UNITREE_LEGGED_SDK::FL_0].q == 0.0f);

    for(int i=0 ; i<kMotorAmount ; i++)
    {
        position_error[i] = kProneAngles[i] - joint_positions_[i];
    }
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].tau = -5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].tau = -5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].tau = 5;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].tau = 5;

    for(size_t i=0 ; i<calibrate_steps ; i++)
    {
        for(size_t j=0 ; j<kMotorAmount ; j++)
        {
            calibration_action[j] = joint_positions_[j] + position_error[j] / calibrate_steps * i;
            // if(j < 4)
            // {
            //     calibration_action[j] *= kHipScaleReduction;
            // }
        }
        Go1ExecuteActions(calibration_action);
        usleep(10000);
    }

    for(int i=0 ; i<kMotorAmount ; i++)
    {
        position_error[i] = kStandAngles[i] - kProneAngles[i];
    }
    for(size_t i=0 ; i<calibrate_steps ; i++)
    {
        for(size_t j=0 ; j<kMotorAmount ; j++)
        {
            calibration_action[j] = kProneAngles[j] + position_error[j] / calibrate_steps * i;
            // if(j < 4)
            // {
            //     calibration_action[j] *= kHipScaleReduction;
            // }
        }
        Go1ExecuteActions(calibration_action);
        usleep(10000);
    }
    sleep(3);
    spdlog::info("Calibrating stand done.");
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].tau = 0;
}

void Go1Agent::Go1CalibrateProne()
{
    size_t calibrate_steps = 100;
    float position_error[12];
    float calibration_action[12];
    spdlog::info("Calibrating prone...");
    do 
    {
        Go1RoughGetObs();
    }while(state_.motorState[UNITREE_LEGGED_SDK::FL_0].q == 0.0f);

    for(int i=0 ; i<kMotorAmount ; i++)
    {
        position_error[i] = kProneAngles[i] - joint_positions_[i];
    }
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].tau = 0;
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].tau = 0;

    for(size_t i=0 ; i<calibrate_steps ; i++)
    {
        for(size_t j=0 ; j<kMotorAmount ; j++)
        {
            calibration_action[j] = joint_positions_[j] + position_error[j] / calibrate_steps * i;
            // if(j < 4)
            // {
            //     calibration_action[j] *= kHipScaleReduction;
            // }
        }
        Go1ExecuteActions(calibration_action);
        usleep(10000);
    }
    spdlog::info("Calibrating prone done.");
}

void Go1Agent::Go1ProcessActions(std::vector<float>& actions)
{
    for(int i=0; i<actions.size() ; i++)
    {
        actions[i] *= kActionScale;
        if(i < 4)
        {
            actions[i] *= kHipScaleReduction;
        }
        actions[i] += kStandAngles[i];
    }
}

void Go1Agent::Go1ProcessActions(float actions[12])
{
    for(int i=0; i<kMotorAmount ; i++)
    {
        actions[i] *= kActionScale;
        if(i < 4)
        {
            actions[i] *= kHipScaleReduction;
        }
        actions[i] += kStandAngles[i];
    }
}

void Go1Agent::Go1ExecuteActions(std::vector<float>& actions)
{
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].q = actions[0];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].q = actions[1];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].q = actions[2];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].q = actions[3];

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].q = actions[4];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].q = actions[5];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].q = actions[6];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].q = actions[7];

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].q = actions[8];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].q = actions[9];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].q = actions[10];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].q = actions[11];

    udp_.SetSend(cmd_);
}

void Go1Agent::Go1ExecuteActions(float actions[12])
{
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].q = actions[0];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].q = actions[1];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].q = actions[2];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].q = actions[3];

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].q = actions[4];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].q = actions[5];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].q = actions[6];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].q = actions[7];

    cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].q = actions[8];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].q = actions[9];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].q = actions[10];
    cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].q = actions[11];

    spdlog::debug("sdk send motor pos: "
        "FL0:{:>7.4f} FR0:{:>7.4f} RL0:{:>7.4f} RR0:{:>7.4f} "
        "FL1:{:>7.4f} FR1:{:>7.4f} RL1:{:>7.4f} RR1:{:>7.4f} "
        "FL2:{:>7.4f} FR2:{:>7.4f} RL2:{:>7.4f} RR2:{:>7.4f} ", 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_0].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_0].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_0].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_0].q,
        cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_1].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_1].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_1].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_1].q,
        cmd_.motorCmd[UNITREE_LEGGED_SDK::FL_2].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::FR_2].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::RL_2].q, 
        cmd_.motorCmd[UNITREE_LEGGED_SDK::RR_2].q
    );
    udp_.SetSend(cmd_);
}

void Go1Agent::SdkReceive()
{
    while(1)
    {
        udp_.Recv();
    }
}

void Go1Agent::SdkSend()
{
    while(1)
    {
        udp_.Send();
    }
}

void Go1Agent::SetBaseLinVel(float x, float y, float z)
{
    base_lin_vel_[0] = x;
    base_lin_vel_[1] = y;
    base_lin_vel_[2] = z;
}
void Go1Agent::SetBaseLinVel(std::vector<float> base_lin_vel)
{
    base_lin_vel_ = base_lin_vel;
}
void Go1Agent::SetBaseAngVel(float x, float y, float z)
{
    base_ang_vel_[0] = x;
    base_ang_vel_[1] = y;
    base_ang_vel_[2] = z;
}
void Go1Agent::SetBaseAngVel(std::vector<float> base_ang_vel)
{
    base_ang_vel_ = base_ang_vel;
}
void Go1Agent::SetProjectedGravity(float x, float y, float z)
{
    projected_gravity_[0] = x;
    projected_gravity_[1] = y;
    projected_gravity_[2] = z;
}
void Go1Agent::SetProjectedGravity(std::vector<float> projected_gravity)
{
    projected_gravity_ = projected_gravity;
}
void Go1Agent::SetVelocityCommands(float x, float y, float w)
{
    velocity_commands_[0] = x;
    velocity_commands_[1] = y;
    velocity_commands_[2] = w;
}
void Go1Agent::SetVelocityCommands(std::vector<float> velocity_commands)
{
    velocity_commands_ = velocity_commands;
}