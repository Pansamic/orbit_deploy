#include <iostream>
#include <algorithm>
#include "global_log.h"
#include "model/go1_rough_model.h"

Go1RoughModel::Go1RoughModel() : OrbitBaseModel()
{
    std::string model_path(__FILE__);
    model_path = model_path.substr(0, model_path.find_last_of('/'));
    model_path = model_path.substr(0, model_path.find_last_of('/'));
    model_path = model_path.substr(0, model_path.find_last_of('/'));
    model_path += "/models/go1_rough.onnx";
    
    OrbitBaseModel::LoadModel(model_path.c_str());

    if(kInputSize != OrbitBaseModel::GetInputSize())
    {
        throw OrbitBaseModel::ModelInOutException("Input size mismatch");
    }
    if(kOutputSize != OrbitBaseModel::GetOutputSize())
    {
        throw OrbitBaseModel::ModelInOutException("Output size mismatch");
    }
}

Go1RoughModel::~Go1RoughModel()
{

}

std::vector<float> Go1RoughModel::RunModel()
{
    OrbitBaseModel::RunModel();
    return output_tensor_values_;
}

void Go1RoughModel::SetBaseLinVel(std::vector<float> base_lin_vel)
{
    if(base_lin_vel.size() != kBaseLinVelSize)
    {
        spdlog::error("Base linear velocity size mismatch: expected {}, got {}", kBaseLinVelSize, base_lin_vel.size());
        return;
    }
    std::copy(base_lin_vel.begin(), base_lin_vel.end(), input_tensor_values_.begin());
}

void Go1RoughModel::SetBaseLinVel(float x, float y, float z)
{
    input_tensor_values_[0] = x;
    input_tensor_values_[1] = y;
    input_tensor_values_[2] = z;
}

void Go1RoughModel::SetBaseAngVel(std::vector<float> base_ang_vel)
{
    if(base_ang_vel.size() != kBaseLinVelSize)
    {
        spdlog::error("Base angular velocity size mismatch: expected {}, got {}", kBaseAngVelSize, base_ang_vel.size());
        return;
    }
    size_t offset = kBaseLinVelSize;
    std::copy(base_ang_vel.begin(), base_ang_vel.end(), input_tensor_values_.begin()+offset);
}

void Go1RoughModel::SetBaseAngVel(float x, float y, float z)
{
    input_tensor_values_[3] = x;
    input_tensor_values_[4] = y;
    input_tensor_values_[5] = z;
}

void Go1RoughModel::SetProjectedGravity(std::vector<float> projected_gravity)
{
    if(projected_gravity.size() != kProjectedGravitySize)
    {
        spdlog::error("Projected gravity size mismatch: expected {}, got {}", kProjectedGravitySize, projected_gravity.size());
        return;
    }
    size_t offset = kBaseLinVelSize + kBaseAngVelSize;
    std::copy(projected_gravity.begin(), projected_gravity.end(), input_tensor_values_.begin()+offset);
}

void Go1RoughModel::SetProjectedGravity(float x, float y, float z)
{
    input_tensor_values_[6] = x;
    input_tensor_values_[7] = y;
    input_tensor_values_[8] = z;
}

void Go1RoughModel::SetVelocityCommands(std::vector<float> velocity_commands)
{
    if(velocity_commands.size() != kVelocityCommandsSize)
    {
        spdlog::error("Velocity commands size mismatch: expected {}, got {}", kVelocityCommandsSize, velocity_commands.size());
        return;
    }
    size_t offset = kBaseLinVelSize + kBaseAngVelSize + kProjectedGravitySize;
    std::copy(velocity_commands.begin(), velocity_commands.end(), input_tensor_values_.begin()+offset);
}

void Go1RoughModel::SetVelocityCommands(float x, float y, float w)
{
    input_tensor_values_[9] = x;
    input_tensor_values_[10] = y;
    input_tensor_values_[11] = w;
}

void Go1RoughModel::SetJointPositions(float FL_hip, float FR_hip, float RL_hip, float RR_hip,
                                 float FL_thigh, float FR_thigh, float RL_thigh, float RR_thigh,
                                 float FL_calf, float FR_calf, float RL_calf, float RR_calf)
{
    input_tensor_values_[12] = FL_hip;
    input_tensor_values_[13] = FR_hip;
    input_tensor_values_[14] = RL_hip;
    input_tensor_values_[15] = RR_hip;
    input_tensor_values_[16] = FL_thigh;
    input_tensor_values_[17] = FR_thigh;
    input_tensor_values_[18] = RL_thigh;
    input_tensor_values_[19] = RR_thigh;
    input_tensor_values_[20] = FL_calf;
    input_tensor_values_[21] = FR_calf;
    input_tensor_values_[22] = RL_calf;
    input_tensor_values_[23] = RR_calf;
}

void Go1RoughModel::SetJointPositions(std::vector<float> joint_positions)
{
    if(joint_positions.size() != kJointPositionsSize)
    {
        spdlog::error("Joint positions size mismatch: expected {}, got {}", kJointPositionsSize, joint_positions.size());
        return;
    }
    const size_t offset = kBaseLinVelSize + kBaseAngVelSize 
                        + kProjectedGravitySize + kVelocityCommandsSize;
    std::copy(joint_positions.begin(), joint_positions.end(), input_tensor_values_.begin() + offset);
}

void Go1RoughModel::SetJointVelocities(float FL_hip, float FR_hip, float RL_hip, float RR_hip,
                                  float FL_thigh, float FR_thigh, float RL_thigh, float RR_thigh,
                                  float FL_calf, float FR_calf, float RL_calf, float RR_calf)
{
    input_tensor_values_[24] = FL_hip;
    input_tensor_values_[25] = FR_hip;
    input_tensor_values_[26] = RL_hip;
    input_tensor_values_[27] = RR_hip;
    input_tensor_values_[28] = FL_thigh;
    input_tensor_values_[29] = FR_thigh;
    input_tensor_values_[30] = RL_thigh;
    input_tensor_values_[31] = RR_thigh;
    input_tensor_values_[32] = FL_calf;
    input_tensor_values_[33] = FR_calf;
    input_tensor_values_[34] = RL_calf;
    input_tensor_values_[35] = RR_calf;
}

void Go1RoughModel::SetJointVelocities(std::vector<float> joint_velocities)
{
    if(joint_velocities.size() != kJointVelocitiesSize)
    {
        spdlog::error("Joint velocities size mismatch: expected {}, got {}", kJointVelocitiesSize, joint_velocities.size());
        return;
    }
    const size_t offset = kBaseLinVelSize + kBaseAngVelSize 
                        + kProjectedGravitySize + kVelocityCommandsSize
                        + kJointPositionsSize;
    std::copy(joint_velocities.begin(), joint_velocities.end(), input_tensor_values_.begin() + offset);
}

void Go1RoughModel::SetActions(float FL_hip, float FR_hip, float RL_hip, float RR_hip,
                          float FL_thigh, float FR_thigh, float RL_thigh, float RR_thigh,
                          float FL_calf, float FR_calf, float RL_calf, float RR_calf)
{
    input_tensor_values_[36] = FL_hip;
    input_tensor_values_[37] = FR_hip;
    input_tensor_values_[38] = RL_hip;
    input_tensor_values_[39] = RR_hip;
    input_tensor_values_[40] = FL_thigh;
    input_tensor_values_[41] = FR_thigh;
    input_tensor_values_[42] = RL_thigh;
    input_tensor_values_[43] = RR_thigh;
    input_tensor_values_[44] = FL_calf;
    input_tensor_values_[45] = FR_calf;
    input_tensor_values_[46] = RL_calf;
    input_tensor_values_[47] = RR_calf;
}

void Go1RoughModel::SetActions(std::vector<float> actions)
{
    if(actions.size() != kActionsSize)
    {
        spdlog::error("Actions size mismatch: expected {}, got {}", kActionsSize, actions.size());
        return;
    }
    const size_t offset = kBaseLinVelSize + kBaseAngVelSize 
                        + kProjectedGravitySize + kVelocityCommandsSize
                        + kJointPositionsSize + kJointVelocitiesSize;
    std::copy(actions.begin(), actions.end(), input_tensor_values_.begin() + offset);
}

void Go1RoughModel::SetHeightScan(std::vector<float> height_scan)
{
    if(height_scan.size() != kHeightScanSize)
    {
        spdlog::error("Height scan size mismatch: expected {}, got {}", kHeightScanSize, height_scan.size());
        return;
    }
    const size_t offset = kBaseLinVelSize + kBaseAngVelSize 
                        + kProjectedGravitySize + kVelocityCommandsSize
                        + kJointPositionsSize + kJointVelocitiesSize
                        + kActionsSize;
    std::copy(height_scan.begin(), height_scan.end(), input_tensor_values_.begin() + offset);
}

// Path: src/model/orbit_base_model.cpp