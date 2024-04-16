#ifndef GO1_FLAT_MODEL_H
#define GO1_FLAT_MODEL_H

#include "model/orbit_base_model.h"

class Go1FlatModel : public OrbitBaseModel
{
public:
    Go1FlatModel();
    ~Go1FlatModel();
    std::vector<float> RunModel();
    void SetBaseLinVel(float x, float y, float z);
    void SetBaseLinVel(std::vector<float> base_lin_vel);
    void SetBaseAngVel(float x, float y, float z);
    void SetBaseAngVel(std::vector<float> base_ang_vel);
    void SetProjectedGravity(float x, float y, float z);
    void SetProjectedGravity(std::vector<float> projected_gravity);
    void SetVelocityCommands(float x, float y, float w);
    void SetVelocityCommands(std::vector<float> velocity_commands);
    void SetJointPositions(float joint_positions[12]);
    void SetJointPositions(float FL_hip, float FR_hip, float RL_hip, float RR_hip,
                           float FL_thigh, float FR_thigh, float RL_thigh, float RR_thigh,
                           float FL_calf, float FR_calf, float RL_calf, float RR_calf);
    void SetJointPositions(std::vector<float> joint_positions);
    void SetJointVelocities(float joint_velocities[12]);
    void SetJointVelocities(float FL_hip, float FR_hip, float RL_hip, float RR_hip,
                            float FL_thigh, float FR_thigh, float RL_thigh, float RR_thigh,
                            float FL_calf, float FR_calf, float RL_calf, float RR_calf);
    void SetJointVelocities(std::vector<float> joint_velocities);
    void SetActions(float actions[12]);
    void SetActions(float FL_hip, float FR_hip, float RL_hip, float RR_hip,
                    float FL_thigh, float FR_thigh, float RL_thigh, float RR_thigh,
                    float FL_calf, float FR_calf, float RL_calf, float RR_calf);
    void SetActions(std::vector<float> actions);
private:
    const size_t kInputSize = 48;
    const size_t kOutputSize = 12;
    const size_t kBaseLinVelSize = 3;
    const size_t kBaseAngVelSize = 3;
    const size_t kProjectedGravitySize = 3;
    const size_t kVelocityCommandsSize = 3;
    const size_t kJointPositionsSize = 12;
    const size_t kJointVelocitiesSize = 12;
    const size_t kActionsSize = 12;
};

namespace ModelType
{
    const std::string GO1_FLAT = "go1_flat";
}

#endif // GO1_FLAT_MODEL_H