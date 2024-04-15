#ifndef GO1_AGENT_H
#define GO1_AGENT_H

#include <vector>
#include <memory>
#include <string>
#include <exception>

#include "model/go1_rough_model.h"
#include "model/go1_flat_model.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

class Go1Agent
{
public:

    Go1Agent(std::string model_type);
    ~Go1Agent();
    void Run();

private:
    class Go1AgentException : public std::exception
    {
    public:
        Go1AgentException(const std::string& message) : message_(message) {}
        const char* what() const noexcept override { return message_.c_str(); }
    private:
        std::string message_;
    };

    std::shared_ptr<Go1RoughModel> go1_rough_model_;
    std::shared_ptr<Go1FlatModel> go1_flat_model_;
    std::string model_type_;

    UNITREE_LEGGED_SDK::Safety safety_;
    UNITREE_LEGGED_SDK::UDP udp_;
    UNITREE_LEGGED_SDK::LowCmd cmd_;
    UNITREE_LEGGED_SDK::LowState state_;

    std::vector<float> base_lin_vel_;
    std::vector<float> base_ang_vel_;
    std::vector<float> projected_gravity_;
    std::vector<float> pose_; // eular angles
    std::vector<float> actions_;
    std::vector<float> last_actions_;
    std::vector<float> joint_positions_;
    std::vector<float> joint_velocities_;
    std::vector<float> height_scan_;

    /**s
     * order: FL_hip,   FR_hip,   RL_hip,   RR_hip,
     *        FL_thigh, FR_thigh, RL_thigh, RR_thigh,
     *        FL_calf,  FR_calf,  RL_calf,  RR_calf
    */
    const float kStandAngles[12] = { 0.1, -0.1,  0.1, -0.1,
                                     0.8,  0.8,  1.0,  1.0,
                                    -1.5, -1.5, -1.5, -1.5};
    const float kProneAngles[12] = { 0.0 ,  0.0 ,  0.0 ,  0.0,
                                     1.3 ,  1.3 ,  1.3 ,  1.3,
                                    -2.78, -2.78, -2.78, -2.78};
    const float kActionScale = 0.25f;
    const float kHipScaleReduction = 0.5f;
    const float kMotorKp = 20.0f;
    const float kMotorKd = 0.5f;
    const float kMotorAmount = 12;
    const float kLoopFuncInterval = 0.002f;
    
    void Go1RoughRun();
    void Go1FlatRun();
    virtual void Go1RoughGetObs();
    virtual void Go1FlatGetObs();
    void Go1CalibrateStand();
    void Go1CalibrateProne();
    void Go1ProcessActions(std::vector<float>& actions);
    void Go1ProcessActions(float actions[12]);
    void Go1ExecuteActions(std::vector<float>& actions);
    void Go1ExecuteActions(float actions[12]);
    void SdkReceive();
    void SdkSend();
};

namespace ModelType
{
    const std::string GO1_ROUGH = "go1_rough";
    const std::string GO1_FLAT = "go1_flat";
}


#endif // GO1_AGENT_H