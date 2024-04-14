#include <iostream>
#include "agent/unitree_sdk_agent.h"

int main()
{
    // std::filesystem::path p(__FILE__);
    // std::string model_path_str = p.parent_path().parent_path().append("models/go1.onnx");
    // const char* model_path = model_path_str.c_str();
    // GO1Model model(model_path);
    // std::vector<float> output(12);
    // model.SetBaseLinVel(-0.5100842714309692, -0.25824564695358276, -0.013611939735710621);
    // model.SetBaseAngVel(0.10476125776767731, -0.11587665975093842, 0.4536033570766449);
    // model.SetProjectedGravity(-0.1191307008266449, 0.09075706452131271, -0.9887220859527588);
    // model.SetVelocityCommands(-0.6055983304977417, -0.29154282808303833, 0.4787849187850952);
    // std::vector<float> joint_positions = {-0.30786681175231934, 0.5687925219535828, -0.23189622163772583, 0.3966677486896515, 0.34024935960769653, -0.007139921188354492, -0.6113666296005249, 0.24414849281311035, -0.2009967565536499, -0.1371523141860962, -0.12941491603851318, -0.2980935573577881};
    // model.SetJointPositions(joint_positions);
    // std::vector<float> joint_velocities = {0.6042282581329346, 0.3100392520427704, -0.254865437746048, 1.3654866218566895, -1.1592261791229248, 2.177489995956421, -0.10627008229494095, -0.6468568444252014, -0.8571549654006958, -0.5732846856117249, -0.17958234250545502, -1.3418805599212646};
    // model.SetJointVelocities(joint_velocities);
    // std::vector<float> actions = {-0.7770276069641113, 2.0051703453063965, -1.150434970855713, 2.092038154602051, 0.39465996623039246, 0.8091681003570557, -1.9126938581466675, -0.015138263814151287, 0.4393902122974396, -0.28557050228118896, -0.21644124388694763, -0.156391441822052};
    // model.SetActions(actions);
    // std::vector<float> height_scan = {-0.20595499873161316, -0.14674898982048035, -0.1616186797618866, -0.1616186797618866, -0.1616186797618866, -0.16161486506462097, -0.1616186797618866, -0.1859888732433319, -0.18599078059196472, -0.1859869658946991, -0.1859888732433319, -0.1859888732433319, -0.1521429717540741, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.20595690608024597, -0.20595499873161316, -0.20327135920524597, -0.20327135920524597, -0.1616186797618866, -0.1616205871105194, -0.1616205871105194, -0.1859888732433319, -0.18599078059196472, -0.1859869658946991, -0.1859888732433319, -0.1521429717540741, -0.1521429717540741, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.20595881342887878, -0.20595881342887878, -0.20326945185661316, -0.20326945185661316, -0.20327135920524597, -0.20327135920524597, -0.19246432185173035, -0.1859888732433319, -0.1859869658946991, -0.1859888732433319, -0.1859869658946991, -0.1521429717540741, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.2059607207775116, -0.20326945185661316, -0.20327135920524597, -0.20326945185661316, -0.20327135920524597, -0.20326945185661316, -0.19246432185173035, -0.19246432185173035, -0.19246432185173035, -0.18599078059196472, -0.1859888732433319, -0.1521429717540741, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.20595499873161316, -0.20326945185661316, -0.20326945185661316, -0.20326945185661316, -0.20327135920524597, -0.20326945185661316, -0.19246432185173035, -0.19246241450309753, -0.19246432185173035, -0.19246241450309753, -0.20971247553825378, -0.2097143828868866, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.15082499384880066, -0.20326945185661316, -0.20327135920524597, -0.20327135920524597, -0.20327135920524597, -0.19246432185173035, -0.19246241450309753, -0.19246432185173035, -0.19246241450309753, -0.19246432185173035, -0.20971247553825378, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.20310541987419128, -0.20310541987419128, -0.20310541987419128, -0.20310541987419128, -0.20326945185661316, -0.19246432185173035, -0.19246241450309753, -0.19246241450309753, -0.19246241450309753, -0.19246432185173035, -0.20971247553825378, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.2031092345714569, -0.2031092345714569, -0.20310541987419128, -0.20310541987419128, -0.2031092345714569, -0.16793963313102722, -0.16794154047966003, -0.19246241450309753, -0.19246432185173035, -0.20971247553825378, -0.2097143828868866, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.20310541987419128, -0.20310541987419128, -0.20310541987419128, -0.2031092345714569, -0.16793963313102722, -0.16793963313102722, -0.16793963313102722, -0.16794154047966003, -0.16794344782829285, -0.15773722529411316, -0.2097143828868866, -0.20971247553825378, -0.20971247553825378, -0.2097143828868866, -0.21199175715446472, -0.21199175715446472, -0.21199175715446472, -0.2031092345714569, -0.2031092345714569, -0.20310541987419128, -0.20310541987419128, -0.16794154047966003, -0.16793963313102722, -0.16793963313102722, -0.16794535517692566, -0.16793963313102722, -0.15773722529411316, -0.15773722529411316, -0.15773722529411316, -0.15773722529411316, -0.15080973505973816, -0.15080973505973816, -0.15080973505973816, -0.15080973505973816, -0.1873926818370819, -0.1873907744884491, -0.20310541987419128, -0.20310541987419128, -0.16794154047966003, -0.16793963313102722, -0.16794154047966003, -0.16793963313102722, -0.15773722529411316, -0.15773722529411316, -0.15773722529411316, -0.15773722529411316, -0.15773722529411316, -0.17490336298942566, -0.17490145564079285, -0.17489954829216003, -0.15080973505973816};
    // output = model.RunModel();
    // std::cout << "Output: ";
    // for (int i = 0; i < 12; i++)
    // {
    //     std::cout << output[i] << " ";
    // }
    // std::cout << std::endl;
    // return 0;

    // std::string current_file_path = __FILE__;
    // size_t last_slash = current_file_path.find_last_of("/");
    // std::string model_path_str = current_file_path.substr(0, last_slash) + "/../models/go1.onnx";
    // const char* model_path = model_path_str.c_str();
    UnitreeSDKAgent agent("go1_flat");
    agent.Run();
    return 0;
}
