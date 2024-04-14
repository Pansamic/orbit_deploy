#ifndef ORBIT_BASE_MODEL_H
#define ORBIT_BASE_MODEL_H

#include <vector>
#include <memory>
#include "onnxruntime_cxx_api.h"
#include "cpu_provider_factory.h"

class OrbitBaseModel
{
public:
    class ModelInOutException : public std::exception
    {
    public:
        ModelInOutException(const std::string& message) : message_(message) {}
        ~ModelInOutException() {}
        const char* what() const noexcept override { return message_.c_str(); }
    private:
        std::string message_;
    };
    OrbitBaseModel();
    OrbitBaseModel(const char* model_path);
    ~OrbitBaseModel();
    void LoadModel(const char* model_path);
    void RunModel();
    size_t GetInputSize();
    size_t GetOutputSize();

    std::vector<float> input_tensor_values_;
    std::vector<float> output_tensor_values_;

private:
    std::shared_ptr<Ort::Env> env_;
    std::shared_ptr<Ort::SessionOptions> session_options_;
    std::shared_ptr<Ort::AllocatorWithDefaultOptions> allocator_;
    std::shared_ptr<Ort::MemoryInfo> memory_info_;
    std::shared_ptr<Ort::Session> session_;
    
    std::vector<const char*> input_names_;
    std::string input_name_;
    std::vector<int64_t> input_node_dims_;
    size_t input_size_;
    std::vector<Ort::Value> input_tensors_;
    ONNXTensorElementDataType input_type_;

    std::vector<const char*> output_names_;
    std::string output_name_;
    std::vector<int64_t> output_node_dims_;
    size_t output_size_;
    std::vector<Ort::Value> output_tensors_;
    ONNXTensorElementDataType output_type_;
};

#endif // ORBIT_BASE_MODEL_H