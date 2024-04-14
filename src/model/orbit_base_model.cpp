#include <iostream>
#include "model/orbit_base_model.h"

OrbitBaseModel::OrbitBaseModel()
{
    env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "test");
    session_options_ = std::make_shared<Ort::SessionOptions>();
    memory_info_ = std::make_shared<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
    allocator_ = std::make_shared<Ort::AllocatorWithDefaultOptions>();
}

OrbitBaseModel::OrbitBaseModel(const char* model_path)
{
    env_ = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "test");
    session_options_ = std::make_shared<Ort::SessionOptions>();
    memory_info_ = std::make_shared<Ort::MemoryInfo>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
    allocator_ = std::make_shared<Ort::AllocatorWithDefaultOptions>();
    LoadModel(model_path);
}

OrbitBaseModel::~OrbitBaseModel()
{

}

void OrbitBaseModel::LoadModel(const char* model_path)
{
    // initialize onnx model
    session_ = std::make_shared<Ort::Session>(*env_, model_path, *session_options_);

    /***************************************************************/
    /**************               INPUT               **************/
    /***************************************************************/
    auto input_node_name = session_->GetInputNameAllocated(0, *allocator_);
    const char* input_name = input_node_name.get();
    input_name_ = input_name;
    input_names_.push_back(input_name_.c_str());
    Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    input_node_dims_ = input_tensor_info.GetShape();
    input_type_ = input_tensor_info.GetElementType();
    input_size_ = input_node_dims_[0] * input_node_dims_[1];
    input_tensor_values_.resize(input_size_);
    input_tensors_.push_back(Ort::Value::CreateTensor<float>(
        *memory_info_,
        input_tensor_values_.data(), input_size_,
        input_node_dims_.data(), input_node_dims_.size()
    ));

    std::cout << "input_name: " << input_name << std::endl;
    std::cout << "input_type: " << input_type_ << std::endl; 
    std::cout << "input_dimension: " << input_node_dims_[0] << ", " << input_node_dims_[1] << std::endl;
    std::cout << "input_size: " << input_size_ << std::endl;

    /***************************************************************/
    /**************              OUTPUT               **************/
    /***************************************************************/
    auto output_node_name = session_->GetOutputNameAllocated(0, *allocator_);
    const char* output_name = output_node_name.get();
    output_name_ = output_name;
    output_names_.push_back(output_name_.c_str());
    Ort::TypeInfo output_type_info = session_->GetOutputTypeInfo(0);
    auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
    output_node_dims_ = output_tensor_info.GetShape();
    output_type_ = output_tensor_info.GetElementType();
    output_size_ = output_node_dims_[0] * output_node_dims_[1];
    output_tensor_values_.resize(output_size_);
    output_tensors_.push_back(Ort::Value::CreateTensor<float>(
        *memory_info_,
        output_tensor_values_.data(), output_size_,
        output_node_dims_.data(), output_node_dims_.size()
    ));

    std::cout << "output_name: " << output_name << std::endl;
    std::cout << "output_type: " << output_type_ << std::endl;
    std::cout << "output_dimension: " << output_node_dims_[0] << ", " << output_node_dims_[1] << std::endl;
    std::cout << "output_size: " << output_size_ << std::endl;
}

void OrbitBaseModel::RunModel()
{
    session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(), input_tensors_.data(), 1,
        output_names_.data(), output_tensors_.data(), 1
    );
}

size_t OrbitBaseModel::GetInputSize()
{
    return input_size_;
}

size_t OrbitBaseModel::GetOutputSize()
{
    return output_size_;
}

