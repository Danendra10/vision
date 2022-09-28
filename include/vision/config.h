#ifndef CONFIG_H_
#define CONFIG_H_

#include <ros/package.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <stack>
#include <stdint.h>

#include <ros/param.h>
class Config
{
private:
    std::string robot_name;
    std::string CONFIG_FILE_PATH;

    YAML::Node node_parser_;
    YAML::Emitter emitter_;

    std::stack<YAML::Node> node_stack_;
public:
    Config();
    ~Config();

    void load(std::string path);
    void save(std::string path);

    void parseMapBegin(std::string key) { node_stack_.push(node_stack_.top()[key]); }
    void parseMapEnd() { node_stack_.pop(); }

    void parseKeyValue(std::string key, std::string *value) { *value = node_stack_.top()[key].as<std::string>(); }
    void parseKeyValue(std::string key, bool *value) { *value = node_stack_.top()[key].as<bool>(); }
    void parseKeyValue(std::string key, uint8_t *value) { *value = node_stack_.top()[key].as<uint8_t>(); }
    void parseKeyValue(std::string key, uint16_t *value) { *value = node_stack_.top()[key].as<uint16_t>(); }
    void parseKeyValue(std::string key, short int *value) { *value = node_stack_.top()[key].as<short int>(); }
    void parseKeyValue(std::string key, int *value) { *value = node_stack_.top()[key].as<int>(); }
    void parseKeyValue(std::string key, float *value) { *value = node_stack_.top()[key].as<float>(); }
    void parseKeyValue(std::string key, double *value) { *value = node_stack_.top()[key].as<double>(); }

    void emitMapBegin(std::string key) { emitter_ << YAML::Key << key << YAML::Value << YAML::BeginMap; }
    void emitMapEnd() { emitter_ << YAML::EndMap << YAML::Newline << YAML::Newline; }

    void emitKeyValue(std::string key, std::string value) { emitter_ << YAML::Key << key << YAML::Value << value; }
    void emitKeyValue(std::string key, bool value) { emitter_ << YAML::Key << key << YAML::Value << value; }
    void emitKeyValue(std::string key, uint8_t value) { emitter_ << YAML::Key << key << YAML::Value << (int)value; }
    void emitKeyValue(std::string key, uint16_t value) { emitter_ << YAML::Key << key << YAML::Value << (int)value; }
    void emitKeyValue(std::string key, short int value) { emitter_ << YAML::Key << key << YAML::Value << (int)value; }
    void emitKeyValue(std::string key, int value) { emitter_ << YAML::Key << key << YAML::Value << value; }
    void emitKeyValue(std::string key, float value) { emitter_ << YAML::Key << key << YAML::Value << value; }
    void emitKeyValue(std::string key, double value) { emitter_ << YAML::Key << key << YAML::Value << value; }

    //----Enum For Cfg Status
    //======================
    enum {
        STATUS_THRESH_FIELD = 0,
        STATUS_THRESH_BALL = 1,
        STATUS_THRESH_LINE = 2,
    };
};

#endif