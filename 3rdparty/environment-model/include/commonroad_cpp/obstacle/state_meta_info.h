#pragma once

#include <map>
#include <string>

class StateMetaInfo {
  public:
    StateMetaInfo();

    /**
     * Constructor for StateMetaInfo
     * @param metaDataStr TODO
     * @param meta_data_int
     * @param meta_data_float
     * @param meta_data_bool
     */
    StateMetaInfo(std::map<std::string, std::string> &metaDataStr, std::map<std::string, int> &metaDataInt,
                  std::map<std::string, float> &metaDataFloat, std::map<std::string, bool> &metaDataBool);

  private:
    std::map<std::string, std::string> metaDataStr{};
    std::map<std::string, int> metaDataInt{};
    std::map<std::string, float> metaDataFloat{};
    std::map<std::string, bool> metaDataBool{};
};
