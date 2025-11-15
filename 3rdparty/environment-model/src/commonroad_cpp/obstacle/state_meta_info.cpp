#include "commonroad_cpp/obstacle/state_meta_info.h"

StateMetaInfo::StateMetaInfo() = default;

StateMetaInfo::StateMetaInfo(std::map<std::string, std::string> &metaDataStr, std::map<std::string, int> &metaDataInt,
                             std::map<std::string, float> &metaDataFloat, std::map<std::string, bool> &metaDataBool)
    : metaDataStr(metaDataStr), metaDataInt(metaDataInt), metaDataFloat(metaDataFloat), metaDataBool(metaDataBool) {}
