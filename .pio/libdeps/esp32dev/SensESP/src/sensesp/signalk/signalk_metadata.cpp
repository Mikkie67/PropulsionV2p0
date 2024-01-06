#include "signalk_metadata.h"

namespace sensesp {

SKMetadata::SKMetadata(String units, String display_name, String description,
             String short_name, float timeout,
             String alertMethod, String warnMethod,             
             const std::array<String, 2>& alarmMethod,
             const std::array<String, 2>& emergencyMethod)
    : units_{units},
      display_name_{display_name},
      description_{description},
      short_name_{short_name},
      timeout_{timeout},
      alertMethod_{alertMethod},
      warnMethod_{warnMethod},
      alarmMethod_{alarmMethod},
      emergencyMethod_{emergencyMethod}{}



void SKMetadata::add_entry(String sk_path, JsonArray& meta) {
  JsonObject json = meta.createNestedObject();
  json["path"] = sk_path;
  JsonObject val = json.createNestedObject("value");

  if (!this->display_name_.isEmpty()) {
    val["displayName"] = this->display_name_;
  }

  if (!this->units_.isEmpty()) {
    val["units"] = this->units_;
  }

  if (!this->description_.isEmpty()) {
    val["description"] = this->description_;
  }

  if (!this->short_name_.isEmpty()) {
    val["shortName"] = this->short_name_;
  }

  if (this->timeout_ >= 0.0) {
    val["timeout"] = this->timeout_;
  }
  if (!this->alertMethod_.isEmpty()) {
    val["alertMethod"] = this->alertMethod_;
  }
  if (!this->warnMethod_.isEmpty()) {
    val["warnMethod"] = this->warnMethod_;
  }
}

}  // namespace sensesp
