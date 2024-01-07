#include "signalk_metadata.h"

namespace sensesp {

SKMetadata::SKMetadata(String units, String display_name, String description,
                       String short_name, float timeout,
                       const std::array<String, 2> alertMethod,
                       const std::array<String, 2> warnMethod,
                       const std::array<String, 2> alarmMethod,
                       const std::array<String, 2> emergencyMethod)
    : units_{units},
      display_name_{display_name},
      description_{description},
      short_name_{short_name},
      timeout_{timeout},
      alertMethod_{alertMethod},
      warnMethod_{warnMethod},
      alarmMethod_{alarmMethod},
      emergencyMethod_{emergencyMethod} {}

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
  // the rules:
  // 1. if both are empty, do nothing
  // 2. if both are not empty, create a jsonArray
  // 3. otherwise, fine the non-empty one and add a normal entry to meta

  if ((!this->alertMethod_[0].isEmpty() && !this->alertMethod_[1].isEmpty())) {
    JsonArray alertMethodArray = val.createNestedArray("alertMethod");
    alertMethodArray.add(this->alertMethod_[0]);
    alertMethodArray.add(this->alertMethod_[1]);
  } else if (!this->alertMethod_[0].isEmpty()) {
    val["alertMethod"] = this->alertMethod_[0];
  } else if (!this->alertMethod_[1].isEmpty()) {
    val["alertMethod"] = this->alertMethod_[1];
  }

  if ((!this->warnMethod_[0].isEmpty() && !this->warnMethod_[1].isEmpty())) {
    JsonArray warnMethodArray = val.createNestedArray("warnMethod");
    warnMethodArray.add(this->warnMethod_[0]);
    warnMethodArray.add(this->warnMethod_[1]);
  } else if (!this->warnMethod_[0].isEmpty()) {
    val["warnMethod"] = this->warnMethod_[0];
  } else if (!this->warnMethod_[1].isEmpty()) {
    val["warnMethod"] = this->warnMethod_[1];
  }

  if ((!this->alarmMethod_[0].isEmpty() && !this->alarmMethod_[1].isEmpty())) {
    JsonArray alarmMethodArray = val.createNestedArray("alarmMethod");
    alarmMethodArray.add(this->alarmMethod_[0]);
    alarmMethodArray.add(this->alarmMethod_[1]);
  } else if (!this->alarmMethod_[0].isEmpty()) {
    val["alarmMethod"] = this->alarmMethod_[0];
  } else if (!this->alarmMethod_[1].isEmpty()) {
    val["alarmMethod"] = this->alarmMethod_[1];
  }

  if ((!this->emergencyMethod_[0].isEmpty() && !this->emergencyMethod_[1].isEmpty())) {
    JsonArray emergencyMethodArray = val.createNestedArray("emergencyMethod");
    emergencyMethodArray.add(this->emergencyMethod_[0]);
    emergencyMethodArray.add(this->emergencyMethod_[1]);
  } else if (!this->emergencyMethod_[0].isEmpty()) {
    val["emergencyMethod"] = this->emergencyMethod_[0];
  } else if (!this->emergencyMethod_[1].isEmpty()) {
    val["emergencyMethod"] = this->emergencyMethod_[1];
  }
}

}  // namespace sensesp
