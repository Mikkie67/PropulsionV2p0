// signalk_extended_metadata.h
// Renamed from signalk_metadata_zones.h

#ifndef SENSESP_SIGNALK_EXTENDED_METADATA_H_
#define SENSESP_SIGNALK_EXTENDED_METADATA_H_

#include <ArduinoJson.h>
#include "sensesp/signalk/signalk_metadata.h"

namespace sensesp {

/**
 * @brief Extended SKMetadata that includes zone configuration support
 * according to Signal K specification
 */
class signalk_extended_metadata : public SKMetadata {
 public:
  struct Zone {
    float lower = NAN;
    float upper = NAN;
    String state = "normal";
    String message = "";
  };

 public:
  signalk_extended_metadata(const String& units, const String& display_name = "",
                           const String& description = "", 
                           const String& short_name = "", float timeout = -1.0)
      : SKMetadata(units, display_name, description, short_name, timeout) {}

  void add_zone(float lower, float upper, const String& state,
                const String& message = "") {
    Zone zone;
    zone.lower = lower;
    zone.upper = upper;
    zone.state = state;
    zone.message = message;
    zones_.push_back(zone);
  }

  void add_zone(float lower, const String& state, const String& message = "") {
    Zone zone;
    zone.lower = lower;
    zone.state = state;
    zone.message = message;
    zones_.push_back(zone);
  }

  void add_zone(float lower, float upper, const String& state) {
    add_zone(lower, upper, state, "");
  }

  virtual void add_entry(const String& sk_path, JsonArray& meta) override {
    JsonObject meta_entry = meta.add<JsonObject>();
    meta_entry["path"] = sk_path;
    JsonObject value_obj = meta_entry.createNestedObject("value");
    value_obj["units"] = units_;
    value_obj["description"] = description_;
    value_obj["displayName"] = display_name_;
    if (short_name_.length() > 0) value_obj["shortName"] = short_name_;
    if (timeout_ >= 0) value_obj["timeout"] = timeout_;
    // Add zones if present
    if (!zones_.empty()) {
      JsonArray zones_arr = value_obj.createNestedArray("zones");
      for (const auto& zone : zones_) {
        JsonObject z = zones_arr.add<JsonObject>();
        if (!isnan(zone.lower)) z["lower"] = zone.lower;
        if (!isnan(zone.upper)) z["upper"] = zone.upper;
        z["state"] = zone.state;
        if (zone.message.length() > 0) z["message"] = zone.message;
      }
    }
  }

 private:
  std::vector<Zone> zones_;
};

} // namespace sensesp

#endif // SENSESP_SIGNALK_EXTENDED_METADATA_H_
