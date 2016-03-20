#include "network_serialize.h"

SerializedData& operator<<(SerializedData& lhs, std::string rhs) {
  lhs.out << rhs << '\0';
  return lhs;
}

SerializedData& operator<<(SerializedData& lhs, char rhs) {
  lhs.out << (char)rhs;
  return lhs;
}

SerializedData& operator<<(SerializedData& lhs, uint16_t rhs) {
  auto network_endian = htons(rhs);
  lhs.out << ((char*)(&network_endian))[0] << ((char*)(&network_endian))[1];
  return lhs;
}

SerializedData& operator<<(SerializedData& lhs, uint32_t rhs) {
  auto network_endian = htonl(rhs);
  lhs.out << ((char*)(&network_endian))[0] << ((char*)(&network_endian))[1]
          << ((char*)(&network_endian))[2] << ((char*)(&network_endian))[3];
  return lhs;
}

SerializedData& operator>>(SerializedData& lhs, std::string& rhs) {
  std::getline(lhs.out, rhs, '\0');
  return lhs;
}

SerializedData& operator>>(SerializedData& lhs, char& rhs) {
  lhs.out >> rhs;
  return lhs;
}

SerializedData& operator>>(SerializedData& lhs, uint16_t& rhs) {
  auto network_endian = htons(rhs);
  lhs.out >> ((char*)(&network_endian))[0] >> ((char*)(&network_endian))[1];
  return lhs;
}

SerializedData& operator>>(SerializedData& lhs, uint32_t& rhs) {
  lhs.out >> ((char*)(&rhs))[0] >> ((char*)(&rhs))[1] >> ((char*)(&rhs))[2] >>
      ((char*)(&rhs))[3];
  rhs = ntohl(rhs);
  return lhs;
}

std::string from_json(const nlohmann::json& json) { return json.dump(); }

nlohmann::json to_json(const SerializedData& from) {
  return nlohmann::json::parse(from.out.str());
}
