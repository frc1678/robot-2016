#ifndef CITRUS_SOCKET_NETWORK_SERIALIZE_H_
#define CITRUS_SOCKET_NETWORK_SERIALIZE_H_

#include <sstream>
#include <cstdint>
#include <arpa/inet.h>
#include <iostream>
#include "json.hpp"

class SerializedData {
 public:
  SerializedData() {}
  SerializedData(SerializedData&& to_move) : out(to_move.out.str()) {}
  std::stringstream out;
  const char* GetData() const { return out.str().c_str(); } uint32_t GetLength() const { return out.str().length(); }
};

SerializedData& operator<<(SerializedData& lhs, std::string rhs);
SerializedData& operator<<(SerializedData& lhs, char rhs);
SerializedData& operator<<(SerializedData& lhs, uint16_t rhs);
SerializedData& operator<<(SerializedData& lhs, uint32_t rhs);
SerializedData& operator<<(SerializedData& lhs, nlohmann::json& rhs);

SerializedData& operator>>(SerializedData& lhs, std::string& rhs);
SerializedData& operator>>(SerializedData& lhs, char& rhs);
SerializedData& operator>>(SerializedData& lhs, uint16_t& rhs);
SerializedData& operator>>(SerializedData& lhs, uint32_t& rhs);

std::string from_json(const nlohmann::json& json);

nlohmann::json to_json(const SerializedData& from);

#endif /* CITRUS_SOCKET_NETWORK_SERIALIZE_H_ */
