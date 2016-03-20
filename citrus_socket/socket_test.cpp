#include "citrus_socket.h"
#include "gtest/gtest.h"
#include <iostream>
#include <unistd.h>
#include "destination.h"

TEST(Socket, CorrectlyBinds) {
  // When a valid port is passed in, it should bind correctly
  CitrusSocket specified_port_socket(1678);
  EXPECT_EQ(specified_port_socket.GetPort(), 1678);

  // When zero is passed in, an arbitrary port will be assigned by the system
  CitrusSocket unspecified_port_socket(0);
  EXPECT_NE(unspecified_port_socket.GetPort(), 0);

  // The bind should fail when given low numbers as arguments
  bool bind_failed;
  try {
    CitrusSocket occupied_address_socket(1);
  } catch (const std::string &e) {
    if (e == "Bind failed") {
      bind_failed = true;
    }
  }
  EXPECT_TRUE(bind_failed);
}

TEST(Destination, GetsAddress) {
  Destination d("localhost", 1678);
  EXPECT_EQ(d.GetIpAddressString(), "127.0.0.1");
}

TEST(Socket, SendsAndReceives) {
  CitrusSocket sender(2056);
  CitrusSocket receiver(1678);
  SerializedData data;
  sender.Send("Test", Destination("localhost", 1678));
  EXPECT_EQ(receiver.ReceiveString(), "Test");
}

TEST(Socket, NonBlocking) {
  CitrusSocket sender(2056);
  CitrusSocket receiver(1678);
  EXPECT_EQ(receiver.ReceiveString(), "");
  sender.Send("Test", Destination("localhost", 1678));
  EXPECT_EQ(receiver.ReceiveString(), "Test");
}

TEST(SerializedData, TransfersRawValues) {
  CitrusSocket sender(2056), receiver(1678);
  SerializedData data;
  data << "Test" << 'a' << (uint32_t)24 << (uint32_t)48;
  sender.Send(data, Destination("localhost", 1678));
  auto received = receiver.Receive();
  std::string s;
  char a;
  uint32_t b, c;
  received >> s >> a >> b >> c;
  EXPECT_EQ(s, "Test");
  EXPECT_EQ(a, 'a');
  EXPECT_EQ(b, 24);
  EXPECT_EQ(c, 48);
}

TEST(SerializedData, TransfersJson) {
  CitrusSocket sender(2056), receiver(1678);
  nlohmann::json send_json = {{"chezy", "pofz"},
                              {"citrus", 1678},
                              {"list", {{"item", 123.45}, {"item2", false}}}};
  sender.Send(from_json(send_json), Destination("localhost", 1678));
  SerializedData send;
  auto recv_json = to_json(receiver.Receive());

  EXPECT_EQ(recv_json["chezy"], "pofz");
  EXPECT_EQ(recv_json["citrus"], 1678);
  EXPECT_EQ(recv_json["list"]["item"], 123.45);
  EXPECT_EQ(recv_json["list"]["item2"], false);
}
