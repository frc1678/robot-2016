cc_library(
  name = "citrus_socket",
  srcs = ["citrus_socket.cpp", "destination.cpp", "network_serialize.cpp"],
  hdrs = ["citrus_socket.h", "destination.h", "network_serialize.h"],
  deps = ["@json//:main"],
  visibility = ["//visibility:public"]
)

cc_test(
  name = "socket_test",
  srcs = ["socket_test.cpp"],
  deps = ["@gtest//:main", ":citrus_socket", "@json//:main"],
  size = "small"
)
