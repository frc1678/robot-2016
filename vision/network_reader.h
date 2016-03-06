#ifndef _IMAGE_READER_H_
#define _IMAGE_READER_H_

#include <string>

namespace vision {
std::string readFromNetwork(std::string URL, int port, std::string filename);
std::string getResponseBody(std::string reply);
}

#endif
