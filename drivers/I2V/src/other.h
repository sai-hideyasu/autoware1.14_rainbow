#ifndef IV2_OTHER
#define IV2_OTHER

#include <string>
#include <vector>

void buf_rev(unsigned char* buf, int bytesize);
std::vector<unsigned char> string2vector(const std::string &str);
std::string vector2string(const std::vector<unsigned char> &vec);

#endif