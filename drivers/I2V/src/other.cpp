#include "other.h"

void buf_rev(unsigned char* buf, int bytesize)
{
	for(int i=0; i<bytesize/2; i++)
	{
		unsigned char tmp = buf[i];
		buf[i] = buf[bytesize-1-i];
		buf[bytesize-1-i] = tmp;
	}
}

/**
 * @fn std::vector<uint8_t> string2vector(const std::string &str)
 * @brief std::stringをstd::vector<uint8_t>に変換
 * @param[in] str 変換したい文字列
 * @return 変換したstd::vector<uint8_t>の配列
 * @details
*/
std::vector<unsigned char> string2vector(const std::string &str)
{
	std::vector<unsigned char> ret;
	for(int i=0; i<str.size(); i++) ret.emplace_back(str[i]);
	return ret;
}

/**
 * @fn std::string vector2string(const std::vector<uint8_t> &vec)
 * @brief std::vector<uint8_t>をstd::stringに変換
 * @param[in] vec 変換したいstd::vector<uint8_t>の配列
 * @return 変換した文字列
 * @details　
*/
std::string vector2string(const std::vector<unsigned char> &vec)
{
	unsigned char *data = new char[vec.size()+1];
	for(int i=0;i<vec.size();i++) data[i] = vec[i];
	data[vec.size()] = '\0';
	std::string ret((char*)data);
	delete[] data;
	return ret;
}