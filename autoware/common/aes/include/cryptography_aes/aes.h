#ifndef OPENSSL_AES
#define OPENSSL_AES

#include <openssl/evp.h>
#include <iostream>
#include <vector>

namespace cryptography_aes
{

class AES
{
public:
	static const int OK = 0;
	static const int ERROR_EVP_CIPHER_CTX = 1;
	static const int ERROR_EVP_AES_128_CBC = 2;
	static const int ERROR_EVP_ENCRYPTINIT = 3;
	static const int ERROR_EVP_DECRYPTINIT = 4;
	static const int ERROR_EVP_ENCRYPTUPDATE = 5;
	static const int ERROR_EVP_DECRYPTUPDATE = 6;
	static const int ERROR_EVP_ENCRYPTFIANL = 7;
	static const int ERROR_EVP_DECRYPTFIANL = 8;
private:
	const unsigned int BLOCK_SIZE = 16;
	const unsigned int KEY_SIZE = 128;

	std::vector<uint8_t> key_;
	std::vector<uint8_t> constract_vec_;
public:
	AES(const std::vector<uint8_t> &key, const std::vector<uint8_t> &constract_vec);

	int encode(const std::vector<uint8_t> &raw_data, std::vector<uint8_t> &out);

	int decode(const std::vector<uint8_t> &enc_data, std::vector<uint8_t> &out);
};

}
#endif
