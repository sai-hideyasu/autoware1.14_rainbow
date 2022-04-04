#include <cryptography_aes/aes.h>

namespace cryptography_aes
{

int AES::encode(const std::vector<uint8_t> &raw_data, std::vector<uint8_t> &out)
{
	EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
	if(ctx == nullptr)
	{
		std::cout << "error:ctx";
		return ERROR_EVP_CIPHER_CTX;
	}

	const EVP_CIPHER* cipher_suite = EVP_aes_128_cbc();
	if(cipher_suite == nullptr)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:cipher_suite";
		return ERROR_EVP_AES_128_CBC;
	}

	//const uint8_t iv[16] = {};
	//if(EVP_EncryptInit_ex(ctx, cipher_suite, nullptr, key_.data(), iv) == 0)]
	if(EVP_EncryptInit_ex(ctx, cipher_suite, nullptr, key_.data(), constract_vec_.data()) == 0)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:EVP_EncryptInit_ex";
		return ERROR_EVP_ENCRYPTINIT;
	}

	std::vector<uint8_t> enc_data = std::vector<uint8_t>(raw_data.size() + (BLOCK_SIZE - raw_data.size()) % BLOCK_SIZE);
	int len = 0;
	if(EVP_EncryptUpdate(ctx, enc_data.data(), &len, raw_data.data(), raw_data.size()) == 0)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:EVP_EncryptUpdate";
		return ERROR_EVP_ENCRYPTUPDATE;
	}

	if(EVP_EncryptFinal(ctx, enc_data.data() + len, &len) == 0)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:EVP_EncryptFinal";
		return ERROR_EVP_ENCRYPTFIANL;
	}
	EVP_CIPHER_CTX_free(ctx);

	out = enc_data;
	return OK;
}

int AES::decode(const std::vector<uint8_t> &enc_data, std::vector<uint8_t> &out)
{
	EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
	if(ctx == nullptr)
	{
		std::cout << "error:ctx";
		return ERROR_EVP_CIPHER_CTX;
	}

	const EVP_CIPHER* cipher_suite = EVP_aes_128_cbc();
	if(cipher_suite == nullptr)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:cipher_suite";
		return ERROR_EVP_AES_128_CBC;
	}

	//const uint8_t iv[16] = {};
	//if(EVP_DecryptInit_ex(ctx, cipher_suite, NULL, key_.data(), iv) == 0)
	if(EVP_DecryptInit_ex(ctx, cipher_suite, NULL, key_.data(), constract_vec_.data()) == 0)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:EVP_DecryptInit_ex";
		return ERROR_EVP_DECRYPTINIT;
	}

	std::vector<uint8_t> plain_data(enc_data.size());
	int plain_data_len = 0;
	if(EVP_DecryptUpdate(ctx, plain_data.data(), &plain_data_len, enc_data.data(), enc_data.size()) == 0)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:EVP_DEcryptUpdate";
		return ERROR_EVP_ENCRYPTUPDATE;
	}

	int remain_plain_data_len;
	if (EVP_DecryptFinal(ctx, plain_data.data() + plain_data_len, &remain_plain_data_len) == 0)
	{
		EVP_CIPHER_CTX_cleanup(ctx);
		std::cout << "error:EVP_DecryptFinal";
		return ERROR_EVP_DECRYPTFIANL;
	}

	plain_data.resize(plain_data_len + remain_plain_data_len);
	EVP_CIPHER_CTX_free(ctx);
	out = plain_data;
	return OK;
}

AES::AES(const std::vector<uint8_t> &key, const std::vector<uint8_t> &constract_vec)
	: key_(key)
	, constract_vec_(constract_vec)
{
}

}