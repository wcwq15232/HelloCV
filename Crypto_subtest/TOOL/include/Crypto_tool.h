#ifndef CRYPTO_TOOL_H
#define CRYPTO_TOOL_H

#include "console_tool.h"
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

using std::string, std::vector, std::map, std::cout, std::cin, std::endl;

enum class Crypto_type { CAESAR, XOR };
enum class Crypto_status { ENCRYPT, DECRYPT };

extern map<char, int> HEX2D;

class Crypto {
private:
    Crypto_status crypto_status;
    
    string caesar(const string& text, const string& key);
    string XOR(const string& text, const string& key);

public:
    Crypto_type crypto_type;
    string encrypt(const string& plaintext, const string& key);
    string decrypt(const string& ciphertext, const string& key);
    void set_type(Crypto_type type);
    void set_status(Crypto_status status);
};

class FileHandler {
private:
    // 程序从文件读入时返回引用以所以要有这玩意
    string content;

public:
    bool success;
    
    FileHandler() : success(false) {}
    string& file_to_string(const string& path);
    void string_to_file(const string& text, const string& path = "output.txt");
};

class Menu {
private:
    Crypto crypto;
    FileHandler fileHandler;
    
    void handleTextEncryption();
    void handleTextDecryption();
    void handleFileEncryption();
    void handleFileDecryption();
    void waitForEnter();
    
public:
    int run();
};

#endif