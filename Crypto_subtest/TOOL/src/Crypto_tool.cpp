#include "../include/Crypto_tool.h"
#include "../include/console_tool.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <map>

using std::string, std::vector, std::map, std::cout, std::cin, std::endl;

map<char, int> HEX2D = {
    {'0', 0}, {'1', 1}, {'2', 2}, {'3', 3}, {'4', 4},
    {'5', 5}, {'6', 6}, {'7', 7}, {'8', 8}, {'9', 9},
    {'A', 10}, {'B', 11}, {'C', 12}, {'D', 13}, {'E', 14}, {'F', 15},
    {'a', 10}, {'b', 11}, {'c', 12}, {'d', 13}, {'e', 14}, {'f', 15}
};
map<int, char> D2HEX = {
    {0, '0'}, {1, '1'}, {2, '2'}, {3, '3'}, {4, '4'},
    {5, '5'}, {6, '6'}, {7, '7'}, {8, '8'}, {9, '9'},
    {10, 'A'}, {11, 'B'}, {12, 'C'}, {13, 'D'}, {14, 'E'}, {15, 'F'}
};

union CharSplit {
    unsigned char value;
    struct {
        unsigned char low4 : 4;   // 低4位
        unsigned char high4 : 4;  // 高4位
    } bits;
};

string HEX2S(string hex_s){
    if (hex_s.size() % 2 != 0) {
        Console::error("输入应为偶数个十六进制字符 (0-9, A-F, a-f)");
        return "FAILED";
    }
    string result;
    for (int i = 0; i < hex_s.size(); i += 2){
        CharSplit cs;
        cs.bits.high4 = static_cast<u_char>(HEX2D[hex_s[i]]);
        cs.bits.low4 = static_cast<u_char>(HEX2D[hex_s[i + 1]]);
        result += static_cast<char>(cs.value);
    }
    return result;
}

string S2HEX(string str){
    string result;
    result.reserve(str.size() * 2);

    for (char c: str){
        CharSplit cs = {static_cast<u_char>(c)};
        result += D2HEX[cs.bits.high4];
        result += D2HEX[cs.bits.low4];  
    }
    
    return result;
}

string Crypto::encrypt(const string& plaintext, const string& key) {
    set_status(Crypto_status::ENCRYPT);
    if (crypto_type == Crypto_type::CAESAR)
        return caesar(plaintext, key);
    else if (crypto_type == Crypto_type::XOR)
        return XOR(plaintext, key);
    else 
        return "";
}

string Crypto::decrypt(const string& ciphertext, const string& key) {
    set_status(Crypto_status::DECRYPT);
    if (crypto_type == Crypto_type::CAESAR)
        return caesar(ciphertext, key);
    else if (crypto_type == Crypto_type::XOR)
        return XOR(ciphertext, key);
    else 
        return "";
}

void Crypto::set_type(Crypto_type type) { crypto_type = type; }
void Crypto::set_status(Crypto_status status) { crypto_status = status; }

string Crypto::caesar(const string& text, const string& key) {
    string result;
    int offset = 0;

    if (key.empty()) {
        Console::error("密钥不可为空");
        return "FAILED";
    }
    
    for (char i : key) {
        if (i < '0' || i > '9') {
            Console::error("凯撒密码密钥必须为数字");
            return "FAILED";
        }
        offset = offset * 10 + i - '0';
    }

    if (offset < 1 || offset > 25) {
        Console::error("凯撒密码密钥范围应为 [1, 25]");
        return "FAILED";
    }

    if (crypto_status == Crypto_status::DECRYPT) {
        offset = 26 - offset;
    }

    result.reserve(text.size());
    for (char c : text) {
        if (c >= 'A' && c <= 'Z')
            result += (char)((c - 'A' + offset) % 26 + 'A');
        else if (c >= 'a' && c <= 'z')
            result += (char)((c - 'a' + offset) % 26 + 'a');
        else
            result += c;
    }
    return result;
}

string Crypto::XOR(const string& text, const string& key) {
    string result;
    
    if (key.empty()) {
        Console::error("密钥不可为空");
        return "FAILED";
    }
    
    if (key.size() % 2 != 0) {
        Console::error("密钥应为偶数个十六进制字符 (0-9, A-F, a-f)");
        return "FAILED";
    }

    vector<unsigned char> keys;
    keys.reserve(key.size() / 2);
    
    for (size_t i = 0; i < key.size(); i += 2) {
        char high = key[i];
        char low = key[i + 1];
        
        if (HEX2D.find(high) == HEX2D.end() || HEX2D.find(low) == HEX2D.end()) {
            Console::error("密钥包含非十六进制字符");
            return "FAILED";
        }
        
        keys.push_back(static_cast<unsigned char>(HEX2D[high] * 16 + HEX2D[low]));
    }

    result.reserve(text.size());
    for (size_t i = 0; i < text.size(); ++i) {
        result += static_cast<char>(text[i] ^ keys[i % keys.size()]);
    }
    
    return result;
}


void Menu::handleTextEncryption() {
    Console::title("文本加密");
    bool change_i = false;
    bool change_o = false;
    string plaintext, key;

    if (crypto.crypto_type == Crypto_type::XOR){
        while (true){
            string chosen;
            Console::prompt("请选择输入类型");
            Console::menuItem("0", "普通文本");
            Console::menuItem("1", "16进制字串");
            if (cin.peek() == '\n') cin.ignore();
            std::getline(cin, chosen);
            if (chosen == "0"){
                break;
            } else if (chosen == "1") {
                change_i = true;
                break;
            }
        }
    }

    Console::prompt("请输入明文");
    if (cin.peek() == '\n') cin.ignore();
    std::getline(cin, plaintext);
    
    if (change_i){
        if (plaintext.size() % 2 != 0) {
            Console::error("输入应为偶数个十六进制字符 (0-9, A-F, a-f)");
            return;
        }
        plaintext = HEX2S(plaintext);
    }

    Console::prompt("请输入密钥");
    std::getline(cin, key);
    
    string ciphertext = crypto.encrypt(plaintext, key);
    if (ciphertext != "FAILED") {
        Console::success("加密成功！");

        if (crypto.crypto_type == Crypto_type::XOR){
            while (true){
                string chosen;
                Console::prompt("请选择输出类型");
                Console::menuItem("0", "普通文本");
                Console::menuItem("1", "16进制字串");
                if (cin.peek() == '\n') cin.ignore();
                std::getline(cin, chosen);
                if (chosen == "0"){
                    break;
                } else if (chosen == "1") {
                    change_o = true;
                    break;
                }
            }
        }
        if (change_o){
            ciphertext = S2HEX(ciphertext);
        }
        Console::result("密文", ciphertext);
    }
}



void Menu::handleTextDecryption() {
    Console::title("文本解密");
    bool change_i = false;
    bool change_o = false;
    string ciphertext, key;

    if (crypto.crypto_type == Crypto_type::XOR){
        while (true){
            string chosen;
            Console::prompt("请选择输入类型");
            Console::menuItem("0", "普通文本");
            Console::menuItem("1", "16进制字串");
            if (cin.peek() == '\n') cin.ignore();
            std::getline(cin, chosen);
            if (chosen == "0"){
                break;
            } else if (chosen == "1") {
                change_i = true;
                break;
            }
        }
    }

    Console::prompt("请输入密文");
    if (cin.peek() == '\n') cin.ignore();
    std::getline(cin, ciphertext);
    
    if (change_i){
        if (ciphertext.size() % 2 != 0) {
            Console::error("输入应为偶数个十六进制字符 (0-9, A-F, a-f)");
            return;
        }
        ciphertext = HEX2S(ciphertext);
    }

    Console::prompt("请输入密钥");
    std::getline(cin, key);
    
    string plaintext = crypto.decrypt(ciphertext, key);
    if (plaintext != "FAILED") {
        Console::success("解密成功！");

        if (crypto.crypto_type == Crypto_type::XOR){
            while (true){
                string chosen;
                Console::prompt("请选择输出类型");
                Console::menuItem("0", "普通文本");
                Console::menuItem("1", "16进制字串");
                if (cin.peek() == '\n') cin.ignore();
                std::getline(cin, chosen);
                if (chosen == "0"){
                    break;
                } else if (chosen == "1") {
                    change_o = true;
                    break;
                }
            }
        }
        if (change_o){
            plaintext = S2HEX(plaintext);
        }
        Console::result("明文", plaintext);
    }
}

void Menu::handleFileEncryption() {
    Console::title("文件加密");
    
    string inpath, outpath, key;
    Console::prompt("请输入待加密文件路径");
    if (cin.peek() == '\n') cin.ignore();;
    std::getline(cin, inpath);
    
    Console::prompt("请输入输出文件路径");
    if (cin.peek() == '\n') cin.ignore();;
    std::getline(cin, outpath);
    
    Console::prompt("请输入密钥");
    if (cin.peek() == '\n') cin.ignore();;
    std::getline(cin, key);
    
    string plaintext = fileHandler.file_to_string(inpath);
    if (!fileHandler.success) return;
    
    Console::info("文件读取成功，正在进行加密...");
    string ciphertext = crypto.encrypt(plaintext, key);
    
    if (ciphertext != "FAILED") {
        fileHandler.string_to_file(ciphertext, outpath);
        if (fileHandler.success) {
            Console::success("文件加密完成！");
        }
    }
}

void Menu::handleFileDecryption() {
    Console::title("文件解密");
    
    string inpath, outpath, key;
    Console::prompt("请输入待解密文件路径");
    if (cin.peek() == '\n') cin.ignore();;
    std::getline(cin, inpath);
    
    Console::prompt("请输入输出文件路径");
    if (cin.peek() == '\n') cin.ignore();;
    std::getline(cin, outpath);
    
    Console::prompt("请输入密钥");
    if (cin.peek() == '\n') cin.ignore();;
    std::getline(cin, key);
    
    string ciphertext = fileHandler.file_to_string(inpath);
    if (!fileHandler.success) return;
    
    Console::info("文件读取成功，正在进行解密...");
    string plaintext = crypto.decrypt(ciphertext, key);
    
    if (plaintext != "FAILED") {
        fileHandler.string_to_file(plaintext, outpath);
        if (fileHandler.success) {
            Console::success("文件解密完成！");
        }
    }
}

void Menu::waitForEnter() {
    cout << endl;
    Console::prompt("按回车键继续...");
    if (cin.peek() == '\n') cin.ignore();;
    Console::clear();
}

int Menu::run() {
    string chosen;
    
    while (true) {
        Console::separator();
        Console::title("主菜单 - 选择加密算法");
        Console::menuItem("0", "凯撒密码 (Caesar Cipher)");
        Console::menuItem("1", "异或加密 (XOR Encryption)");
        Console::menuItem("-1", "退出程序");
        Console::separator();
        Console::prompt("请选择算法");

        cout << ">>> ";
        if (cin.peek() == '\n') cin.ignore();;
        getline(cin, chosen);
        Console::clear();

        if (chosen == "-1") {
            Console::success("退出");
            return 0;
        } else if (chosen == "0") {
            crypto.set_type(Crypto_type::CAESAR);
            Console::info("凯撒加解密");
            Console::warning("注意：仅转换英文字母，密钥范围 [1, 25]");
        } else if (chosen == "1") {
            crypto.set_type(Crypto_type::XOR);
            Console::info("异或加解密");
            Console::warning("注意：密钥应为十六进制字符串 (如: 1A2B3C)");
        } else {
            Console::error("无效选择，请重新输入");
            continue;
        }

        // 操作菜单循环
        while (true) {
            Console::separator();
            Console::title("操作菜单");
            Console::menuItem("0", "文本加密");
            Console::menuItem("1", "文本解密");
            Console::menuItem("2", "文件加密");
            Console::menuItem("3", "文件解密");
            Console::menuItem("-1", "返回上级菜单");
            Console::separator();
            Console::prompt("请选择操作");
            cout << ">>> ";
            if (cin.peek() == '\n') cin.ignore();;
            getline(cin, chosen);
            Console::clear();

            if (chosen == "-1") {
                break;
            } else if (chosen == "0") {
                crypto.set_status(Crypto_status::ENCRYPT);
                handleTextEncryption();
                waitForEnter();
            } else if (chosen == "1") {
                crypto.set_status(Crypto_status::DECRYPT);
                handleTextDecryption();
                waitForEnter();
            } else if (chosen == "2") {
                crypto.set_status(Crypto_status::ENCRYPT);
                handleFileEncryption();
                waitForEnter();
            } else if (chosen == "3") {
                crypto.set_status(Crypto_status::DECRYPT);
                handleFileDecryption();
                waitForEnter();
            } else {
                Console::error("无效选择，请重新输入");
            }
        }
    }
}

string& FileHandler::file_to_string(const string& path) {
    success = false;
    content.clear();
    
    std::ifstream file(path);
    if (!file.is_open()) {
        Console::error("无法打开文件: " + path);
        return content;
    }
    
    try {
        std::stringstream buffer;
        buffer << file.rdbuf();
        content = buffer.str();
        file.close();
        success = true;
        Console::success("文件读取成功");
    } catch (const std::exception& e) {
        Console::error("文件读取失败: " + string(e.what()));
    }
    
    return content;
}

void FileHandler::string_to_file(const string& text, const string& path) {
    success = false;
    
    std::ofstream file(path);
    if (!file.is_open()) {
        Console::error("无法创建文件: " + path);
        return;
    }
    
    try {
        file << text;
        file.close();
        success = true;
        Console::success("文件写入成功: " + path);
    } catch (const std::exception& e) {
        Console::error("文件写入失败: " + string(e.what()));
    }
}