#ifndef CONSOLE_TOOL_H
#define CONSOLE_TOOL_H

#include <iostream>
#include <string>
#include <stdexcept>

using std::string, std::cout, std::cin, std::endl;

enum class Color {
    BLACK = 0,
    BLUE = 1,
    GREEN = 2,
    CYAN = 3,
    RED = 4,
    MAGENTA = 5,
    YELLOW = 6,
    WHITE = 7,
    GRAY = 8,
    BRIGHT_BLUE = 9,
    BRIGHT_GREEN = 10,
    BRIGHT_CYAN = 11,
    BRIGHT_RED = 12,
    BRIGHT_MAGENTA = 13,
    BRIGHT_YELLOW = 14,
    BRIGHT_WHITE = 15
};

class Console {
private:
    static string getForegroundCode(Color color) {
        switch(color) {
            case Color::BLACK: return "30";
            case Color::RED: return "31";
            case Color::GREEN: return "32";
            case Color::YELLOW: return "33";
            case Color::BLUE: return "34";
            case Color::MAGENTA: return "35";
            case Color::CYAN: return "36";
            case Color::WHITE: return "37";
            case Color::GRAY: return "90";
            case Color::BRIGHT_RED: return "91";
            case Color::BRIGHT_GREEN: return "92";
            case Color::BRIGHT_YELLOW: return "93";
            case Color::BRIGHT_BLUE: return "94";
            case Color::BRIGHT_MAGENTA: return "95";
            case Color::BRIGHT_CYAN: return "96";
            case Color::BRIGHT_WHITE: return "97";
            default: return "37";
        }
    }
    
    static string getBackgroundCode(Color color) {
        switch(color) {
            case Color::BLACK: return "40";
            case Color::RED: return "41";
            case Color::GREEN: return "42";
            case Color::YELLOW: return "43";
            case Color::BLUE: return "44";
            case Color::MAGENTA: return "45";
            case Color::CYAN: return "46";
            case Color::WHITE: return "47";
            case Color::GRAY: return "100";
            case Color::BRIGHT_RED: return "101";
            case Color::BRIGHT_GREEN: return "102";
            case Color::BRIGHT_YELLOW: return "103";
            case Color::BRIGHT_BLUE: return "104";
            case Color::BRIGHT_MAGENTA: return "105";
            case Color::BRIGHT_CYAN: return "106";
            case Color::BRIGHT_WHITE: return "107";
            default: return "40";
        }
    }

public:
    static void clear() {
        cout << "\033[2J\033[1;1H";
    }

    static void setColor(Color foreground, Color background = Color::BLACK) {
        cout << "\033[" << getForegroundCode(foreground) << "m";
        if (background != Color::BLACK) {
            cout << "\033[" << getBackgroundCode(background) << "m";
        }
    }

    static void resetColor() {
        cout << "\033[0m";
    }

    static void setCursorPosition(int x, int y) {
        cout << "\033[" << y << ";" << x << "H";
    }

    static void out(const string& text, Color foreground, Color background = Color::BLACK) {
        setColor(foreground, background);
        cout << text;
        resetColor();
    }

    static void info(const string& text) {
        out(text + "\n", Color::BRIGHT_BLUE);
    }

    static void success(const string& text) {
        out(text + "\n", Color::BRIGHT_GREEN);
    }

    static void warning(const string& text) {
        out(text + "\n", Color::BRIGHT_YELLOW);
    }

    static void error(const string& text) {
        out(text + "\n", Color::BRIGHT_RED);
    }

    static void debug(const string& text) {
        out(text + "\n", Color::MAGENTA);
    }

    static void highlight(const string& text) {
        out(text + "\n", Color::BRIGHT_CYAN);
    }

    static void title(const string& text) {
        out(text + "\n", Color::BRIGHT_MAGENTA);
    }

    static void separator(char ch = '=', Color color = Color::GRAY) {
        string sep(60, ch);
        out(sep + "\n", color);
    }

    static void menuItem(const string& index, const string& text) {
        setColor(Color::BRIGHT_WHITE);
        cout << "[";
        setColor(Color::BRIGHT_CYAN);
        cout << index;
        setColor(Color::BRIGHT_WHITE);
        cout << "] " << text << endl;
        resetColor();
    }

    static void prompt(const string& text) {
        out("> " + text, Color::BRIGHT_GREEN);
    }

    static void result(const string& label, const string& value) {
        setColor(Color::BRIGHT_WHITE);
        cout << label << ": ";
        setColor(Color::BRIGHT_YELLOW);
        cout << value << endl;
        resetColor();
    }
};

#endif