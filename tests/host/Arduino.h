#pragma once
#include <cstdint>
#include <climits>
#include <string>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <type_traits>
#include <vector>
class String : public std::string {
public:
    using std::string::string;
    using std::string::operator=;
    String(const std::string& value) : std::string(value) {}
    template<class T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    explicit String(T value) : std::string(std::to_string(value)) {}
    String(double value, unsigned precision) {
        char buffer[128];
        std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
        assign(buffer);
    }
    bool isEmpty() const { return empty(); }
    char charAt(size_t i) const { return i < size() ? (*this)[i] : '\0'; }
    int indexOf(const String& s, size_t from = 0) const {
        auto p = find(s, from);
        return p == npos ? -1 : static_cast<int>(p);
    }
    int indexOf(char c, size_t from = 0) const {
        auto p = find(c, from);
        return p == npos ? -1 : static_cast<int>(p);
    }
    int lastIndexOf(char c) const {
        auto p = rfind(c);
        return p == npos ? -1 : static_cast<int>(p);
    }
    String substring(size_t from, size_t to = npos) const {
        if (from >= size())
            return "";
        return substr(from, to == npos ? npos : to - from);
    }
    bool startsWith(const String& s) const { return compare(0, s.size(), s) == 0; }
    bool endsWith(const String& s) const { return size() >= s.size() && compare(size() - s.size(), s.size(), s) == 0; }
    void trim() {
        auto first = find_first_not_of(" \t\r\n");
        if (first == npos) {
            clear();
            return;
        }
        assign(substr(first, find_last_not_of(" \t\r\n") - first + 1));
    }
    void toLowerCase() {
        for (char& c: *this)
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    bool equalsIgnoreCase(String s) const {
        String copy = *this;
        copy.toLowerCase();
        s.toLowerCase();
        return copy == s;
    }
    long toInt() const { return std::strtol(c_str(), nullptr, 10); }
    float toFloat() const { return std::strtof(c_str(), nullptr); }
    void replace(const String& from, const String& to) {
        size_t pos = 0;
        while (!from.empty() && (pos = find(from, pos)) != npos) {
            std::string::replace(pos, from.size(), to);
            pos += to.size();
        }
    }
};
class HardwareSerial {
public:
    std::deque<char> input;
    std::vector<String> output;
    void setRxBufferSize(int) {}
    void begin(int, int, int, int) {}
    int available() const { return static_cast<int>(input.size()); }
    int read() {
        char c = input.front();
        input.pop_front();
        return c;
    }
    void print(const String& value) { output.push_back(value); }
    void reply(const String& value) {
        for (char c: value)
            input.push_back(c);
    }
};
inline HardwareSerial Serial1;
inline bool isDigit(char c) {
    return c >= '0' && c <= '9';
}
constexpr int SERIAL_8N1 = 0;
template<class T>
T constrain(T value, T low, T high) {
    return std::max(low, std::min(value, high));
}
extern uint32_t testMillis;
inline unsigned long millis() {
    return testMillis;
}
