#pragma once
#include <cstdarg>
#include <iostream>
#include <string>
#include <chrono>

using namespace std;

template<typename T>
void prints(T a) {
    std::cout << a << std::endl;
}

template<typename T, typename T2>
void prints(T a, T2 b){
    std::cout << a << " " << b << std::endl;
}

template<typename T, typename T2, typename T3>
void prints(T a, T2 b, T3 c){
    std::cout << a << " " << b << " " << c << std::endl;
}

template <
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}