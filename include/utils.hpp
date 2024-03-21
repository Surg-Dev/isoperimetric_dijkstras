#pragma once
#include <cstdarg>
#include <iostream>
#include <string>

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