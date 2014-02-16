#ifndef VECTOR_H
#define VECTOR_H

#include <initializer_list>
#include <assert.h>
#include "catch.hpp"

template<typename T, int N>
class vector
{
public:

    T data[N];

    vector() {
        for(int i = 0; i < N; ++i) data[i] = 0;
    }

    vector(std::initializer_list<T> data) {
        int i = 0;
        for(T n : data) {
            CHECK(i < N);
            if(i < N) this->data[i++] = n;
        }
    }
};

typedef vector<double,3> double3;
typedef vector<double,2> double2;

#endif // VECTOR_H
