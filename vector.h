#ifndef VECTOR_H
#define VECTOR_H

#include <iterator>

constexpr int BUF_SIZE = 100'000;

struct Vector {
    struct Iterator {
        using iterator_category = std::forward_iterator_tag;
        using difference_type   = std::ptrdiff_t;
        using value_type        = int;
        using pointer           = int*;
        using reference         = int&;

        explicit Iterator(const pointer ptr) : m_ptr(ptr) {}

        reference operator*() const { return *m_ptr; }
        pointer operator->()const { return m_ptr; }
        Iterator& operator++() { m_ptr++; return *this; }
        Iterator operator++(int) { const Iterator tmp = *this; ++(*this); return tmp; }
        friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_ptr == b.m_ptr; };
        friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_ptr != b.m_ptr; };

        pointer m_ptr;
    };

    Iterator begin() { return Iterator(buffer); }
    Iterator end() { return Iterator(buffer + size); }

    void push_back(const int x) {
        if (size == BUF_SIZE) {
            perror("bro what the fuck");
            exit(1);
        }
        buffer[size++] = x;
    }

    int buffer[BUF_SIZE];
    long long size = 0;
};



#endif //VECTOR_H
