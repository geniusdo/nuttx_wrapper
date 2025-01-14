#ifndef STATIC_QUEUE_HPP
#define STATIC_QUEUE_HPP
#include <cstdint>
#include <cstddef>
#include <cstring>
// implementation of a static circular queue
template <typename T, int max_size>
class StaticQueue
{
public:
    StaticQueue() : front_idx(0), _size(0), data((T *)raw_data) {}
    ~StaticQueue() = default;

    bool empty() const { return _size == 0; }

    size_t size() const
    {
        return _size;
    }

    void push(const T &value)
    {
        if (_size < max_size)
        {
            _size++;
        }
        else
        {
            // ringbuf full, move the first pointer
            front_idx++;
        }
        memcpy(&data[(front_idx + _size) % max_size], &value, sizeof(T));
        return;
    }

    void pop()
    {
        if (empty())
            return;
        _size--;
        front_idx = (front_idx + 1) % max_size;
        return;
    }

    T &front()
    {
        return data[front_idx];
    }

    T &back()
    {
        return data[(front_idx + _size) % max_size];
    }

protected:
    uint8_t raw_data[max_size * sizeof(T)];
    size_t front_idx;
    size_t _size;
    T *const data;
};

#endif // STATIC_QUEUE_HPP