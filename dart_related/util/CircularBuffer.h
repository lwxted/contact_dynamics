/**
 * Circular buffer, a thin wrapper around the std::queue data structure that
 * allows at most a fixed number of elements at one time.
 * @author Ted Li
 */

#include <deque>

template <typename T>
class CircularBuffer {
private:
  std::deque<T> _data;
  size_t _max_size;

public:
  CircularBuffer() : _max_size(0) {}
  CircularBuffer(size_t max_size) : _max_size(max_size) {}

  size_t size() {
    return _data.size();
  }

  T& operator[] (const size_t i) {
    return _data[i];
  }

  void add(T item) {
    if (_data.size() == _max_size) {
      _data.pop_front();
    }
    _data.push_back(item);
  }

  std::deque<T>& deque() {
    return _data;
  }

};
