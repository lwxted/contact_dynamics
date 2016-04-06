/**
 * Reporter class that buffers and ouputs / reports states in a pseudo-CSV
 * format.
 *
 * @author Ted Li
 */

#include <vector>
#include <string>
#include <fstream>

template <class T>
class Reporter {
private:
  std::vector<std::vector<T> > _data;

public:
  void add_data(const std::vector<T>& data_point)
  {
    _data.push_back(std::vector<T>(data_point));
    if (!_data.empty()) {
      if (_data[0].size() != data_point.size()) {
        throw std::invalid_argument("Data size not consistent.");
      }
    }
  }

  void clear_data()
  {
    _data.clear();
  }

  void dump_to_file_at(const std::string& file_name)
  {
    std::ofstream output;
    output.open(file_name, std::fstream::out);
    for (auto it = _data.begin(); it != _data.end(); ++it) {
      for (auto itt = it->begin(); itt != it->end(); ++itt) {
        output << *itt;
        if (itt + 1 != it->end()) {
          output << ", ";
        }
      }
      output << std::endl;
    }
    output.close();
  }
};
