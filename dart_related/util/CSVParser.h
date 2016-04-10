/**
 * Reads in data in a pseudo-CSV format.
 *
 * @author Ted Li
 */

#include <fstream>
#include <iostream>
#include <vector>

template <class T>
class CSVParser {
private:
  std::vector<std::vector<T> > _data;

public:
  void load_from_file_at(
    const std::string& file_name,
    T (*f)(const std::string& s)
  ) {
    _data.clear();
    std::ifstream input;
    input.open(file_name, std::fstream::in);
    std::vector<std::string> lines;
    for (std::string temp; getline(input, temp); ) {
      lines.push_back(temp);
    }
    input.close();
    for (auto it = lines.begin(); it != lines.end(); ++it) {
      _data.push_back(std::vector<T>());
      std::istringstream line_stream(*it);
      for (std::string temp; getline(line_stream, temp, ','); ) {
        T value;
        if (temp[0] == ' ') {
          value = f(temp.substr(1, temp.size() - 1));
        } else {
          value = f(temp);
        }
        _data.back().push_back(value);
      }
    }
  }

  const std::vector<std::vector<T> >& data() {
    return _data;
  }
};
