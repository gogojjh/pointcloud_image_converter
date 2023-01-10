#ifndef TICTOC_H_
#define TICTOC_H_

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace pc_img_conv {
class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;  // returns millisec
  }

  double tocSecond() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count();  // returns millisec
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
}  // namespace pc_img_conv

#endif  // TICTOC_H
