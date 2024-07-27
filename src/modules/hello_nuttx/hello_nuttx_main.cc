#include "hello_nuttx.h"

extern "C" {
int hello_nuttx_main(int argc, char** argv) {
  HelloNuttx test_nuttx;
  std::printf("Test Nuttx with C++!\n");
  return test_nuttx.GetTest();
}
}
