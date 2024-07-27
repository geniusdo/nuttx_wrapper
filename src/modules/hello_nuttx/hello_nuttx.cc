#include "hello_nuttx.h"

HelloNuttx::HelloNuttx() {
  test_ = 1;
}

int HelloNuttx::GetTest() const {
  return test_;
}
