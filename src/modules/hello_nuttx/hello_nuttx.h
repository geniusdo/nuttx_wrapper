#pragma once

#include <nuttx/nuttx.h>
#include <cstdio>

class HelloNuttx {
 public:
  HelloNuttx();
  int GetTest() const;

 private:
  int test_{0};
};
