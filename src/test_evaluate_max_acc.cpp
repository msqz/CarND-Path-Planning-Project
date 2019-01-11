#include <iostream>
#include <assert.h>
#include "cost.h"
#include "path.h"

void test_max_acc() {
  Path path;
  Localization localization;
  
  double cost = evaluate_max_acc(path, localization);
  assert(abs(cost - 1) == 0);
}

int main() {
  test_max_acc();
  std::cout << "done" << std::endl;

  return 0;
}
