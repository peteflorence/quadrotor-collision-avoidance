#ifndef VALUE_GRID_EVALUATOR_H
#define VALUE_GRID_EVALUATOR_H

#include <iostream>
#include "trajectory.h"
#include "value_grid.h"

class ValueGridEvaluator {
public:
	ValueGrid* GetValueGridPtr();
  void TestEvaluator();

private:
	ValueGrid value_grid;
};

#endif