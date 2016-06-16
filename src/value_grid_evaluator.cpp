#include "value_grid_evaluator.h"

void ValueGridEvaluator::TestEvaluator() {

  std::cout << "Printing from inside ValueGridEvaluator " << std::endl;  

}

ValueGrid* ValueGridEvaluator::GetValueGridPtr() {
	return &value_grid;
};
