#pragma once
#include "task.hpp"
#include <string>

// Load a ground epistemic planning task from the JSON file
PlanningTask load_task(const std::string& json_path);

// Build task.init from task.theory (a finitary S5 theory). Returns false and
// leaves task.init empty if more than 2^max_free_atoms valuations would have
// to be enumerated.
bool materialize_theory(PlanningTask& task, unsigned max_free_atoms);
