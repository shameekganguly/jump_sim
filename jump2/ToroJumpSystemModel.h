//ToroJumpSystemModel.h
#ifndef TORO_JUMP_SYSTEM_MODEL_H
#define TORO_JUMP_SYSTEM_MODEL_H

#include "system_model/SimulationSystemModel.h"
#include "ToroJumpController.h"
#include <string>

struct ToroJumpSystemState: public SimulationSystemState {
	// need to instantiate the system state by creating a blank (default) controller state.
	ToroJumpSystemState() {
		_ctrl_state = new ToroJumpControllerState();
	}
};

class ToroJumpSystemModel: public SimulationSystemModel {
public:
	// ctor
	// loads a robot model from file
	ToroJumpSystemModel (const std::string& world_fname, const std::string& robot_fname, const std::string& robot_name, const Eigen::VectorXd q_home, const Eigen::VectorXd dq_home)
	: SimulationSystemModel(
		world_fname, robot_fname, robot_name, "ToroJumpSimSystem"
	) {
		// load the controller
		// NOTE: this definitely has to be done before the base class constructor
		// is called to avoid a seg fault
		_controller = new ToroJumpController(robot_fname);

		// initialize state stores
		_curr_state = new ToroJumpSystemState();
		_initial_state = new ToroJumpSystemState();

		// set home position, velocity
		_initial_state->_ctrl_state->_q = q_home;
		_initial_state->_ctrl_state->_dq = dq_home;

		// reinitialize
		reinitialize(); // TODO: check for improper reinitialization and throw execption
	}
};

#endif // TORO_JUMP_SYSTEM_MODEL_H
