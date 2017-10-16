// WholeBodyController.h
#ifndef WHOLE_BODY_CONTROLLER_H
#define WHOLE_BODY_CONTROLLER_H

#include "trajectory_optimization/ParametrizedTrajectory.h"
#include "model/ModelInterface.h"
#include "model/RBDLModel.h"
#include <Eigen/Dense>
#include <string>

struct ControllerState {
public:
	// trajectory being executed
	ParametrizedTrajectory _trajectory;

	// robot q
	Eigen::VectorXd _q;

	// robot dq
	Eigen::VectorXd _dq;

	// robot dof
	uint _dof;

	// state name
	std::string _state_name = "WholeBodyControllerState";

	// copy
	virtual void copy(ControllerState* other) {
		_q = other->_q;
		_dq = other->_dq;
		_trajectory = other->_trajectory;
		_dof = other->_dof;
		_state_name = other->_state_name;
	}

	// virtual destructor to mark struct polymorphic
	virtual ~ControllerState() {}
};

// class to encapsulate whole body controller
class WholeBodyController {
public:
	// ctor
	WholeBodyController(){
		// nothing to do in parent class
	}

	// set controller state
	// NOTE: this automatically updates part of the state as well if required
	// since computing control torques might cause a state change
	virtual void controllerStateIs (ControllerState* state) {
		_state->copy(state);
	}

	// get control torques
	const Eigen::VectorXd& controlTorques () const {
		return _torques;
	}

	// controller state
	ControllerState* _state;

protected:
	// control torques
	Eigen::VectorXd _torques;
};

#endif //WHOLE_BODY_CONTROLLER_H
