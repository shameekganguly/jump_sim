// SystemModel.h
#ifndef SYSTEM_MODEL_H
#define SYSTEM_MODEL_H

#include "whole_body_controller/WholeBodyController.h"
#include "trajectory_optimization/ParametrizedTrajectory.h"
#include <string>

// struct to encapsulate system state. consists of controller state and plant state
struct SystemState {
public:
	// controller state. includes current ParametrizedTrajectory being executed.
	// NOTE: this variable must be instantiated by the derived class
	ControllerState* _ctrl_state;

	// TODO: add perceptual state. e.g. map for localization, object dictionary, etc.
	// This might be polled by the TrajectoryOptimizer to re-parametrize trajectory

	// state name
	std::string _state_name = "SystemState";

	// virtual destructor to mark struct polymorphic
	virtual ~SystemState() {
		delete _ctrl_state;
	}

	// copy from another state object, by const reference
	virtual void copy(SystemState* other) {
		_ctrl_state->copy(other->_ctrl_state);
		_state_name = other->_state_name;
	}
};

// class to encapsulate closed loop system model which consists of a controller
// and the plant being controlled.
// base class plant state is immutable. But the class can be reset to an 
// initial state (kind of like homing).
class SystemModel {
public:
	// constructor
	SystemModel(const std::string& system_name)
	: _controller(NULL), _system_name(system_name) {
		// nothing to do
	}

	// set controller state
	virtual void controllerStateIs (ControllerState* state) {
		_controller->controllerStateIs(state);
		// update current state
		_curr_state->_ctrl_state->copy(_controller->_state);
	}

	// get controller state
	const ControllerState& controllerState () const {
		return *(_curr_state->_ctrl_state);
	}

	// get internal state
	const SystemState& systemState () const {
		return *_curr_state;
	}

	// reset to initial state. returns False if this is currently not possible
	virtual bool reinitialize() {
		// reset self state
		_curr_state->copy(_initial_state);
		// set controller state. NOTE: this segfaults if controller is NULL
		_controller->controllerStateIs(_curr_state->_ctrl_state);
		// TODO: ^^ideally, at this point, controller's internal state should 
		// be the same as the set state. But this is really not guaranteed.

		return true;
	}

	// objective function for current execution on a given trajectory
	// Note: this internally updates the controller state to follow the given trajectory
	// TODO: consider having this only on a simulated system model?
	virtual double evaluateTrajectory(const ParametrizedTrajectory& trajectory) {
		return -1;
	}

	// const access to controller
	const WholeBodyController* controller() const {
		return _controller;
	}

	// dtor
	virtual ~SystemModel() {
		delete _controller;
		delete _curr_state;
		delete _initial_state;
	}

protected:
	// whole body controller
	WholeBodyController* _controller;

	// initial state
	SystemState* _initial_state;

	// current state
	SystemState* _curr_state;

	// system name
	std::string _system_name;
};

#endif //SYSTEM_MODEL_H
