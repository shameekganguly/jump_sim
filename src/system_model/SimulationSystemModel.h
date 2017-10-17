//SimulationSystemModel.h
#ifndef SIMULATION_SYSTEM_MODEL_H
#define SIMULATION_SYSTEM_MODEL_H

#include "SystemModel.h"
#include "simulation/Sai2Simulation.h"
#include <thread>
#include <Eigen/Dense>

struct SimulationSystemState: public SystemState {
	// simulation rate
	uint _sim_rate = 1500; // usually 3x the rate of control rate in simulation time

	// simulation speed up rate (real-time = 1)
	uint _sim_speed_up_rate = 1;

	// control rate
	uint _control_rate = 500;

	// pause state
	bool _is_paused = false;

	// running state
	bool _is_running = false;

	virtual void copy(SystemState* other) {
		SystemState::copy(other);

		auto other_simstate = dynamic_cast<SimulationSystemState*>(other);
		_sim_rate = other_simstate->_sim_rate;
		_sim_speed_up_rate = other_simstate->_sim_speed_up_rate;
		_control_rate = other_simstate->_control_rate;
		_is_paused = other_simstate->_is_paused;
		_is_running = other_simstate->_is_running;
	}
};

class SimulationSystemModel: public SystemModel {
public:
	// ctor
	// NOTE: derived app level class should call reinitialize right after
	SimulationSystemModel(const std::string& world_fname, const std::string& robot_fname, const std::string& robot_name, const std::string& system_name);

	// dtor
	virtual ~SimulationSystemModel();	

	// set an arbitrary initial state for the robot
	// Note: this automatically calls reinitialize right after
	virtual bool robotInitialStateIs(const Eigen::VectorXd& q_set, const Eigen::VectorXd& dq_set);

	// TODO: add a function to set an arbitrary state for other sim objects

	// pause, unpause system
	virtual void pauseIs(bool should_pause);

	// start, stop system completely
	virtual void runningIs(bool should_run);

	// overloaded base class: reinitialize
	virtual bool reinitialize();

private:
	// internal controller thread
	virtual void runControl();

	// internal simulation thread
	virtual void runSimulation();

protected:
	// internal model copy function from simulation to the controller
	virtual void simToModelTransfer();

public:
	// simulation interface
	Simulation::Sai2Simulation* _sim;

	//controller thread
	std::thread _control_thread;

	//simulation thread
	std::thread _sim_thread;

	// robot name
	std::string _robot_name;
};

#endif //SIMULATION_SYSTEM_MODEL_H
