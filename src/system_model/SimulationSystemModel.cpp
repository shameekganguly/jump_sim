// implementation for the SimulationSystemModel class

#include "SimulationSystemModel.h"

using namespace std;
using namespace Eigen;

// ctor
SimulationSystemModel::SimulationSystemModel(
	const std::string& world_fname,
	const std::string& robot_fname,
	const std::string& robot_name,
	const std::string& system_name
)
: SystemModel(system_name)
{
	// load simulation environment
	_sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);

	// name robot
	_robot_name = robot_name;
}

// set an arbitrary initial state for the robot
bool SimulationSystemModel::robotInitialStateIs(
	const Eigen::VectorXd& q_set,
	const Eigen::VectorXd& dq_set
) {
	// update initial state
	_initial_state->_ctrl_state->_q = q_set;
	_initial_state->_ctrl_state->_dq = dq_set;
	// TODO: ^^ how does this deal with the controller possibly being in the wrong 
	// state for the given robot state?

	return reinitialize();
}

// control pause
void SimulationSystemModel::pauseIs(bool should_pause) {
	// simply set a pause. The control and sim threads should automatically react to the pause
	(dynamic_cast<SimulationSystemState*>(_curr_state))->_is_paused = should_pause;
}

// start/stop simulation
void SimulationSystemModel::runningIs(bool should_run) {
	// start system if currently stopped
	if (should_run && !(dynamic_cast<SimulationSystemState*>(_curr_state))->_is_running) {
		//TODO: start simulation, control
	}
	// stop system if currently started
	if (!should_run && (dynamic_cast<SimulationSystemState*>(_curr_state))->_is_running) {
		//TODO: stop simulation, control
	}
}

// dtor 
SimulationSystemModel::~SimulationSystemModel() {
	delete _sim;
}

// overloaded base class function: reinitialize
bool SimulationSystemModel::reinitialize() {
	// stop system if it is running
	runningIs(false);

	// base class function resets controller and syncs current state and initial state
	if (!SystemModel::reinitialize()) { return false; }

	// reset sim
	// TODO: for now, we only reset robot's state. Ideally, we should reset the
	// state of other objects in the scene, as well as the internal simulation state
	_sim->setJointPositions(_robot_name, _curr_state->_ctrl_state->_q);
	_sim->setJointVelocities(_robot_name, _curr_state->_ctrl_state->_dq);

	// TODO: do some consistency checks
	return true;
}
