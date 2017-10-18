// implementation for the SimulationSystemModel class

#include "SimulationSystemModel.h"
#include "timer/LoopTimer.h"

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
	auto state = dynamic_cast<SimulationSystemState*>(_curr_state);
	// start system if currently stopped
	if (should_run && !(state->_is_running)) {
		// start simulation
		state->_is_running = true;
		_sim_thread = thread(&SimulationSystemModel::runSimulation, this);

		// start control
		_control_thread = thread(&SimulationSystemModel::runControl, this);
	}
	// stop system if currently started
	if (!should_run && state->_is_running) {
		// stop simulation
		state->_is_running = false;
		_sim_thread.join();

		// stop control
		_control_thread.join();
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

// internal thread functions: simulation
void SimulationSystemModel::runSimulation() {
	// get pointer to current state
	auto state = dynamic_cast<SimulationSystemState*>(_curr_state);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(state->_sim_rate);
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (state->_is_running) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		if (!state->_is_paused) {
			_sim->integrate(loop_dt);
		}

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

// internal thread functions: control
void SimulationSystemModel::runControl() {
	// get pointer to current state
	auto state = dynamic_cast<SimulationSystemState*>(_curr_state);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(state->_control_rate);
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;

	while (state->_is_running) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// copy over controller state to system state
		_curr_state->_ctrl_state->copy(_controller->_state);

		// copy model from sim to controller
		// this can be overloaded from the derived sim system class
		simToModelTransfer();

		// if not paused, update control torques
		if (!state->_is_paused) {
			_controller->controllerStateIs(_curr_state->_ctrl_state);
			_sim->setJointTorques(_robot_name, _controller->controlTorques());
		}

		// update system current state with controller state
		_curr_state->_ctrl_state->copy(_controller->_state);

		// TODO: verify that full TORO controller state is being copied

		// update last time
		last_time = curr_time;
	}
}

// internal model copy function from simulation to the system internal state
void SimulationSystemModel::simToModelTransfer() {
	// read joint positions, velocities, update controller state
	_sim->getJointPositions(_robot_name, _curr_state->_ctrl_state->_q);
	_sim->getJointVelocities(_robot_name, _curr_state->_ctrl_state->_dq);
}
