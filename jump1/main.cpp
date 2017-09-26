/*  hw1 - main.cpp
This file includes the required code to implement problem 2.2.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 4/9/17
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/ChaiGraphics.h"
#include "simulation/Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/jump1/world.urdf";
const string robot_fname = "../resources/toro/toro.urdf";
const string robot_name = "Toro";
const string camera_name = "camera_side";
// const string camera_name = "camera_isometric";
// const string camera_name = "camera_front";
// const string camera_name = "camera_top";
// const string ee_link_name = "link6";

// contact link names
const string right_foot_name = "RL_foot";
const string left_foot_name = "LL_foot";
const string torso_name = "hip_base";

// global variables
Eigen::VectorXd q_home;
ForceSensorSim* left_foot_force_sensor;
ForceSensorDisplay* left_foot_force_display;
ForceSensorSim* right_foot_force_sensor;
ForceSensorDisplay* right_foot_force_display;

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);
	graphics->_world->setBackgroundColor(0.7, 0.7, 0.5);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);
	sim->setCollisionRestitution(0.0);
    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(1.0);
    sim->setCoeffFrictionDynamic(1.0);

	// set initial condition
	q_home.setZero(robot->dof());
	q_home << 0.0, //0 floating_base_px
				0.0,	//1 floating_base_py
				1.0,	//2 floating_base_pz
				0.0/180.0*M_PI, //3 floating_base_rx
				4.0/180.0*M_PI, //4 floating_base_ry
				0.0/180.0*M_PI,	//5 floating_base_rz
				-30.0/180.0*M_PI,	//6 right thigh adduction
				-60.0/180.0*M_PI,	//7 right thigh pitch
				-15/180.0*M_PI,	//8 right knee roll
				90/180.0*M_PI,	//9 right knee pitch
				15/180.0*M_PI,	//10 right ankle adduction
				-15/180.0*M_PI,	//11 right ankle pitch - axis incorrect
				0/180.0*M_PI,	//12 trunk roll
				-15/180.0*M_PI,	//13 right shoulder pitch
				45/180.0*M_PI,	//14 right shoulder adduction
				0/180.0*M_PI,	//15 right shoulder roll
				90/180.0*M_PI,	//16 right elbow pitch
				30/180.0*M_PI,	//17 right elbow roll
				0/180.0*M_PI,	//18 right hand adduction - axis incorrect
				-15/180.0*M_PI,	//19 left shoulder pitch
				45/180.0*M_PI,	//20 left shoulder adduction
				0/180.0*M_PI,	//21 left shoulder roll
				90/180.0*M_PI,	//22 left elbow pitch
				30/180.0*M_PI,	//23 left elbow roll
				0/180.0*M_PI,	//24 left hand adduction - axis incorrect
				0/180.0*M_PI,	//25 neck roll
				0/180.0*M_PI,	//26 neck pitch - axis incorrect
				30/180.0*M_PI,	//27 left thigh adduction
				-60/180.0*M_PI,	//28 left thigh pitch
				15/180.0*M_PI,	//29 left knee roll
				90/180.0*M_PI,	//30 left knee pitch
				-15/180.0*M_PI,	//31 left ankle adduction
				-15/180.0*M_PI;	//32 left ankle pitch - axis incorrect
	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	cout << "Reached here 1" << endl;
	robot->updateModel();
	cout << "Reached here 2" << endl;
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot, sim);

	// initialize force sensor: needs Sai2Simulation sim interface type
	left_foot_force_sensor = new ForceSensorSim(robot_name, left_foot_name, Eigen::Affine3d::Identity(), sim, robot);
	left_foot_force_display = new ForceSensorDisplay(left_foot_force_sensor, graphics);
	right_foot_force_sensor = new ForceSensorSim(robot_name, right_foot_name, Eigen::Affine3d::Identity(), sim, robot);
	right_foot_force_display = new ForceSensorDisplay(right_foot_force_sensor, graphics);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		left_foot_force_display->update();
		right_foot_force_display->update();
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime(); //secs

	// controller state variables
	enum FSMState {
		Balancing,
		Jumping,
		FallingMidAir
	};
	FSMState curr_state = FSMState::FallingMidAir;

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd tau_act = Eigen::VectorXd::Zero(robot->dof() - 6);
	Eigen::VectorXd gj(robot->dof());
	Eigen::MatrixXd selection_mat(robot->dof() - 6, robot->dof());
	// cout << selection_mat << endl;
	Eigen::MatrixXd actuated_space_projection;
	actuated_space_projection.setZero(robot->dof() - 6, robot->dof());
	Eigen::MatrixXd actuated_space_inertia;
	actuated_space_inertia.setZero(robot->dof() - 6, robot->dof() - 6);
	std::vector<Eigen::Vector3d> left_foot_force_list;
	std::vector<Eigen::Vector3d> left_foot_point_list;
	std::vector<Eigen::Vector3d> right_foot_force_list;
	std::vector<Eigen::Vector3d> right_foot_point_list;
	Eigen::MatrixXd J_task;
	Eigen::MatrixXd Jv_com(3, robot->dof()); //TODO: consider full 6-DOF com task?
	Eigen::MatrixXd Jw_left_foot(3, robot->dof());
	Eigen::MatrixXd Jw_right_foot(3, robot->dof());
	Eigen::MatrixXd Jv_left_foot(3, robot->dof());
	Eigen::MatrixXd Jv_right_foot(3, robot->dof());
	const Eigen::Vector3d left_foot_frame_pos_local(0.0, 0.0, 0.0); //TODO: should this be at the ground point or the joint?
	const Eigen::Vector3d right_foot_frame_pos_local(0.0, 0.0, 0.0); //TODO: should this be at the ground point or the joint?
	Eigen::Vector3d left_foot_frame_pos_world; // position of left foot frame
	Eigen::Vector3d right_foot_frame_pos_world; // position of right foot frame
	Eigen::MatrixXd J_feet_tension(1, robot->dof());
	Eigen::Vector3d left_to_right_foot_unit_vec;

	// gains
	double kplcom = 40.0; // COM linear kp
	double kvlcom = 10.0; // COM linear kv
	double kpj = 70.0; // joint space kp
	double kvj = 30.0; // joint space kv
	double kpwleftf = 40; // left foot admittance kp for zero moment control
	double kvwleftf = 10; // left foot kv damping
	double kpwrightf = 40; // right foot admittance kp for zero moment control
	double kvwrightf = 10; // right foot kv damping

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// update model every once in a while
		// TODO: this should be in a separate thread
		if (timer.elapsedCycles() % 10 == 1) {
			robot->updateModel();
			robot->gravityVector(gj, Eigen::Vector3d(0.0, 0.0, -0.98));
			actuated_space_inertia = (robot->_M_inv.block(6,6,robot->dof()-6, robot->dof()-6)).inverse();
			actuated_space_projection = actuated_space_inertia * (robot->_M_inv.block(6,0,robot->dof()-6, robot->dof()));
			// cout << robot->_M_inv.block(6,0,robot->dof()-6, robot->dof()) << endl;

			// get the task Jacobians
			// - Jv com: TODO: need to add up the Jacobians for all the links
			// For now, we use the torso frame as a surrogate
			robot->Jv(Jv_com, torso_name, Eigen::Vector3d(0.0, 0.0, 0.0));
			// - Jw left foot rot
			robot->Jw(Jw_left_foot, left_foot_name);
			// - Jw right foot rot
			robot->Jw(Jw_right_foot, right_foot_name);
			// - J feet tension
			robot->Jv(Jv_left_foot, left_foot_name, left_foot_frame_pos_local);
			robot->Jv(Jv_right_foot, right_foot_name, right_foot_frame_pos_local);
			robot->position(left_foot_frame_pos_world, left_foot_name, left_foot_frame_pos_local);
			robot->position(right_foot_frame_pos_world, right_foot_name, right_foot_frame_pos_local);
			left_to_right_foot_unit_vec = (right_foot_frame_pos_world - left_foot_frame_pos_world);
			left_to_right_foot_unit_vec.normalize();
			J_feet_tension = left_to_right_foot_unit_vec*(Jv_right_foot - Jv_left_foot);
		}

		// get all contact points from the sim
		sim->getContactList(left_foot_point_list, left_foot_force_list, robot_name, left_foot_name);
		sim->getContactList(right_foot_point_list, right_foot_force_list, robot_name, right_foot_name);

		// update force sensors
		left_foot_force_sensor->update();
		// Eigen::Vector3d left_foot_force;
		// Eigen::Vector3d left_foot_moment;
		// left_foot_force_sensor->getForce(left_foot_force);
		// left_foot_force_sensor->getMoment(left_foot_moment);
		// cout << "Left foot force: " << left_foot_moment.transpose()
		// 		<< " Left foot moment: " << left_foot_moment.transpose() << endl;
		right_foot_force_sensor->update();

		// set tau_act
		if (curr_state == FSMState::FallingMidAir) {
			// simply compensate for gravity and brace for landing
			tau_act = actuated_space_inertia*(- kvj*(robot->_dq.tail(robot->dof()-6)));
			tau_act += actuated_space_projection*gj;
			// TODO: check for contact and switch to FSMState::Balancing
			if (left_foot_point_list.size() || right_foot_point_list.size()) {
				curr_state = FSMState::Balancing;
				cout << "Switching from FallingMidAir to Balancing" << endl;
			}
		}

		if (curr_state == FSMState::Balancing) {
			// TODO: start zero moment control at feet, small tension, COM stabilization and posture control to q_home
			// - size the task Jacobian
			size_t J_task_size = 3; // for the COM task
			// TODO: consider omega for the body as well? In effect, right now, we are relying on posture to handle it
			bool f_left_foot_contact = (left_foot_point_list.size());
			bool f_right_foot_contact = (right_foot_point_list.size());
			if (f_left_foot_contact) { J_task_size += 3; } // for moment control at left foot
			if (f_right_foot_contact) { J_task_size += 3; } // for moment control at right foot
			if (f_left_foot_contact && f_right_foot_contact) { J_task_size += 1; } // for internal tension control between feet
			J_task.setZero(J_task_size, robot->dof());

			// - set the task Jacobian
			size_t J_task_counter = 0;
			J_task.block(J_task_counter,0,3,robot->dof()) = Jv_com;
			J_task_counter += 3;
			if (f_left_foot_contact) {
				J_task.block(J_task_counter,0,3,robot->dof()) = Jw_left_foot;
				J_task_counter += 3;
			}
			if (f_right_foot_contact) {
				J_task.block(J_task_counter,0,3,robot->dof()) = Jw_right_foot;
				J_task_counter += 3;
			}
			if (f_left_foot_contact && f_right_foot_contact) {
				J_task.block(J_task_counter,0,1,robot->dof()) = J_feet_tension;
				J_task_counter += 1;
			}

			// - compute task forces

			// - compute posture torques

			// - compute required joint torques

			// TODO: if COM velocity has been zero for a while, switch to FSMState::Jumping

			// TODO: check friction cone constraint
		}

		if (curr_state == FSMState::Jumping) {
			// TODO: start accelerating COM upwards while maintaining tension between feet
			// TODO: when contact is lost, switch to FSMState::FallingMidAir
		}

		// assemble full tau vector for simulation
		tau.tail(robot->dof()-6) = tau_act;
		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Jump Sim", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
