#include <iostream>
#include <string>
#include <thread>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;

const string robot_fname = "resources/eight_bar.urdf";
const string world_fname = "resources/world.urdf";
const string robot_name = "EightBar";
const string camera_name = "camera_top";

// contact link names
const string right_foot_name = "right_foot";
const string left_foot_name = "left_foot";
const string torso_name = "hip_base";

const Vector3d left_foot_pos_local(0.0, -0.015, 0.0);
const Vector3d right_foot_pos_local(0.0, 0.015, 0.0);

// global variables
VectorXd q_home;
bool f_global_sim_pause = false; // use with caution!

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// util function to calculate the pseudo inverse
MatrixXd pseudoinverse(const MatrixXd &mat, double tolerance = 1e-4);

int main (int argc, char** argv) {
	// initialize random
	srand (time(NULL));

	cout << "Loading URDF world model file: " << robot_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);
	graphics->_world->setBackgroundColor(0.7, 0.7, 0.5);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);
	int dof = robot->dof();
	int actuated_dof = robot->dof() - 3; // planar

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	sim->setCollisionRestitution(0.2);
 //    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(1.0);
    sim->setCoeffFrictionDynamic(1.0);

	// set test condition
	q_home.setZero(dof);
	q_home << -1.8, //0 floating_base_px
				0.0,	//1 floating_base_py
				((float) (rand() % 30 - 15)) /180.0*M_PI,	//2 floating_base_rz
				// 0.0/180.0*M_PI,	//2 floating_base_rz
				60.0/180.0*M_PI,	//3 left thigh adduction
				50.0/180.0*M_PI,	//4 left knee adduction
				-15.0/180.0*M_PI,	//5 left foot adduction
				-60.0/180.0*M_PI,	//6 right thigh adduction
				-50.0/180.0*M_PI,	//7 right knee adduction
				15.0/180.0*M_PI;	//8 right foot adduction


	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot, sim);

	// // initialize force sensor: needs Sai2Simulation sim interface type
	// left_foot_force_sensor = new ForceSensorSim(robot_name, left_foot_name, Eigen::Affine3d::Identity(), sim, robot);
	// left_foot_force_display = new ForceSensorDisplay(left_foot_force_sensor, graphics);
	// right_foot_force_sensor = new ForceSensorSim(robot_name, right_foot_name, Eigen::Affine3d::Identity(), sim, robot);
	// right_foot_force_display = new ForceSensorDisplay(right_foot_force_sensor, graphics);

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
		// left_foot_force_display->update();
		// right_foot_force_display->update();
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
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	// controller state variables
	enum FSMState {
		BalancingDoubleStance,
		BalancingSingleStance,
		Jumping,
		FallingMidAir
	};
	FSMState curr_state = FSMState::FallingMidAir;

	// dof counts
	int dof = robot->dof();
	int act_dof = robot->dof() - 3; // 2D free base

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd temp_tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd tau_act = Eigen::VectorXd::Zero(robot->dof() - 3);
	Eigen::VectorXd tau_act_passive = Eigen::VectorXd::Zero(robot->dof() - 3);
	Eigen::VectorXd gj(robot->dof());
	Eigen::MatrixXd selection_mat(robot->dof() - 3, robot->dof());
	// cout << selection_mat << endl;
	Eigen::MatrixXd actuated_space_projection;
	actuated_space_projection.setZero(robot->dof() - 3, robot->dof());
	Eigen::MatrixXd actuated_space_inertia;
	actuated_space_inertia.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd null_actuated_space_projection_contact;
	null_actuated_space_projection_contact.setZero(robot->dof() - 3, robot->dof() - 3);

	Eigen::MatrixXd actuated_space_projection_contact_both;
	actuated_space_projection_contact_both.setZero(robot->dof() - 3, robot->dof());
	Eigen::MatrixXd actuated_space_inertia_contact_both;
	actuated_space_inertia_contact_both.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd actuated_space_inertia_contact_inv_both;
	actuated_space_inertia_contact_inv_both.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd feet_internal_forces_selection_mat;
	feet_internal_forces_selection_mat.setZero(3,6);
	feet_internal_forces_selection_mat(0,2) = 1; // left foot moment
	feet_internal_forces_selection_mat(1,5) = 1; // right foot moment
	feet_internal_forces_selection_mat(2,1) = -1; // internal tension
	feet_internal_forces_selection_mat(2,4) = 1; // internal tension
	Eigen::MatrixXd feet_null_projector_both;
	feet_null_projector_both.setZero(act_dof+3, act_dof);
	Eigen::VectorXd feet_tension_projector_both_gravity(dof);
	Eigen::MatrixXd feet_null_projector_both_inv(act_dof, act_dof+3);

	Eigen::MatrixXd actuated_space_projection_contact_left;
	actuated_space_projection_contact_left.setZero(robot->dof() - 3, robot->dof());
	Eigen::MatrixXd actuated_space_inertia_contact_left;
	actuated_space_inertia_contact_left.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd actuated_space_inertia_contact_inv_left;
	actuated_space_inertia_contact_inv_left.setZero(robot->dof() - 3, robot->dof() - 3);

	Eigen::MatrixXd actuated_space_projection_contact_right;
	actuated_space_projection_contact_right.setZero(robot->dof() - 3, robot->dof());
	Eigen::MatrixXd actuated_space_inertia_contact_right;
	actuated_space_inertia_contact_right.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd actuated_space_inertia_contact_inv_right;
	actuated_space_inertia_contact_inv_right.setZero(robot->dof() - 3, robot->dof() - 3);

	std::vector<Eigen::Vector3d> left_foot_force_list;
	std::vector<Eigen::Vector3d> left_foot_point_list;
	std::vector<Eigen::Vector3d> right_foot_force_list;
	std::vector<Eigen::Vector3d> right_foot_point_list;
	Eigen::MatrixXd J_task;
	Eigen::MatrixXd L_task;
	Eigen::VectorXd F_task;
	Eigen::VectorXd F_task_passive;
	Eigen::MatrixXd J0_com(6, robot->dof());
	Eigen::MatrixXd Jv_com(2, robot->dof()); //TODO: consider full 6-DOF com task?
	Eigen::Vector3d com_pos;
	Eigen::Vector3d com_pos_err;
	Eigen::Vector3d com_v;
	// Eigen::MatrixXd Jw_left_foot(3, robot->dof());
	// Eigen::MatrixXd Jw_right_foot(3, robot->dof());
	Eigen::MatrixXd J0_left_foot(6, robot->dof());
	Eigen::MatrixXd J0_right_foot(6, robot->dof());
	Eigen::MatrixXd Jv_left_foot(2, robot->dof());
	Eigen::MatrixXd Jv_right_foot(2, robot->dof());
	Eigen::Vector3d left_foot_frame_pos_world; // position of left foot frame
	Eigen::Vector3d right_foot_frame_pos_world; // position of right foot frame
	Eigen::MatrixXd J_feet_tension(1, robot->dof());
	Eigen::Vector3d left_to_right_foot_unit_vec;
	Eigen::Vector3d pos_support_centroid;
	Eigen::MatrixXd J_c_both(6, robot->dof());
	Eigen::MatrixXd J_c_left(2, robot->dof());
	Eigen::MatrixXd J_c_right(2, robot->dof());
	Eigen::MatrixXd L_contact_both(6, 6);
	Eigen::MatrixXd L_contact_left(2, 2);
	Eigen::MatrixXd L_contact_right(2, 2);
	Eigen::MatrixXd N_contact_both(dof, dof);
	Eigen::MatrixXd N_contact_left(dof, dof);
	Eigen::MatrixXd N_contact_right(dof, dof);
	Eigen::MatrixXd SN_contact_both(act_dof, dof);
	Eigen::MatrixXd SN_contact_left(act_dof, dof);
	Eigen::MatrixXd SN_contact_right(act_dof, dof);
	Eigen::VectorXd F_contact;
	uint balance_counter = 0;
	const uint BALANCE_COUNT_THRESH = 11;
	uint stable_counter;
	const uint STABLE_COUNT_THRESH = 11;
	uint jump_counter;
	const uint JUMP_COUNT_THRESH = 7;

	Eigen::VectorXd q_fall;

	uint num_successful_jumps = 0;

	// gains
	double kplcom = 20.0; // COM linear kp
	double kvlcom = 10.0; // COM linear kv
	double kpwcom = 20.0; // COM angular kp
	double kvwcom = 10.0; // COM angular kv
	double kpj = 70.0; // joint space kp
	double kvj = 30.0; // joint space kv
	double kpwleftf = 40; // left foot admittance kp for zero moment control
	double kvwleftf = 10; // left foot kv damping
	double kpwrightf = 40; // right foot admittance kp for zero moment control
	double kvwrightf = 10; // right foot kv damping

	// constant parameters
	const double des_com_height_balanced = 0.75; // computed from above

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// get all contact points from the sim
		sim->getContactList(left_foot_point_list, left_foot_force_list, robot_name, left_foot_name);
		sim->getContactList(right_foot_point_list, right_foot_force_list, robot_name, right_foot_name);

		// update model every once in a while
		// TODO: this should be in a separate thread
		robot->updateModel();
		if (timer.elapsedCycles() % 10 == 1) {
			robot->gravityVector(gj, Eigen::Vector3d(3.00, 0.0, 0.0));
			actuated_space_inertia = (robot->_M_inv.block(3,3,act_dof, act_dof)).inverse();
			actuated_space_projection = actuated_space_inertia * (robot->_M_inv.block(3,0,act_dof, dof));
			// cout << robot->_M_inv.block(6,0,robot->dof()-6, robot->dof()) << endl;

			// get the task Jacobians
			// - Jv com: TODO: need to add up the Jacobians for all the links
			// For now, we use the torso frame as a surrogate
			robot->J_0(J0_com, torso_name, Eigen::Vector3d(0.0, 0.0, 0.0));
			Jv_com = J0_com.block(0,0,2,dof);
			robot->J_0(J0_left_foot, left_foot_name, left_foot_pos_local);
			Jv_left_foot = J0_left_foot.block(0,0,2,dof);
			// cout << "Jv_left_foot " << endl << Jv_left_foot << endl;
			robot->J_0(J0_right_foot, right_foot_name, right_foot_pos_local);
			Jv_right_foot = J0_right_foot.block(0,0,2,dof);
			// cout << "Jv_right_foot " << endl << Jv_right_foot << endl;

			// compute estimated feet position in world frame
			robot->position(left_foot_frame_pos_world, left_foot_name, left_foot_pos_local);
			robot->position(right_foot_frame_pos_world, right_foot_name, right_foot_pos_local);

			// J feet tension
			left_to_right_foot_unit_vec = (right_foot_frame_pos_world - left_foot_frame_pos_world);
			left_to_right_foot_unit_vec.normalize();
			J_feet_tension = left_to_right_foot_unit_vec.head(2).transpose()*(Jv_right_foot - Jv_left_foot);

			// contact projections for both feet
			J_c_both << Jv_left_foot, J0_left_foot.block(5,0,1,dof), Jv_right_foot, J0_right_foot.block(5,0,1,dof);
			L_contact_both = (J_c_both*robot->_M_inv*J_c_both.transpose()).inverse();
			N_contact_both = 
				MatrixXd::Identity(dof, dof) - robot->_M_inv*J_c_both.transpose()*L_contact_both*J_c_both;
			SN_contact_both = N_contact_both.block(3, 0, act_dof, dof);
			actuated_space_inertia_contact_inv_both = SN_contact_both*robot->_M_inv*SN_contact_both.transpose();
			actuated_space_inertia_contact_both = pseudoinverse(actuated_space_inertia_contact_inv_both); // pseudo inverse of the inverse which is guaranteed to be singular
			actuated_space_projection_contact_both = robot->_M_inv*SN_contact_both.transpose()*actuated_space_inertia_contact_both;
			feet_null_projector_both.block(0,0,act_dof,act_dof) = actuated_space_inertia_contact_inv_both;
			feet_null_projector_both.block(act_dof,0,3,act_dof) = 
				feet_internal_forces_selection_mat*L_contact_both*J_c_both*(robot->_M_inv.block(0,3,dof,act_dof));
			feet_null_projector_both_inv = pseudoinverse(feet_null_projector_both);
			feet_tension_projector_both_gravity = (L_contact_both*J_c_both*robot->_M_inv).transpose()*feet_internal_forces_selection_mat.row(2).transpose();
			// cout << "J_c_both " << endl << J_c_both << endl;
			// cout << "L_contact_both " << endl << L_contact_both << endl;
			// cout << "robot->_M_inv " << endl << robot->_M_inv << endl;
			// cout << "N_contact_both " << endl << N_contact_both << endl;
			// cout << "J_c_both*N_contact_both " << endl << J_c_both*N_contact_both << endl;
			// cout << "SN_contact_both " << endl << SN_contact_both << endl;
			// cout << "actuated_space_inertia_contact_inv_both " << endl << actuated_space_inertia_contact_inv_both << endl;
			// cout << "actuated_space_inertia_contact_both " << endl << actuated_space_inertia_contact_both << endl;
			// cout << "actuated_space_projection_contact_both " << endl << actuated_space_projection_contact_both << endl;
			// cout << "bar(SNs_both)*SNs_both = Ns_both " << endl << actuated_space_projection_contact_both*SN_contact_both << endl;
			// cout << "feet_null_projector_both " << endl << feet_null_projector_both << endl;
			// cout << "feet_null_projector_both_inv " << endl << feet_null_projector_both_inv << endl;

			// contact projections for left foot only
			J_c_left << Jv_left_foot;
			L_contact_left = (J_c_left*robot->_M_inv*J_c_left.transpose()).inverse();
			N_contact_left = 
				MatrixXd::Identity(dof, dof) - robot->_M_inv*J_c_left.transpose()*L_contact_left*J_c_left;
			SN_contact_left = N_contact_left.block(3, 0, act_dof, dof);
			actuated_space_inertia_contact_inv_left = SN_contact_left*robot->_M_inv*SN_contact_left.transpose();
			actuated_space_inertia_contact_left = actuated_space_inertia_contact_inv_left.inverse();
			actuated_space_projection_contact_left = robot->_M_inv*SN_contact_left.transpose()*actuated_space_inertia_contact_left;

			// contact projections for right foot only
			J_c_right << Jv_right_foot;
			L_contact_right = (J_c_right*robot->_M_inv*J_c_right.transpose()).inverse();
			N_contact_right = 
				MatrixXd::Identity(dof, dof) - robot->_M_inv*J_c_right.transpose()*L_contact_right*J_c_right;
			SN_contact_right = N_contact_right.block(3, 0, act_dof, dof);
			actuated_space_inertia_contact_inv_right = SN_contact_right*robot->_M_inv*SN_contact_right.transpose();
			actuated_space_inertia_contact_right = actuated_space_inertia_contact_inv_right.inverse(); // pseudo inverse of the inverse which is guaranteed to be singular
			actuated_space_projection_contact_right = robot->_M_inv*SN_contact_right.transpose()*actuated_space_inertia_contact_right;

			// update support polygon centroid
			size_t n_points_left = left_foot_point_list.size();
			size_t n_points_right = right_foot_point_list.size();
			if (n_points_left > 0 && n_points_right > 0) {
				pos_support_centroid.setZero();
				pos_support_centroid[1] = (left_foot_frame_pos_world[1] + right_foot_frame_pos_world[1])*0.5;
			}

			// ... for debugging
			// if (n_points) {
			// 	for(auto point: left_foot_point_list) {
			// 		cout << "Left foot point: " << point.transpose() << endl;
			// 	}
			// 	for(auto point: right_foot_point_list) {
			// 		cout << "Right foot point: " << point.transpose() << endl;
			// 	}
			// 	robot->position(left_foot_frame_pos_world, left_foot_name, left_foot_pos_local);
			// 	cout << "Expected Left foot pos: " << left_foot_frame_pos_world.transpose() << endl;
			// 	robot->position(right_foot_frame_pos_world, right_foot_name, right_foot_pos_local);
			// 	cout << "Expected Right foot pos: " << right_foot_frame_pos_world.transpose() << endl;
			// }
		}

		// // update force sensors
		// left_foot_force_sensor->update();
		// // Eigen::Vector3d left_foot_force;
		// // Eigen::Vector3d left_foot_moment;
		// // left_foot_force_sensor->getForce(left_foot_force);
		// // left_foot_force_sensor->getMoment(left_foot_moment);
		// // cout << "Left foot force: " << left_foot_moment.transpose()
		// // 		<< " Left foot moment: " << left_foot_moment.transpose() << endl;
		// right_foot_force_sensor->update();

		// set tau_act
		tau_act.setZero();
		if (curr_state == FSMState::FallingMidAir) {
			// simply compensate for gravity and brace for landing
			q_fall = q_home.tail(act_dof);
			q_fall[0] = q_home[3]*(1 - 2.5*fmax(0.0, fabs(robot->_q[2]) - (5.0/180.0*M_PI)));
			q_fall[3] = -q_fall[0];
			// tau_act = actuated_space_inertia*(-kpj*(robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*(robot->_dq.tail(act_dof)));
			tau_act = actuated_space_inertia*(-kpj*(robot->_q.tail(act_dof) - q_fall) - kvj*(robot->_dq.tail(act_dof)));
			tau_act += actuated_space_projection*gj;
			// check for contact and switch to FSMState::BalancingDoubleStance
			if (left_foot_point_list.size() || right_foot_point_list.size()) { 
				if (balance_counter > BALANCE_COUNT_THRESH) {
					// set default support position before update to be in the center of the projections of
					// both feet
					pos_support_centroid << 0.0, (left_foot_frame_pos_world[1] + right_foot_frame_pos_world[1])*0.5, 0.0;
					// reset the stable balance counter
					stable_counter = 0;
					// if (fabs(robot->_q[2]) > 7.0/180.0*M_PI) {
					// 	//^^ poor posture, switch to single support stance
					// 	curr_state = FSMState::BalancingSingleStance;
					// 	cout << "Switching from FallingMidAir to BalancingSingleStance" << endl;	
					// } else {
						curr_state = FSMState::BalancingDoubleStance;
						cout << "Switching from FallingMidAir to BalancingDoubleStance" << endl;
					// }
				} else {
					balance_counter++;
				}
			} else {
				balance_counter = 0;
			}
		}

		if (curr_state == FSMState::BalancingSingleStance) {
			balance_counter = BALANCE_COUNT_THRESH;
			// TODO: start zero tension, COM stabilization and posture control to q_home
			robot->position(com_pos, torso_name, Vector3d(0.0,0.0,0.0));

			// - set the task Jacobian, project through the contact null-space
			J_task.setZero(2, dof);
			J_task = Jv_com;
			Eigen::VectorXd task_v = J_task*robot->_dq;
			Eigen::VectorXd task_pos_err(2);
			task_pos_err << 0.0, (com_pos[1] - pos_support_centroid[1]);

			// - compute task forces
			if(left_foot_point_list.size()) {
				J_task = J_task * N_contact_left;
				L_task = (J_task*robot->_M_inv*J_task.transpose()).inverse();
				F_task = L_task*(- 4.0*kplcom*task_pos_err - kvlcom*task_v + J_task*robot->_M_inv*N_contact_left.transpose()*gj);
				null_actuated_space_projection_contact = 
				MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection_contact_left.transpose()*J_task.transpose()*L_task*J_task*robot->_M_inv*SN_contact_left.transpose();

				// - compute required joint torques
				tau_act = actuated_space_projection_contact_left.transpose()*(J_task.transpose()*F_task);
				tau_act += null_actuated_space_projection_contact*(
					actuated_space_projection_contact_left.transpose()*gj + 
					actuated_space_inertia_contact_left*(-kpj*4.0 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*2.0 * robot->_dq.tail(act_dof))
				);
			}
			else if(right_foot_point_list.size()) {
				J_task = J_task * N_contact_right;
				L_task = (J_task*robot->_M_inv*J_task.transpose()).inverse();
				F_task = L_task*(- 4.0*kplcom*task_pos_err - kvlcom*task_v + J_task*robot->_M_inv*N_contact_right.transpose()*gj);
				null_actuated_space_projection_contact = 
				MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection_contact_right.transpose()*J_task.transpose()*L_task*J_task*robot->_M_inv*SN_contact_right.transpose();

				// - compute required joint torques
				tau_act = actuated_space_projection_contact_right.transpose()*(J_task.transpose()*F_task);
				tau_act += null_actuated_space_projection_contact*(
					actuated_space_projection_contact_right.transpose()*gj + 
					actuated_space_inertia_contact_right*(-kpj*4.0 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*2.0 * robot->_dq.tail(act_dof))
				);
			}
			
			
			// cout << "J_task " << endl << J_task << endl;
			// cout << "L_task " << endl << L_task << endl;
			// cout << "com_v " << endl << com_v << endl;
			// cout << "com_pos_err " << endl << com_pos_err << endl;
			// cout << "F_task " << endl << F_task << endl;

			// TODO: if double contact is achieved, switch to falling mid air temporarily which should automatically switch
			// to BalancingDoubleStance with safe control torques for the initial balance
			// if (left_foot_point_list.size() && right_foot_point_list.size()) {
			if (left_foot_frame_pos_world[0] > -0.05 && right_foot_frame_pos_world[0] > -0.05) {
				curr_state = FSMState::BalancingDoubleStance;
				cout << "Switching from BalancingSingleStance to BalancingDoubleStance" << endl;
			}
		}

		if (curr_state == FSMState::BalancingDoubleStance) {
			// TODO: start zero tension, COM stabilization and posture control to q_home
			robot->position(com_pos, torso_name, Vector3d(0.0,0.0,0.0));

			// - set the task Jacobian, project through the contact null-space
			J_task.setZero(3, dof);
			J_task.block(0,0,2,dof) = Jv_com; // linear part
			J_task(2,2) = 1; // angular part
			J_task = J_task * N_contact_both;
			
			// - compute task forces
			com_v << Jv_com*robot->_dq, (robot->_dq)[2];
			// cout << " Desired zmp pos " << pos_support_centroid[1] << endl;
			com_pos_err << (com_pos[0] + des_com_height_balanced), (com_pos[1] - pos_support_centroid[1]), robot->_q[2];
			//TODO: separate linear and angular parts in task force below
			L_task = (J_task*robot->_M_inv*J_task.transpose()).inverse();
			F_task = L_task*(- kplcom*com_pos_err - kvlcom*com_v + J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			F_task_passive = L_task*(J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			// cout << "J_task " << endl << J_task << endl;
			// cout << "L_task " << endl << L_task << endl;
			// if (L_task(2,2) < 0.001) {
			// 	f_global_sim_pause = true;
			// 	cout << "Global pause " << endl;
			// }
			// cout << "com_v " << endl << com_v << endl;
			// cout << "com_pos_err " << endl << com_pos_err << endl;
			// cout << "F_task " << endl << F_task << endl;

			// - compute posture torques
			null_actuated_space_projection_contact = 
				MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection_contact_both.transpose()*J_task.transpose()*L_task*J_task*robot->_M_inv*SN_contact_both.transpose();

			// - compute required joint torques
			tau_act = actuated_space_projection_contact_both.transpose()*(J_task.transpose()*F_task);
			// tau_act += null_actuated_space_projection_contact*(actuated_space_projection_contact_both.transpose()*gj);
			tau_act += null_actuated_space_projection_contact*(
				actuated_space_projection_contact_both.transpose()*gj + 
				actuated_space_inertia_contact_both*(-kpj*4.0 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*2.0 * robot->_dq.tail(act_dof))
			);
			// tau_act = actuated_space_projection_contact_both.transpose()*( - robot->_M*kvj*(robot->_dq));

			// remove moments at the feet
			Eigen::VectorXd tau_0(act_dof);
			Eigen::VectorXd tau_0_rhs(act_dof+3);
			tau_0_rhs.head(act_dof) = actuated_space_inertia_contact_inv_both*tau_act;
			tau_0_rhs(act_dof) = 0; // left foot moment
			tau_0_rhs(act_dof+1) = 0; // right foot moment
			tau_0_rhs(act_dof+2) = 30 + feet_tension_projector_both_gravity.dot(gj); // internal tension between feet = 0
			tau_0 = feet_null_projector_both_inv*tau_0_rhs;
			// cout << "(SNs)^T*tau_act " << ((SN_contact_both).transpose()*tau_act).transpose() << endl;
			// cout << "(SNs)^T*tau_0 " << ((SN_contact_both).transpose()*tau_0).transpose() << endl;
			tau_act = tau_0;
			
			// - compute passive joint torques in case slippage/stickage is detected
			tau_act_passive = actuated_space_projection_contact_both.transpose()*(J_task.transpose()*F_task_passive);
			// tau_act += null_actuated_space_projection_contact*(actuated_space_projection_contact_both.transpose()*gj);
			tau_act_passive += null_actuated_space_projection_contact*(
				actuated_space_projection_contact_both.transpose()*gj + 
				actuated_space_inertia_contact_both*(-kpj*4.0 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*2.0 * robot->_dq.tail(act_dof))
			);
			// tau_act = actuated_space_projection_contact_both.transpose()*( - robot->_M*kvj*(robot->_dq));

			// remove moments at the feet
			tau_0_rhs.head(act_dof) = actuated_space_inertia_contact_inv_both*tau_act_passive;
			tau_0_rhs(act_dof) = 0; // left foot moment
			tau_0_rhs(act_dof+1) = 0; // right foot moment
			tau_0_rhs(act_dof+2) = 30 + feet_tension_projector_both_gravity.dot(gj); // internal tension between feet = 0
			tau_0 = feet_null_projector_both_inv*tau_0_rhs;
			// cout << "(SNs)^T*tau_act " << ((SN_contact_both).transpose()*tau_act).transpose() << endl;
			// cout << "(SNs)^T*tau_0 " << ((SN_contact_both).transpose()*tau_0).transpose() << endl;
			tau_act_passive = tau_0;

			// if COM velocity has been zero for a while, switch to FSMState::Jumping
			// if (left_foot_point_list.size() && right_foot_point_list.size() && com_v.array().abs().maxCoeff() < 5e-3) {
			if ((left_foot_point_list.size() || right_foot_point_list.size()) && com_v.array().abs().maxCoeff() < 5e-3) {
				if (stable_counter > STABLE_COUNT_THRESH) {
					curr_state = FSMState::Jumping;
					cout << "Switching from BalancingDoubleStance to Jumping" << endl;
					// reset jump counter
					jump_counter = 0;
				}
				else {
					stable_counter++;
				}
			} else {
				stable_counter = 0;
			}

			// check friction cone constraint only if in two point support. Ok to slip a bit in single point support
			// if (left_foot_point_list.size() || right_foot_point_list.size()) {
			// ^^ this check does not work because it is possible that the robot is in contact at the next time step
			// and so it might slip
				temp_tau.setZero(dof);
				temp_tau.tail(act_dof) = tau_act;
				bool is_sticking = false;
				bool is_slipping = false;
				// if (left_foot_point_list.size() && right_foot_point_list.size()) {
					F_contact = -L_contact_both*J_c_both*robot->_M_inv*(temp_tau - gj);
					if (F_contact[0] > 0 || F_contact[3] > 0) {
						is_sticking = true;
					}
					else if (fabs(F_contact[1]/F_contact[0]) > 0.9 || fabs(F_contact[4]/F_contact[3]) > 0.9) {
						is_slipping = true;
					}
				// } else if (left_foot_point_list.size()) {
					// F_contact = -L_contact_left*J_c_left*robot->_M_inv*(temp_tau - gj);
					// if (F_contact[0] > -0.1) {
					// 	is_sticking = true;
					// }
					// else if (fabs(F_contact[1]/F_contact[0]) > 0.8) {
					// 	is_slipping = true;
					// }
				// } else if (right_foot_point_list.size()) {
					// F_contact = -L_contact_right*J_c_right*robot->_M_inv*(temp_tau - gj);
					// if (F_contact[0] > -0.1) {
					// 	is_sticking = true;
					// }
					// else if (fabs(F_contact[1]/F_contact[0]) > 0.8) {
					// 	is_slipping = true;
					// }
				// }
				if (is_sticking) {
					// cout << "Exceeded normal force " << F_contact.transpose() << endl;
					// if (left_foot_force_list.size()) {
					// 	cout << "Actual left foot contact forces " << left_foot_force_list[0].transpose() << endl;
					// }
					// if (right_foot_force_list.size()) {
					// 	cout << "Actual right foot contact forces " << right_foot_force_list[0].transpose() << endl;
					// }
					// tau_act.setZero(act_dof); //play safe
					tau_act = tau_act_passive;
				}
				else if (is_slipping) {
					// cout << "Slip detected " << F_contact.transpose() << endl;
					// tau_act.setZero(act_dof); //play safe
					tau_act = tau_act_passive;
				}
			// }
			// cout << "tau_act " << endl << tau_act << endl;
		}

		if (curr_state == FSMState::Jumping) {
			Eigen::Vector3d com_desired_velocity; // vx, vy, w
			float inter_foot_distance = fabs(right_foot_frame_pos_world[1] - left_foot_frame_pos_world[1]);
			com_desired_velocity << -4.0*fmin(1.0, 0.6/inter_foot_distance), -0.05*pos_support_centroid[1], 0.0;
			// cout << "inter foot distance " << inter_foot_distance << endl;
			// cout << "com_desired_velocity " << com_desired_velocity.transpose() << endl;
			// TODO: start accelerating COM upwards while maintaining tension between feet
			// NOTE: simply accelerating upwards is not a good idea. Instead, 
			// we might need to bend the knees first to lower the COM, and then accelerate it upwards
			J_task.setZero(3, dof);
			J_task.block(0,0,2,dof) = Jv_com; // linear part
			J_task(2,2) = 1; // angular part
			J_task = J_task * N_contact_both;
			// cout << "J_task " << endl << J_task << endl;
			
			// - compute task forces
			L_task = (J_task*robot->_M_inv*J_task.transpose()).inverse();
			// cout << "L_task " << endl << L_task << endl;
			robot->position(com_pos, torso_name, Vector3d(0.0,0.0,0.0));
			com_v << Jv_com*robot->_dq, (robot->_dq)[2];
			// cout << "com_v " << endl << com_v << endl;
			//TODO: separate linear and angular parts in task force below
			F_task = L_task*( - kvlcom*(com_v - com_desired_velocity) + J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			// cout << "F_task " << endl << F_task << endl;
			if (robot->_q[4] < 0 || robot->_q[7] > 0) {
				// only compensate for gravity beyond this point
				F_task = L_task*(J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			}

			// - compute posture torques
			null_actuated_space_projection_contact = 
				MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection_contact_both.transpose()*J_task.transpose()*L_task*J_task*robot->_M_inv*SN_contact_both.transpose();

			// - compute required joint torques
			tau_act = actuated_space_projection_contact_both.transpose()*(J_task.transpose()*F_task);
			tau_act += null_actuated_space_projection_contact*(
				actuated_space_projection_contact_both.transpose()*gj + 
				actuated_space_inertia_contact_both*(-kpj*4.0 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*2.0 * robot->_dq.tail(act_dof))
			);

			// remove moments at the feet
			Eigen::VectorXd tau_0(act_dof);
			Eigen::VectorXd tau_0_rhs(act_dof+3);
			tau_0_rhs.head(act_dof) = actuated_space_inertia_contact_inv_both*tau_act;
			tau_0_rhs(act_dof) = 0; // left foot moment
			tau_0_rhs(act_dof+1) = 0; // right foot moment
			tau_0_rhs(act_dof+2) = 0 + feet_tension_projector_both_gravity.dot(gj); // internal tension between feet = 0
			tau_0 = feet_null_projector_both_inv*tau_0_rhs;
			// cout << "(SNs)^T*tau_act " << ((SN_contact_both).transpose()*tau_act).transpose() << endl;
			// cout << "(SNs)^T*tau_0 " << ((SN_contact_both).transpose()*tau_0).transpose() << endl;
			tau_act = tau_0;
			// cout << "tau_act jump" << tau_act.transpose() << endl;
			// Force ankle torques to zero if they still are not so. this can be the case close to singularity
			tau_act[2] = 0.0;
			tau_act[5] = 0.0;

			// TODO: might need to stop applying forces when we reach the task singularity
			
			// TODO: when contact is lost, switch to FSMState::FallingMidAir
			if (left_foot_point_list.size() == 0 && right_foot_point_list.size() == 0) {
				if (jump_counter > JUMP_COUNT_THRESH) {
					curr_state = FSMState::FallingMidAir;
					cout << "Switching from Jumping to FallingMidAir" << endl;
					num_successful_jumps++;
					cout << "Num jumps: " << num_successful_jumps << endl;
					balance_counter = 0;
				}
				else {
					jump_counter++;
				}
			} else {
				jump_counter = 0;
			}
		}

		// // assemble full tau vector for simulation
		// cout << tau_act.transpose() << endl;
		// if (isnan(tau_act.array()).maxCoeff() > 0) {
		// 	// << this does not stop the simulation from blowing up. probably because the torque values are very high even before being NaN
		// 	// TODO: handle better
		// 	tau_act.setZero(act_dof);
		// 	cout << "nan torques" << endl;
		// }

		tau.tail(act_dof) = tau_act;
		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
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
		if (!f_global_sim_pause) {
			sim->integrate(loop_dt);
		}

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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - 2D 8bar Jump Sim", NULL, NULL);
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

//------------------------------------------------------------------------------
MatrixXd pseudoinverse(const MatrixXd &mat, double tolerance) // choose appropriately
{
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    MatrixXd singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = 1.0 / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = 0.0;
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}
