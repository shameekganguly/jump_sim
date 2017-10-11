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
#include "model/RBDLModel.h"
#include "graphics/ChaiGraphics.h"
#include "simulation/Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;

const string world_fname = "resources/jump1/world.urdf";
const string robot_fname = "../resources/toro/toro_headless.urdf";
const string robot_name = "Toro";
string camera_name = "camera_side";
// string camera_name = "camera_isometric";
// string camera_name = "camera_front";
// string camera_name = "camera_top";
// string ee_link_name = "link6";

// contact link names
const string right_foot_name = "RL_foot";
const string left_foot_name = "LL_foot";
const string torso_name = "hip_base";

static const string arr[] = {
	"hip_base",
	"RL_KOSY_L1",
	"RL_KOSY_L23",
	"RL_KOSY_L4",
	"RL_KOSY_L56",
	"RL_ankle",
	"RL_foot",
	"LL_KOSY_L1",
	"LL_KOSY_L23",
	"LL_KOSY_L4",
	"LL_KOSY_L56",
	"LL_ankle",
	"LL_foot",
	"trunk",
	"ra_link1",
	"ra_link2",
	"ra_link3",
	"ra_link4",
	"ra_link5",
	"ra_link6",
	"la_link1",
	"la_link2",
	"la_link3",
	"la_link4",
	"la_link5",
	"la_link6",
	"neck_link1",
	// "neck_link2"
};
vector<string> link_names (arr, arr + sizeof(arr) / sizeof(arr[0]) );

// std::vector<std::string> link_names ();

// global variables
Eigen::VectorXd q_home;
ForceSensorSim* left_foot_force_sensor;
ForceSensorDisplay* left_foot_force_display;
ForceSensorSim* right_foot_force_sensor;
ForceSensorDisplay* right_foot_force_display;

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Model::RBDLModel* robot_rbdl, Simulation::Sai2Simulation* sim);
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);

bool f_global_sim_pause = false; // use with caution!
// bool f_global_sim_pause = true; // use with caution!

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// util function to calculate the pseudo inverse
MatrixXd pseudoinverse(const MatrixXd &mat, double tolerance = 1e-4);

// util 
void comPosition(Vector3d& robot_com, Model::RBDLModel* robot, const Eigen::VectorXd& q, const std::vector<std::string> link_names);
void comJacobian(MatrixXd& ret_J0, Model::RBDLModel* robot, const Eigen::VectorXd& q, const std::vector<std::string> link_names);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);
	graphics->_world->setBackgroundColor(0.7, 0.7, 0.5);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);
	auto robot_rbdl = dynamic_cast<Model::RBDLModel *>(robot->_model_internal);

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
				1.2,	//2 floating_base_pz
				0.0/180.0*M_PI, //3 floating_base_rx
				0.0/180.0*M_PI, //4 floating_base_ry
				0.0/180.0*M_PI,	//5 floating_base_rz
				-30.0/180.0*M_PI,	//6 right thigh adduction
				-45.0/180.0*M_PI,	//7 right thigh pitch
				-15/180.0*M_PI,	//8 right knee roll
				82/180.0*M_PI,	//9 right knee pitch
				15/180.0*M_PI,	//10 right ankle adduction
				-15/180.0*M_PI,	//11 right ankle pitch
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
				// 30/180.0*M_PI,	//26 neck pitch
				30/180.0*M_PI,	//27 left thigh adduction
				-45/180.0*M_PI,	//28 left thigh pitch
				15/180.0*M_PI,	//29 left knee roll
				82/180.0*M_PI,	//30 left knee pitch
				-15/180.0*M_PI,	//31 left ankle adduction
				-15/180.0*M_PI;	//32 left ankle pitch
	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;
	// Eigen::Vector3d ll_pos, hip_pos;
	// robot->position(ll_pos, left_foot_name, Eigen::Vector3d(0.0,0.0,0.0));
	// robot->position(hip_pos, torso_name, Eigen::Vector3d(0.0,0.0,0.0));
	// cout << "Hip leg displacement " << (hip_pos - ll_pos).transpose() << endl;
	// NOTE: The above code computes the nominal hip height in the balanced position

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot, sim);

	// initialize force sensor: needs Sai2Simulation sim interface type
	left_foot_force_sensor = new ForceSensorSim(robot_name, left_foot_name, Eigen::Affine3d::Identity(), sim, robot);
	// left_foot_force_display = new ForceSensorDisplay(left_foot_force_sensor, graphics);
	right_foot_force_sensor = new ForceSensorSim(robot_name, right_foot_name, Eigen::Affine3d::Identity(), sim, robot);
	// right_foot_force_display = new ForceSensorDisplay(right_foot_force_sensor, graphics);

	// next start the control thread
	thread ctrl_thread(control, robot, robot_rbdl, sim);
	
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
void control(Model::ModelInterface* robot, Model::RBDLModel* robot_rbdl, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime(); //secs

	// controller state variables
	enum FSMState {
		Balancing,
		Jumping,
		FallingMidAir
	};
	FSMState curr_state = FSMState::FallingMidAir;

	// dof counts
	int dof = robot->dof();
	int act_dof = robot->dof() - 6; // 3D free base

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd tau_act(act_dof);
	Eigen::VectorXd temp_tau(dof);
	Eigen::VectorXd tau_act_passive(act_dof);
	Eigen::VectorXd gj(robot->dof());
	Eigen::MatrixXd selection_mat(act_dof, robot->dof());
	// cout << selection_mat << endl;
	Eigen::MatrixXd actuated_space_projection;
	actuated_space_projection.setZero(act_dof, robot->dof());
	Eigen::MatrixXd actuated_space_inertia;
	actuated_space_inertia.setZero(act_dof, act_dof);
	Eigen::MatrixXd null_actuated_space_projection_contact;
	null_actuated_space_projection_contact.setZero(act_dof, act_dof);

	Eigen::MatrixXd actuated_space_projection_contact_both;
	actuated_space_projection_contact_both.setZero(act_dof, dof);
	Eigen::MatrixXd actuated_space_inertia_contact_both;
	actuated_space_inertia_contact_both.setZero(act_dof, act_dof);
	Eigen::MatrixXd actuated_space_inertia_contact_inv_both;
	actuated_space_inertia_contact_inv_both.setZero(act_dof, act_dof);
	Eigen::MatrixXd feet_internal_forces_selection_mat;
	feet_internal_forces_selection_mat.setZero(6,12);
	feet_internal_forces_selection_mat(0,3) = 1; // left foot x moment
	feet_internal_forces_selection_mat(1,5) = 1; // left foot z moment
	feet_internal_forces_selection_mat(2,9) = 1; // right foot x moment
	feet_internal_forces_selection_mat(3,11) = 1; // right foot z moment
	feet_internal_forces_selection_mat(4,1) = -1; // internal tension. default is in y direction
	feet_internal_forces_selection_mat(4,7) = 1; // internal tension
	feet_internal_forces_selection_mat(5,4) = -1; // internal plane torsion
	feet_internal_forces_selection_mat(5,10) = 1; // internal plane torsion (about y)
	Eigen::MatrixXd feet_null_projector_both;
	feet_null_projector_both.setZero(act_dof+6, act_dof);
	Eigen::VectorXd feet_internal_force_projected_both_gravity(6);
	Eigen::MatrixXd feet_null_projector_both_inv(act_dof, act_dof+6);
	Eigen::MatrixXd J_c_both(12, robot->dof());
	Eigen::MatrixXd L_contact_both(12, 12);
	Eigen::MatrixXd N_contact_both(dof, dof);
	Eigen::MatrixXd SN_contact_both(act_dof, dof);
	Eigen::VectorXd F_contact;

	std::vector<Eigen::Vector3d> left_foot_force_list;
	std::vector<Eigen::Vector3d> left_foot_point_list;
	std::vector<Eigen::Vector3d> right_foot_force_list;
	std::vector<Eigen::Vector3d> right_foot_point_list;

	Eigen::MatrixXd J0_com(6, robot->dof());
	Eigen::MatrixXd Jv_com(3, robot->dof());
	Eigen::MatrixXd Jw_torso(3, robot->dof());
	Eigen::Matrix3d rot_torso;
	Eigen::MatrixXd Jw_left_foot(3, robot->dof());
	Eigen::MatrixXd Jw_right_foot(3, robot->dof());
	Eigen::Matrix3d rot_left_foot;
	Eigen::Matrix3d rot_right_foot;
	Eigen::MatrixXd J0_left_foot(6, robot->dof());
	Eigen::MatrixXd J0_right_foot(6, robot->dof());
	Eigen::MatrixXd Jv_left_foot(3, robot->dof());
	Eigen::MatrixXd Jv_right_foot(3, robot->dof());
	const Vector3d left_foot_pos_local(0.02, 0.0, -0.05);
	const Vector3d right_foot_pos_local(0.02, 0.0, -0.05);
	Eigen::Vector3d left_foot_frame_pos_world; // position of left foot frame
	Eigen::Vector3d right_foot_frame_pos_world; // position of right foot frame
	Eigen::MatrixXd J_feet_tension(1, robot->dof());
	double feet_distance;
	Eigen::Vector3d left_to_right_foot_unit_vec;
	Eigen::Vector3d pos_support_centroid;

	Eigen::MatrixXd J_task;
	Eigen::MatrixXd L_task;
	Eigen::VectorXd F_task;
	Eigen::VectorXd F_task_passive;
	Eigen::Vector3d com_pos;
	Eigen::Vector3d com_pos_err;
	Eigen::Vector3d torso_ang_err;
	Eigen::Vector3d com_v;
	Eigen::Vector3d torso_ang_v;

	MatrixXd J_inter_foot_distance(1, dof);
	MatrixXd J_com_zmp_x_err(1, dof);
	MatrixXd J_fall_task (8, dof);
	MatrixXd L_fall_task (8, 8);
	MatrixXd M_inv_sel (dof, act_dof);
	MatrixXd N_fall_task(act_dof, act_dof);
	Vector3d left_foot_ang_err;
	Vector3d right_foot_ang_err;
	double com_zmp_x_err;
	VectorXd fall_task_err_acc(8);
	VectorXd fall_lin_task_err(2);
	VectorXd fall_ang_task_err(6);
	VectorXd fall_task_vel(8);
	VectorXd F_fall_task(8);

	Eigen::Matrix3d rot_torso_des = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d rot_left_foot_des = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d rot_right_foot_des = Eigen::Matrix3d::Identity();
	robot->rotation(rot_left_foot_des, left_foot_name);
	robot->rotation(rot_right_foot_des, right_foot_name);

	uint balance_counter = 0;
	const uint BALANCE_COUNT_THRESH = 11;
	uint stable_counter;
	const uint STABLE_COUNT_THRESH = 31;
	uint jump_counter;
	const uint JUMP_COUNT_THRESH = 5;

	// gains
	double kplcom = 15.0; // COM linear kp
	double kvlcom = 5.0; // COM linear kv
	double kpacom = 15.0; // COM angular kp
	double kvacom = 5.0; // COM angular kv
	double kpj = 70.0; // joint space kp
	double kvj = 30.0; // joint space kv
	double kpwleftf = 40; // left foot admittance kp for zero moment control
	double kvwleftf = 10; // left foot kv damping
	double kpwrightf = 40; // right foot admittance kp for zero moment control
	double kvwrightf = 10; // right foot kv damping
	double kpfeetdistance = 50; // gain to map internal tension between feet from distance between them

	// constant parameters
	const double des_com_height_balanced = 0.86; // computed from above

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// check if paused
		if (f_global_sim_pause) { continue;}

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// get all contact points from the sim
		sim->getContactList(left_foot_point_list, left_foot_force_list, robot_name, left_foot_name);
		sim->getContactList(right_foot_point_list, right_foot_force_list, robot_name, right_foot_name);

		// some times there is a bug where the point list does not get updated correctly. So we have
		// to add a hack to ensure that the list is empty when we are sure that the robot is not on the 
		// ground
		if (right_foot_frame_pos_world[2] > 0.1 && left_foot_frame_pos_world[2] > 0.1) {
			left_foot_point_list.clear();
			left_foot_force_list.clear();
			right_foot_point_list.clear();
			right_foot_force_list.clear();
		}

		// update com, zmp and foot position/velocity all the time
		if (true) {
			// COM position
			comPosition(com_pos, robot_rbdl, robot->_q, link_names);

			// foot positions, rotations
			robot->position(left_foot_frame_pos_world, left_foot_name, left_foot_pos_local);
			robot->rotation(rot_left_foot, left_foot_name);
			// cout << "rot_left_foot " << endl << rot_left_foot << endl;
			robot->position(right_foot_frame_pos_world, right_foot_name, right_foot_pos_local);
			robot->rotation(rot_right_foot, right_foot_name);

			// inter foot distance, direction vector
			left_to_right_foot_unit_vec = (right_foot_frame_pos_world - left_foot_frame_pos_world);
			feet_distance = left_to_right_foot_unit_vec.norm();
			left_to_right_foot_unit_vec.normalize();
			// cout << "left_to_right_foot_unit_vec " << left_to_right_foot_unit_vec.transpose() << endl;
			// cout << "feet_distance " << feet_distance << endl;

			// update support polygon centroid
			pos_support_centroid.setZero();
			// TODO: use computed convex hull when actually in contact
			pos_support_centroid[1] = (left_foot_frame_pos_world[1] + right_foot_frame_pos_world[1])*0.5;
			pos_support_centroid[0] = (left_foot_frame_pos_world[0] + right_foot_frame_pos_world[0])*0.5;
			// cout << "Estimated ZMP position: " << pos_support_centroid.transpose() << endl;
			// cout << "Actual COM position: " << com_pos.transpose() << endl;
		}

		// update model every once in a while
		// TODO: this should be in a separate thread
		if (timer.elapsedCycles() % 10 == 1) {
			robot->updateModel();
			robot->gravityVector(gj, Eigen::Vector3d(0.0, 0.0, -3.00));

			// free space inertial projections
			actuated_space_inertia = (robot->_M_inv.block(6,6,robot->dof()-6, robot->dof()-6)).inverse();
			actuated_space_projection = actuated_space_inertia * (robot->_M_inv.block(6,0,robot->dof()-6, robot->dof()));
			// cout << robot->_M_inv.block(6,0,robot->dof()-6, robot->dof()) << endl;

			// get the task Jacobians
			// - com Jv
			comJacobian(J0_com, robot_rbdl, robot->_q, link_names);
			Jv_com = J0_com.block(0,0,3,dof);

			// - Jv, Jw at feet
			robot->J_0(J0_left_foot, left_foot_name, left_foot_pos_local);
			Jv_left_foot = J0_left_foot.block(0,0,3,dof);
			Jw_left_foot = J0_left_foot.block(3,0,3,dof);
			// cout << "Jv_left_foot " << endl << Jv_left_foot << endl;
			robot->J_0(J0_right_foot, right_foot_name, right_foot_pos_local);
			Jv_right_foot = J0_right_foot.block(0,0,3,dof);
			Jw_right_foot = J0_right_foot.block(3,0,3,dof);

			// - J feet tension
			J_feet_tension = left_to_right_foot_unit_vec.transpose()*(Jv_right_foot - Jv_left_foot);
			// cout << "feet_internal_forces_selection_mat " << endl << feet_internal_forces_selection_mat << endl;

			// update torso angular parameters
			robot->Jw(Jw_torso, torso_name);
			// cout << "Jw_torso " << endl << Jw_torso << endl;
			robot->rotation(rot_torso, torso_name);

			// update feet_internal_forces_selection_mat with currect inter-foot direction vector
			if (feet_distance > 0.1) {
				feet_internal_forces_selection_mat.block(4,0,1,3) = left_to_right_foot_unit_vec.transpose();
				feet_internal_forces_selection_mat.block(4,6,1,3) = -left_to_right_foot_unit_vec.transpose();
				feet_internal_forces_selection_mat.block(5,3,1,3) = left_to_right_foot_unit_vec.transpose();
				feet_internal_forces_selection_mat.block(5,9,1,3) = -left_to_right_foot_unit_vec.transpose();
			}

			// contact projections for both feet
			J_c_both << J0_left_foot, J0_right_foot;
			L_contact_both = (J_c_both*robot->_M_inv*J_c_both.transpose()).inverse();
			N_contact_both = 
				MatrixXd::Identity(dof, dof) - robot->_M_inv*J_c_both.transpose()*L_contact_both*J_c_both;
			SN_contact_both = N_contact_both.block(6, 0, act_dof, dof);
			actuated_space_inertia_contact_inv_both = SN_contact_both*robot->_M_inv*SN_contact_both.transpose();
			actuated_space_inertia_contact_both = pseudoinverse(actuated_space_inertia_contact_inv_both); // pseudo inverse of the inverse which is guaranteed to be singular
			actuated_space_projection_contact_both = robot->_M_inv*SN_contact_both.transpose()*actuated_space_inertia_contact_both;
			feet_null_projector_both.block(0,0,act_dof,act_dof) = actuated_space_inertia_contact_inv_both;
			feet_null_projector_both.block(act_dof,0,6,act_dof) = 
				feet_internal_forces_selection_mat*L_contact_both*J_c_both*(robot->_M_inv.block(0,6,dof,act_dof));
			feet_null_projector_both_inv = pseudoinverse(feet_null_projector_both);
			feet_internal_force_projected_both_gravity = feet_internal_forces_selection_mat*(L_contact_both*J_c_both*robot->_M_inv)*gj;
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

			// update fall task dynamics
			J_inter_foot_distance = J_feet_tension;
			J_com_zmp_x_err = Vector3d(1.0, 0, 0).transpose()*((Jv_left_foot + Jv_right_foot)*0.5 - Jv_com);
			// TODO: ^^ fix x vector above to be in plane perpendicular to the feet vector
			J_fall_task << J_inter_foot_distance, J_com_zmp_x_err, Jw_left_foot, Jw_right_foot;
			// cout << "J_fall_task " << endl << J_fall_task << endl;
			M_inv_sel = robot->_M_inv.block(0,6,dof, act_dof);
			L_fall_task = (J_fall_task*M_inv_sel*actuated_space_inertia*M_inv_sel.transpose()*J_fall_task.transpose()).inverse();
			// cout << "L_fall_task " << endl << L_fall_task << endl;
			N_fall_task =
				MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection*J_fall_task.transpose()*L_fall_task*J_fall_task*M_inv_sel;

			// update double stance task dynamics
			// - set the task Jacobian, project through the contact null-space
			J_task.setZero(6, dof);
			// J_task.setZero(3, dof);
			J_task << Jv_com, Jw_torso;
			// J_task << Jv_com;
			// cout << "Jv_com " << endl << Jv_com << endl;
			J_task = J_task * N_contact_both;
			L_task = (J_task*robot->_M_inv*J_task.transpose()).inverse();
			// cout << "J_task " << endl << J_task << endl;
			// cout << "L_task_inv " << endl << J_task*robot->_M_inv*J_task.transpose() << endl;
			// cout << "L_task " << endl << L_task << endl;
			null_actuated_space_projection_contact =
				MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection_contact_both.transpose()*J_task.transpose()*L_task*J_task*robot->_M_inv*SN_contact_both.transpose();
		}

		// pause simulation if com velocity exceeds threshold
		if ((Jv_com*robot->_dq).norm() > 2 && curr_state == FSMState::Balancing) {
		cout << "Global pause as COM speed exceeded " << endl;
			f_global_sim_pause = true;
			continue;
		}

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
		tau_act.setZero();
		if (curr_state == FSMState::FallingMidAir) {
			// calculate task errors
			const double des_inter_foot_distance = 0.75;
			com_zmp_x_err = pos_support_centroid[0] - com_pos[0]; //TODO: account for yaw rotation
			robot->orientationError(left_foot_ang_err, rot_left_foot_des, rot_left_foot);
			robot->orientationError(right_foot_ang_err, rot_right_foot_des, rot_right_foot);

			// calculate required task force
			fall_lin_task_err << (feet_distance - des_inter_foot_distance), com_zmp_x_err;
			fall_ang_task_err << left_foot_ang_err, right_foot_ang_err;
			fall_task_err_acc << -20.0*fall_lin_task_err, -10.0*fall_ang_task_err;
			// cout << "fall task_err " << lin_task_err.transpose() << " " << ang_task_err.transpose() << endl;
			fall_task_vel = J_fall_task*robot->_dq;
			F_fall_task = L_fall_task*(fall_task_err_acc + -10.0*(fall_task_vel) + J_fall_task*robot->_M_inv*gj);
			// cout << "fall F_fall_task " << F_fall_task.transpose() << endl;

			// calculate required actuator torques
			tau_act = actuated_space_projection*(J_fall_task.transpose()*F_fall_task);
			tau_act += N_fall_task*(
				actuated_space_projection*gj +
				actuated_space_inertia*(-kpj*4*(robot->_q.tail(act_dof)- q_home.tail(act_dof)) - kvj*1*(robot->_dq.tail(act_dof)))
			);

			// // simply compensate for gravity and brace for landing
			// tau_act = actuated_space_inertia*(-kpj*2*(robot->_q.tail(act_dof)- q_home.tail(act_dof)) - kvj*(robot->_dq.tail(act_dof)));
			// tau_act += actuated_space_projection*gj;

			// check for contact and switch to FSMState::Balancing
			if ((left_foot_point_list.size() || right_foot_point_list.size()) && robot->_dq[2] < 0) {
				if (balance_counter > BALANCE_COUNT_THRESH) {
					// reset the stable balance counter
					stable_counter = 0;
					// if (fabs(robot->_q[2]) > 7.0/180.0*M_PI) {
					// 	//^^ poor posture, switch to single support stance
					// 	curr_state = FSMState::BalancingSingleStance;
					// 	cout << "Switching from FallingMidAir to BalancingSingleStance" << endl;	
					// } else {
						curr_state = FSMState::Balancing;
						cout << "Switching from FallingMidAir to Balancing" << endl;
					// cout << "Landing: Estimated ZMP position: " << pos_support_centroid.transpose() << endl;
					// cout << "Landing: Actual COM position: " << com_pos.transpose() << endl;
					// }
				} else {
					balance_counter++;
				}
			} else {
				balance_counter = 0;
			}
		}

		if (curr_state == FSMState::Balancing) {
			// start zero tension, COM stabilization and posture control to q_home

			// - compute task forces
			com_v = Jv_com*robot->_dq;
			torso_ang_v = Jw_torso*robot->_dq;
			// cout << " Desired zmp pos " << pos_support_centroid.transpose() << endl;
			com_pos_err << (com_pos[0] - pos_support_centroid[0]), (com_pos[1] - pos_support_centroid[1]), (com_pos[2] - des_com_height_balanced);
			robot->orientationError(torso_ang_err, rot_torso_des, rot_torso);
			// cout << "torso_ang_err " << torso_ang_err.transpose() << endl;
			// cout << "torso_ang_v " << torso_ang_v.transpose() << endl;

			//separate linear and angular parts in task force below
			Eigen::Vector3d acc_com_err = - Eigen::Vector3d(kplcom, kplcom, kplcom*0.5).array()*com_pos_err.array() - Eigen::Vector3d(kvlcom, kvlcom, kvlcom).array()*com_v.array();
			Eigen::Vector3d acc_tor_ang_err = - kpacom*torso_ang_err.array() - kvacom*torso_ang_v.array();
			Eigen::VectorXd acc_task_err(6);
			acc_task_err << acc_com_err, acc_tor_ang_err;
			F_task = L_task*(acc_task_err + J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			// F_task = L_task*(acc_com_err + J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			F_task_passive = L_task*(J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			// if (L_task(2,2) < 0.001) {
			// 	f_global_sim_pause = true;
			// 	cout << "Global pause " << endl;
			// }
			// cout << "com_v " << endl << com_v.transpose() << endl;
			// cout << "com_pos_err " << endl << com_pos_err.transpose() << endl;
			// cout << "F_task " << endl << F_task.transpose() << endl;
			// cout << "F_task_passive " << endl << F_task_passive.transpose() << endl;

			// - compute required joint torques
			tau_act = actuated_space_projection_contact_both.transpose()*(J_task.transpose()*F_task);
			// tau_act += null_actuated_space_projection_contact*(actuated_space_projection_contact_both.transpose()*gj);
			tau_act += null_actuated_space_projection_contact*(
				actuated_space_projection_contact_both.transpose()*gj + 
				actuated_space_inertia_contact_both*(-kpj*0.1 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*0.3 * robot->_dq.tail(act_dof))
			);
			// tau_act = actuated_space_projection_contact_both.transpose()*( - robot->_M*kvj*(robot->_dq));

			// cout << "tau_act before internal forces correction: " << tau_act.transpose() << endl;

			// calculate feet stabilizing forces
			double des_feet_tension = kpfeetdistance*(feet_distance - 0.75);
			// cout << "des_feet_tension " << des_feet_tension << endl;

			// remove moments at the feet
			// cout << "feet_internal_force_projected_both_gravity: " << feet_internal_force_projected_both_gravity.transpose() << endl;
			Eigen::VectorXd tau_0(act_dof);
			Eigen::VectorXd tau_0_rhs(act_dof+6);

			tau_0_rhs.head(act_dof) = actuated_space_inertia_contact_inv_both*tau_act;
			tau_0_rhs(act_dof) = 0 + feet_internal_force_projected_both_gravity(0); // left foot moment x
			tau_0_rhs(act_dof+1) = 0 + feet_internal_force_projected_both_gravity(1); // left foot moment y
			tau_0_rhs(act_dof+2) = 0 + feet_internal_force_projected_both_gravity(2); // right foot moment x
			tau_0_rhs(act_dof+3) = 0 + feet_internal_force_projected_both_gravity(3); // right foot moment y
			tau_0_rhs(act_dof+4) = des_feet_tension + feet_internal_force_projected_both_gravity(4); // internal tension between feet = 30
			tau_0_rhs(act_dof+5) = 0 + feet_internal_force_projected_both_gravity(5); // internal moment between feet = 0
			tau_0 = feet_null_projector_both_inv*tau_0_rhs;
			// cout << "actuated_space_inertia_contact_inv_both*tau_act " << ((actuated_space_inertia_contact_inv_both)*tau_act).transpose() << endl;
			// cout << "actuated_space_inertia_contact_inv_both*tau_0 " << ((actuated_space_inertia_contact_inv_both)*tau_0).transpose() << endl;
			tau_act = tau_0;
			
			// - compute passive joint torques in case slippage/stickage is detected
			tau_act_passive = actuated_space_projection_contact_both.transpose()*(J_task.transpose()*F_task_passive);
			// tau_act += null_actuated_space_projection_contact*(actuated_space_projection_contact_both.transpose()*gj);
			tau_act_passive += null_actuated_space_projection_contact*(
				actuated_space_projection_contact_both.transpose()*gj + 
				actuated_space_inertia_contact_both*(-kpj*0.1 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*0.3 * robot->_dq.tail(act_dof))
			);
			// // tau_act = actuated_space_projection_contact_both.transpose()*( - robot->_M*kvj*(robot->_dq));

			// // remove moments at the feet
			tau_0_rhs.head(act_dof) = actuated_space_inertia_contact_inv_both*tau_act_passive;
			tau_0_rhs(act_dof) = 0 + feet_internal_force_projected_both_gravity(0); // left foot moment x
			tau_0_rhs(act_dof+1) = 0 + feet_internal_force_projected_both_gravity(1); // left foot moment y
			tau_0_rhs(act_dof+2) = 0 + feet_internal_force_projected_both_gravity(2); // right foot moment x
			tau_0_rhs(act_dof+3) = 0 + feet_internal_force_projected_both_gravity(3); // right foot moment y
			tau_0_rhs(act_dof+4) = 0 + feet_internal_force_projected_both_gravity(4); // internal tension between feet = 30
			tau_0_rhs(act_dof+5) = 0 + feet_internal_force_projected_both_gravity(5); // internal moment between feet = 0
			tau_0 = feet_null_projector_both_inv*tau_0_rhs;
			// cout << "(SNs)^T*tau_act_passive " << ((SN_contact_both).transpose()*tau_act_passive).transpose() << endl;
			// cout << "(SNs)^T*tau_0_passive " << ((SN_contact_both).transpose()*tau_0).transpose() << endl;
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
					double F_contact_plane_left = Eigen::Vector2d(F_contact[0], F_contact[1]).norm();
					double F_contact_plane_right = Eigen::Vector2d(F_contact[6], F_contact[7]).norm();
					// cout << "Expected F contact: " << F_contact.transpose() << endl;
					if (F_contact[2] < 0 || F_contact[8] < 0) {
						is_sticking = true;
					}
					else if (
						fabs(F_contact_plane_left/F_contact[2]) > 0.8 ||
						fabs(F_contact_plane_right/F_contact[8]) > 0.8) {
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
			// cout << "tau_act " << endl << tau_act.transpose() << endl;
		}

		if (curr_state == FSMState::Jumping) {
			// start accelerating COM upwards while maintaining tension between feet
			Eigen::Vector3d com_desired_velocity; // vx, vy, vz
			com_desired_velocity << 0.0, 0.0, 2.0*fmin(1.0, 0.75/feet_distance);
			Eigen::Vector3d torso_desired_ang_v; // wx, wy, wz
			torso_desired_ang_v << 0.0, -3.5, 0.0;

			// - compute task forces
			com_v = Jv_com*robot->_dq;
			torso_ang_v = Jw_torso*robot->_dq;
			// cout << " Desired zmp pos " << pos_support_centroid.transpose() << endl;
			com_pos_err << (com_pos[0] - pos_support_centroid[0]), (com_pos[1] - pos_support_centroid[1]), 0.0;
			robot->orientationError(torso_ang_err, rot_torso_des, rot_torso);
			//TODO: ^^ here, the desired rotation should be relative to the current orientation of the body
			// cout << "torso_ang_err " << torso_ang_err.transpose() << endl;
			// cout << "torso_ang_v " << torso_ang_v.transpose() << endl;
			Eigen::Vector3d acc_com_err = - Eigen::Vector3d(kplcom, kplcom, kplcom*0.5).array()*com_pos_err.array() - Eigen::Vector3d(kvlcom, kvlcom, kvlcom*2.0).array()*(com_v - com_desired_velocity).array();
			Eigen::Vector3d acc_tor_ang_err = - kpacom*2.0*torso_ang_err.array() - kvacom*2.0*(torso_ang_v - torso_desired_ang_v).array();
			Eigen::VectorXd acc_task_err(6);
			acc_task_err << acc_com_err, acc_tor_ang_err;
			F_task = L_task*(acc_task_err + J_task*robot->_M_inv*N_contact_both.transpose()*gj);

			// if knee pitch exceeds a certain angle, switch to fall controller
			if (robot->_q[9] < 45.0/180.0*M_PI || robot->_q[29] < 45.0/180.0*M_PI) {
				// TODO: check thigh pitch as well?
				curr_state = FSMState::FallingMidAir;
				cout << "Switching from Jumping to FallingMidAir" << endl;
				cout << "Desired velocity was " << com_desired_velocity.transpose() << endl;
				// cout << "Jumping: Estimated ZMP position: " << pos_support_centroid.transpose() << endl;
				// cout << "Jumping: Actual COM position: " << com_pos.transpose() << endl;
			}
			// alternatively, when contact is lost, switch to FSMState::FallingMidAir
			if (left_foot_point_list.size() == 0 && right_foot_point_list.size() == 0) {
				if (jump_counter > JUMP_COUNT_THRESH) {
					curr_state = FSMState::FallingMidAir;
					cout << "Switching from Jumping to FallingMidAir" << endl;
					// num_successful_jumps++;
					// cout << "Num jumps: " << num_successful_jumps << endl;
					balance_counter = 0;
				}
				else {
					jump_counter++;
				}
			} else {
				jump_counter = 0;
			}

			// F_task = L_task*(acc_com_err + J_task*robot->_M_inv*N_contact_both.transpose()*gj);
			// if (L_task(2,2) < 0.001) {
			// 	f_global_sim_pause = true;
			// 	cout << "Global pause " << endl;
			// }
			// cout << "com_v " << endl << com_v.transpose() << endl;
			// cout << "com_pos_err " << endl << com_pos_err.transpose() << endl;
			// cout << "F_task " << endl << F_task.transpose() << endl;

			// - compute required joint torques
			tau_act = actuated_space_projection_contact_both.transpose()*(J_task.transpose()*F_task);
			// tau_act += null_actuated_space_projection_contact*(actuated_space_projection_contact_both.transpose()*gj);
			tau_act += null_actuated_space_projection_contact*(
				actuated_space_projection_contact_both.transpose()*gj + 
				actuated_space_inertia_contact_both*(-kpj*0.1 * (robot->_q.tail(act_dof) - q_home.tail(act_dof)) - kvj*0.3 * robot->_dq.tail(act_dof))
			);
			// tau_act = actuated_space_projection_contact_both.transpose()*( - robot->_M*kvj*(robot->_dq));

			// cout << "tau_act before internal forces correction: " << tau_act.transpose() << endl;

			// calculate feet stabilizing forces
			// double des_feet_tension = kpfeetdistance*(feet_distance - 0.75);
			// cout << "des_feet_tension " << des_feet_tension << endl;

			// remove moments at the feet
			// cout << "feet_internal_force_projected_both_gravity: " << feet_internal_force_projected_both_gravity.transpose() << endl;
			Eigen::VectorXd tau_0(act_dof);
			Eigen::VectorXd tau_0_rhs(act_dof+6);

			tau_0_rhs.head(act_dof) = actuated_space_inertia_contact_inv_both*tau_act;
			tau_0_rhs(act_dof) = 0 + feet_internal_force_projected_both_gravity(0); // left foot moment x
			tau_0_rhs(act_dof+1) = 0 + feet_internal_force_projected_both_gravity(1); // left foot moment y
			tau_0_rhs(act_dof+2) = 0 + feet_internal_force_projected_both_gravity(2); // right foot moment x
			tau_0_rhs(act_dof+3) = 0 + feet_internal_force_projected_both_gravity(3); // right foot moment y
			// tau_0_rhs(act_dof+4) = des_feet_tension + feet_internal_force_projected_both_gravity(4); // internal tension between feet = 30
			tau_0_rhs(act_dof+4) = 0 + feet_internal_force_projected_both_gravity(4); // internal tension between feet = 30
			tau_0_rhs(act_dof+5) = 0 + feet_internal_force_projected_both_gravity(5); // internal moment between feet = 0
			tau_0 = feet_null_projector_both_inv*tau_0_rhs;
			// cout << "actuated_space_inertia_contact_inv_both*tau_act " << ((actuated_space_inertia_contact_inv_both)*tau_act).transpose() << endl;
			// cout << "actuated_space_inertia_contact_inv_both*tau_0 " << ((actuated_space_inertia_contact_inv_both)*tau_0).transpose() << endl;
			tau_act = tau_0;
		}

		// assemble full tau vector for simulation
		tau.tail(act_dof) = tau_act;
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
	timer.setLoopFrequency(1500); //1.5kHz timer
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
    if ((key == 'P' || key == 'p') && action == GLFW_PRESS)
    {
        // pause simulation
        f_global_sim_pause = !f_global_sim_pause;
    }
    if ((key == '1') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_front";
    }
    if ((key == '2') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_side";
    }
    if ((key == '3') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_top";
    }
    if ((key == '4') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_isometric";
    }
}

//------------------------------------------------------------------------------
MatrixXd pseudoinverse(const MatrixXd &mat, double tolerance) // choose appropriately
{
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    MatrixXd singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    // cout << "Inverting matrix of size: " << mat.rows() << " " << mat.cols() << endl;
    // cout << "Singular values " << singularValues.transpose() << endl;
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

//------------------------------------------------------------------------------
void comPosition(Vector3d& robot_com, Model::RBDLModel* robot, const Eigen::VectorXd& q, const std::vector<std::string> link_names) {
	robot_com.setZero();
	double mass;
	double robot_mass = 0.0;
	Vector3d center_of_mass_local;
	Vector3d center_of_mass_world;
	Matrix3d inertia;
	for (string link_name: link_names) {
		robot->getLinkMass(mass, center_of_mass_local, inertia, link_name);
		robot->position(center_of_mass_world, link_name, center_of_mass_local, q);
		robot_com += center_of_mass_world*mass;
		robot_mass += mass;
	}
	robot_com = robot_com/robot_mass;
}

//------------------------------------------------------------------------------
void comJacobian(MatrixXd& ret_J0, Model::RBDLModel* robot, const Eigen::VectorXd& q, const std::vector<std::string> link_names) {
	ret_J0.setZero(6, q.size());
	MatrixXd link_J0;
	link_J0.setZero(6, q.size());
	double mass;
	double robot_mass = 0.0;
	Vector3d center_of_mass_local;
	Matrix3d inertia;
	for (string link_name: link_names) {
		robot->getLinkMass(mass, center_of_mass_local, inertia, link_name);
		robot->J_0(link_J0, link_name, center_of_mass_local, q);
		ret_J0 += link_J0*mass;
		robot_mass += mass;
	}
	ret_J0 = ret_J0/robot_mass; //TODO: this is obviously incorrect for Jw. Need to fix by implementing the parallel axis theorem.
}
