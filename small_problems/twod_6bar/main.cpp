#include <iostream>
#include <string>
#include <thread>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "model/ModelInterface.h"
#include "graphics/ChaiGraphics.h"
#include "simulation/Sai2Simulation.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;

const string robot_fname = "resources/six_bar.urdf";
const string world_fname = "resources/world.urdf";
const string robot_name = "SixBar";
const string camera_name = "camera_top";

// contact link names
const string right_foot_name = "right_leg";
const string left_foot_name = "left_leg";
const string torso_name = "hip_base";

const Vector3d left_foot_pos_local(0.0, -0.5, 0.0);
const Vector3d right_foot_pos_local(0.0, 0.5, 0.0);

// global variables
VectorXd q_home;

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

// util function to calculate the pseudo inverse
MatrixXd pseudoinverse(const MatrixXd &mat, double tolerance = 1e-4);

int main (int argc, char** argv) {
	// initialize random
	srand (time(NULL));

	cout << "Loading URDF world model file: " << robot_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);
	graphics->_world->setBackgroundColor(0.7, 0.7, 0.5);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);
	int dof = robot->dof();
	int actuated_dof = robot->dof() - 3; // planar

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);
	sim->setCollisionRestitution(0.2);
 //    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(1.0);
    sim->setCoeffFrictionDynamic(1.0);

	// set test condition
	q_home.setZero(dof);
	q_home << -1.8, //0 floating_base_px
				0.0,	//1 floating_base_py
				((float) (rand() % 10 - 5)) /180.0*M_PI,	//2 floating_base_rz
				70.0/180.0*M_PI,	//3 left thigh adduction
				30.0/180.0*M_PI,	//4 left knee adduction
				-70.0/180.0*M_PI,	//5 right thigh adduction
				-30.0/180.0*M_PI;	//6 right knee adduction

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
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
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
	int act_dof = robot->dof() - 3; // 2D free base

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd temp_tau = Eigen::VectorXd::Zero(robot->dof());
	Eigen::VectorXd tau_act = Eigen::VectorXd::Zero(robot->dof() - 3);
	Eigen::VectorXd gj(robot->dof());
	Eigen::MatrixXd selection_mat(robot->dof() - 3, robot->dof());
	// cout << selection_mat << endl;
	Eigen::MatrixXd actuated_space_projection;
	actuated_space_projection.setZero(robot->dof() - 3, robot->dof());
	Eigen::MatrixXd actuated_space_inertia;
	actuated_space_inertia.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd actuated_space_projection_contact;
	actuated_space_projection_contact.setZero(robot->dof() - 3, robot->dof());
	Eigen::MatrixXd actuated_space_inertia_contact;
	actuated_space_inertia_contact.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd actuated_space_inertia_contact_inv;
	actuated_space_inertia_contact_inv.setZero(robot->dof() - 3, robot->dof() - 3);
	Eigen::MatrixXd null_actuated_space_projection_contact;
	null_actuated_space_projection_contact.setZero(robot->dof() - 3, robot->dof() - 3);
	std::vector<Eigen::Vector3d> left_foot_force_list;
	std::vector<Eigen::Vector3d> left_foot_point_list;
	std::vector<Eigen::Vector3d> right_foot_force_list;
	std::vector<Eigen::Vector3d> right_foot_point_list;
	Eigen::MatrixXd J_task;
	Eigen::MatrixXd L_task;
	Eigen::VectorXd F_task;
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
	Eigen::MatrixXd J_c(4, robot->dof());
	Eigen::Vector3d left_foot_frame_pos_world; // position of left foot frame
	Eigen::Vector3d right_foot_frame_pos_world; // position of right foot frame
	Eigen::MatrixXd J_feet_tension(1, robot->dof());
	Eigen::Vector3d left_to_right_foot_unit_vec;
	Eigen::Vector3d pos_support_centroid;
	Eigen::MatrixXd L_contact(4, 4);
	Eigen::MatrixXd N_contact(dof, dof);
	Eigen::VectorXd F_contact(4);
	Eigen::MatrixXd SN_contact(act_dof, dof);
	uint balance_counter;
	const uint BALANCE_COUNT_THRESH = 11;

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
	const double des_com_height_balanced = 0.95; // computed from above

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
		// 	// cout << robot->_M_inv.block(6,0,robot->dof()-6, robot->dof()) << endl;

		// 	// get the task Jacobians
			// - Jv com: TODO: need to add up the Jacobians for all the links
			// For now, we use the torso frame as a surrogate
			robot->J_0(J0_com, torso_name, Eigen::Vector3d(0.0, 0.0, 0.0));
			Jv_com = J0_com.block(0,0,2,dof);
			// // - Jw left foot rot
			// robot->Jw(Jw_left_foot, left_foot_name);
			// // - Jw right foot rot
			// robot->Jw(Jw_right_foot, right_foot_name);
		// 	// - J feet tension
			robot->J_0(J0_left_foot, left_foot_name, left_foot_pos_local);
			Jv_left_foot = J0_left_foot.block(0,0,2,dof);
			// cout << "Jv_left_foot " << endl << Jv_left_foot << endl;
			robot->J_0(J0_right_foot, right_foot_name, right_foot_pos_local);
			Jv_right_foot = J0_right_foot.block(0,0,2,dof);
			// cout << "Jv_right_foot " << endl << Jv_right_foot << endl;
			J_c << Jv_left_foot, Jv_right_foot;
			// cout << "J_c " << endl << J_c << endl;
			// cout << "robot->_M_inv " << endl << robot->_M_inv << endl;
			L_contact = (J_c*robot->_M_inv*J_c.transpose()).inverse();
			N_contact = 
				MatrixXd::Identity(dof, dof) - robot->_M_inv*J_c.transpose()*L_contact*J_c;
			// cout << "N_contact " << endl << N_contact << endl;
			// cout << "J_c*N_contact " << endl << J_c*N_contact << endl;
			SN_contact = N_contact.block(3, 0, act_dof, dof);
			// cout << "SN_contact " << endl << SN_contact << endl;
			actuated_space_inertia_contact_inv = SN_contact*robot->_M_inv*SN_contact.transpose();
			// cout << "actuated_space_inertia_contact_inv " << endl << actuated_space_inertia_contact_inv << endl;
			actuated_space_inertia_contact = pseudoinverse(actuated_space_inertia_contact_inv); // pseudo inverse of the inverse which is guaranteed to be singular
			// cout << "actuated_space_inertia_contact " << endl << actuated_space_inertia_contact << endl;
			actuated_space_projection_contact = robot->_M_inv*SN_contact.transpose()*actuated_space_inertia_contact;
			// cout << "actuated_space_projection_contact " << endl << actuated_space_projection_contact << endl;
			// cout << "bar(SNs)*SNs = Ns " << endl << actuated_space_projection_contact*SN_contact << endl;
		// 	robot->position(left_foot_frame_pos_world, left_foot_name, left_foot_frame_pos_local);
		// 	robot->position(right_foot_frame_pos_world, right_foot_name, right_foot_frame_pos_local);
		// 	left_to_right_foot_unit_vec = (right_foot_frame_pos_world - left_foot_frame_pos_world);
		// 	left_to_right_foot_unit_vec.normalize();
		// 	J_feet_tension = left_to_right_foot_unit_vec*(Jv_right_foot - Jv_left_foot);

			// update support polygon centroid
			size_t n_points = left_foot_point_list.size() + right_foot_point_list.size();
			if (n_points > 1) {
				pos_support_centroid.setZero();
				for(auto point: left_foot_point_list) {
					pos_support_centroid += point;
				}
				for(auto point: right_foot_point_list) {
					pos_support_centroid += point;
				}
				pos_support_centroid /= n_points;
			}
			// size_t n_points = left_foot_point_list.size() + right_foot_point_list.size();
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
			balance_counter = 0;
			// simply compensate for gravity and brace for landing
			tau_act = actuated_space_inertia*(- kvj*(robot->_dq.tail(act_dof)));
			tau_act += actuated_space_projection*gj;
			// TODO: check for contact and switch to FSMState::Balancing
			if (left_foot_point_list.size() || right_foot_point_list.size()) {
				curr_state = FSMState::Balancing;
				cout << "Switching from FallingMidAir to Balancing" << endl;
				// set default support position before update
				pos_support_centroid << 0.0, robot->_q[1], 0.0;
			}
		}

		if (curr_state == FSMState::Balancing) {
			balance_counter++;
			if (balance_counter > BALANCE_COUNT_THRESH) {
				// TODO: start zero tension, COM stabilization and posture control to q_home
				
				// - set the task Jacobian, project through the contact null-space
				J_task.setZero(3, dof);
				J_task.block(0,0,2,dof) = Jv_com; // linear part
				J_task(2,2) = 1; // angular part
				J_task = J_task * N_contact;
				// cout << "J_task " << endl << J_task << endl;
				
				// - compute task forces
				L_task = (J_task*robot->_M_inv*J_task.transpose()).inverse();
				// cout << "L_task " << endl << L_task << endl;
				robot->position(com_pos, torso_name, Vector3d(0.0,0.0,0.0));
				com_v << Jv_com*robot->_dq, (robot->_dq)[2];
				// cout << "com_v " << endl << com_v << endl;
				com_pos_err << (com_pos[0] + des_com_height_balanced), (com_pos[1] - pos_support_centroid[1]), robot->_q[2];
				// cout << "com_pos_err " << endl << com_pos_err << endl;
				//TODO: separate linear and angular parts in task force below
				F_task = L_task*(- kplcom*com_pos_err - kvlcom*com_v + J_task*robot->_M_inv*N_contact.transpose()*gj);
				// cout << "F_task " << endl << F_task << endl;

				// - compute posture torques
				null_actuated_space_projection_contact = 
					MatrixXd::Identity(act_dof,act_dof) - actuated_space_projection_contact.transpose()*J_task.transpose()*L_task*J_task*robot->_M_inv*SN_contact.transpose();

				// - compute required joint torques
				tau_act = actuated_space_projection_contact.transpose()*(J_task.transpose()*F_task);
				tau_act += null_actuated_space_projection_contact*(actuated_space_projection_contact.transpose()*gj);
				// tau_act = actuated_space_projection_contact.transpose()*( - robot->_M*kvj*(robot->_dq));
				
				// TODO: if COM velocity has been zero for a while, switch to FSMState::Jumping

				// TODO: check friction cone constraint
				temp_tau.setZero(dof);
				temp_tau.tail(act_dof) = tau_act;
				F_contact = -L_contact*J_c*robot->_M_inv*(temp_tau - gj);
				if (F_contact[0] > -0.1 || F_contact[2] > -0.1) {
					cout << "Exceeded normal force " << F_contact.transpose() << endl;
					cout << "Without actuator torques " << (-L_contact*J_c*robot->_M_inv*(-gj)).transpose()<< endl;
					if (left_foot_force_list.size()) {
						cout << "Actual left foot contact forces " << left_foot_force_list[0].transpose() << endl;
					}
					if (right_foot_force_list.size()) {
						cout << "Actual right foot contact forces " << right_foot_force_list[0].transpose() << endl;
					}
					tau_act.setZero(act_dof); //play safe
				}
				else if (fabs(F_contact[1]/F_contact[0]) > 0.8 || fabs(F_contact[3]/F_contact[2]) > 0.8) {
					cout << "Slip detected " << F_contact.transpose() << endl;
					tau_act.setZero(act_dof); //play safe
				}
				// cout << "tau_act " << endl << tau_act << endl;
			}
		}

		// if (curr_state == FSMState::Jumping) {
		// 	// TODO: start accelerating COM upwards while maintaining tension between feet
		// 	// TODO: when contact is lost, switch to FSMState::FallingMidAir
		// }

		// // assemble full tau vector for simulation
		// cout << tau_act.transpose() << endl;
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - 2D 6bar Jump Sim", NULL, NULL);
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
