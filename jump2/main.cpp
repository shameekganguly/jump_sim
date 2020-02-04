/*  jump2 - main.cpp

This simulation exploits the trajectory optimization framework to create better 
jump trajectories in an interactive setting.

Author: Shameek Ganguly shameekg@stanford.edu
Date: 10/12/17
*/

#include "ToroJumpSystemModel.h"
#include "graphics/ChaiGraphics.h"
#include "model/ModelInterface.h"

#include <iostream>
#include <string>

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;

const string world_fname = "resources/jump2/world.urdf";
const string robot_fname = "../resources/toro/toro_headless.urdf";
const string robot_name = "Toro";
string camera_name = "camera_side";
// string camera_name = "camera_isometric";
// string camera_name = "camera_front";
// string camera_name = "camera_top";
// string ee_link_name = "link6";

// global variables
Eigen::VectorXd q_home;
bool f_global_sim_pause = false; // use with caution!
// bool f_global_sim_pause = true; // use with caution!

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

//------------------------------------------------------------------------------
int main(int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Graphics::ChaiGraphics(world_fname, Graphics::urdf, false);
	graphics->_world->setBackgroundColor(0.7, 0.7, 0.5);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

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
	robot->_dq.setZero(robot->dof());

	// cout << "Reached here" << endl;

	// initialize the robot simulated system
	ToroJumpSystemModel* toro = new ToroJumpSystemModel(world_fname, robot_fname, robot_name, q_home.tail(robot->dof() - 6));
	toro->robotInitialStateIs(robot->_q, robot->_dq); // set system state
	toro->runningIs(true); // start running

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

    // while window is open:
    auto controller = dynamic_cast<const ToroJumpController*> (toro->controller());
    auto controller_state = dynamic_cast<ToroJumpControllerState*> (controller->_state);
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		robot->_q = toro->controllerState()._q;
		robot->updateModel();
		//^^TODO: fix graphics glitching here probably due to non-atomic copy

		// force pause if the system is unstable
		//TODO: check instead for toro->didSystemFail() which is any way required for the optimization
		if (
			(controller->controllerModel()->_com_v).norm() > 2 && 
			controller_state->_fsm_state == ToroControllerFSMState::Balancing
		) {
			cout << "Global pause as COM speed exceeded " << endl;
			f_global_sim_pause = true;
		}

		// set pause state on system
		toro->pauseIs(f_global_sim_pause);

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
	toro->runningIs(false);

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Jump Sim with Dreaming", NULL, NULL);
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
