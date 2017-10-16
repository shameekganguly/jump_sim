//ToroJumpController.h
#ifndef TORO_JUMP_CONTROLLER_H
#define TORO_JUMP_CONTROLLER_H

#include "whole_body_controller/WholeBodyController.h"
#include "model/ModelInterface.h"
#include <string>
#include <vector>

// possible ToroController FSM States
enum ToroControllerFSMState {
	Balancing,
	Jumping,
	FallingMidAir
};

// struct to encapsulate state for the ToroJumpController
struct ToroJumpControllerState: public ControllerState {
public:
	// fsm state
	ToroControllerFSMState _fsm_state = ToroControllerFSMState::FallingMidAir;

	// contact state
	std::vector<Eigen::Vector3d> _left_foot_force_list;
	std::vector<Eigen::Vector3d> _left_foot_point_list;
	std::vector<Eigen::Vector3d> _right_foot_force_list;
	std::vector<Eigen::Vector3d> _right_foot_point_list;

	// desired goals
	Eigen::Matrix3d _rot_torso_des = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d _rot_left_foot_des = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d _rot_right_foot_des = Eigen::Matrix3d::Identity();
	// NOTE: ^^ the desired rotations for the feet are updated by the fall controller in runtime

	// TODO: add q_home

	// gains
	double _kplcom = 15.0; // COM linear kp
	double _kvlcom = 5.0; // COM linear kv
	double _kpacom = 15.0; // COM angular kp
	double _kvacom = 5.0; // COM angular kv
	double _kpj = 70.0; // joint space kp
	double _kvj = 30.0; // joint space kv
	double _kpwleftf = 40; // left foot admittance kp for zero moment control
	double _kvwleftf = 10; // left foot kv damping
	double _kpwrightf = 40; // right foot admittance kp for zero moment control
	double _kvwrightf = 10; // right foot kv damping
	double _kpfeetdistance = 50; // gain to map internal tension between feet from distance between them

	// counters
	uint _balance_counter = 0;
	const uint BALANCE_COUNT_THRESH = 11;
	uint _stable_counter = 0;
	const uint STABLE_COUNT_THRESH = 31;
	uint _jump_counter = 0;
	const uint JUMP_COUNT_THRESH = 5;
	uint _controller_counter = 0;

	// overwritten copy function
	virtual void copy(ControllerState* other);
};

// extended runtime model interface for Toro controller computations
class ToroRuntimeModel: public Model::ModelInterface {
public:
	// ctor
	ToroRuntimeModel (const std::string& path_to_model_file);

	// dtor
	~ToroRuntimeModel();

public:
	// initialize dynamically sized variables
	uint _dof;
	uint _act_dof;

	// generalized model, in addition to the base class
	Eigen::VectorXd _gj;

	// underactuation dynamics
	Eigen::MatrixXd _selection_mat;
	Eigen::MatrixXd _actuated_space_projection;
	Eigen::MatrixXd _actuated_space_inertia;

	// double support contact model
	Eigen::MatrixXd _null_actuated_space_projection_contact;
	Eigen::MatrixXd _actuated_space_projection_contact_both;
	Eigen::MatrixXd _actuated_space_inertia_contact_both;
	Eigen::MatrixXd _actuated_space_inertia_contact_inv_both;
	Eigen::MatrixXd _feet_internal_forces_selection_mat;
	Eigen::MatrixXd _feet_null_projector_both;
	Eigen::VectorXd _feet_internal_force_projected_both_gravity;
	Eigen::MatrixXd _feet_null_projector_both_inv;
	Eigen::MatrixXd _J_c_both;
	Eigen::MatrixXd _L_contact_both;
	Eigen::MatrixXd _N_contact_both;
	Eigen::MatrixXd _SN_contact_both;
	Eigen::VectorXd _F_contact;

	// com kinematics
	Eigen::MatrixXd _J0_com;
	Eigen::MatrixXd _Jv_com;
	Eigen::MatrixXd _Jw_torso;
	Eigen::Matrix3d _rot_torso;
	const std::string _torso_name = "hip_base";

	// feet kinematics
	const std::string _right_foot_name = "RL_foot";
	const std::string _left_foot_name = "LL_foot";
	Eigen::MatrixXd _Jw_left_foot;
	Eigen::MatrixXd _Jw_right_foot;
	Eigen::Matrix3d _rot_left_foot;
	Eigen::Matrix3d _rot_right_foot;
	Eigen::MatrixXd _J0_left_foot;
	Eigen::MatrixXd _J0_right_foot;
	Eigen::MatrixXd _Jv_left_foot;
	Eigen::MatrixXd _Jv_right_foot;
	const Eigen::Vector3d _left_foot_pos_local = Eigen::Vector3d(0.02, 0.0, -0.05);
	const Eigen::Vector3d _right_foot_pos_local = Eigen::Vector3d(0.02, 0.0, -0.05);
	Eigen::Vector3d _left_foot_frame_pos_world; // position of left foot frame
	Eigen::Vector3d _right_foot_frame_pos_world; // position of right foot frame
	Eigen::MatrixXd _J_feet_tension;
	double _feet_distance;
	Eigen::Vector3d _left_to_right_foot_unit_vec;
	Eigen::Vector3d _pos_support_centroid;

	// double support task dynamics
	Eigen::MatrixXd _J_task;
	Eigen::MatrixXd _L_task;
	Eigen::VectorXd _F_task;
	Eigen::VectorXd _F_task_passive;
	Eigen::Vector3d _com_pos;
	Eigen::Vector3d _com_pos_err;
	Eigen::Vector3d _torso_ang_err;
	Eigen::Vector3d _com_v;
	Eigen::Vector3d _torso_ang_v;

	// fall task dynamics
	Eigen::MatrixXd _J_inter_foot_distance;
	Eigen::MatrixXd _J_com_zmp_x_err;
	Eigen::MatrixXd _J_fall_task;
	Eigen::MatrixXd _L_fall_task;
	Eigen::MatrixXd _M_inv_sel;
	Eigen::MatrixXd _N_fall_task;
	Eigen::Vector3d _left_foot_ang_err;
	Eigen::Vector3d _right_foot_ang_err;
	double _com_zmp_x_err;
	Eigen::VectorXd _fall_task_err_acc;
	Eigen::VectorXd _fall_lin_task_err;
	Eigen::VectorXd _fall_ang_task_err;
	Eigen::VectorXd _fall_task_vel;
	Eigen::VectorXd _F_fall_task;

	// link names. set at initialization
	std::vector<std::string> _link_names;

public:
	// -- internal functions --

	// compute com position
	void comPosition(Eigen::Vector3d& robot_com) const;

	// compute com linear Jacobian
	void comLinearJacobian(Eigen::MatrixXd& ret_Jv) const;
};

// class to encapsulate state machine and whole body controller for the jumping Toro
class ToroJumpController: public WholeBodyController {

public:
	// ctor
	ToroJumpController (const std::string& robot_fname);

	// dtor
	~ToroJumpController();

	// overloaded parent function
	void controllerStateIs (ControllerState* state);

private:
	// internal model interface
	ToroRuntimeModel* _robot;

	// joint torque related stuff
	Eigen::VectorXd _tau_act;
	Eigen::VectorXd _temp_tau;
	Eigen::VectorXd _tau_act_passive;
};

#endif // TORO_JUMP_CONTROLLER_H
