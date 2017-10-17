// implementation for ToroJumpController classes
#include "ToroJumpController.h"
#include <iostream>

using namespace Eigen;
using namespace std;

// util functions fwd declaration
MatrixXd pseudoinverse(const MatrixXd &mat, double tolerance = 1e-4);

// ------------------------- ToroJumpControllerState ----------------------- //
void ToroJumpControllerState::copy(ControllerState* other) {
	// call parent copy
	ControllerState::copy(other);

	auto other_toroctrlstate = dynamic_cast<ToroJumpControllerState*> (other);
	_fsm_state = other_toroctrlstate->_fsm_state;
	_left_foot_force_list = other_toroctrlstate->_left_foot_force_list;
	_left_foot_point_list = other_toroctrlstate->_left_foot_point_list;
	_right_foot_force_list = other_toroctrlstate->_right_foot_force_list;
	_right_foot_point_list = other_toroctrlstate->_right_foot_point_list;
	_rot_torso_des = other_toroctrlstate->_rot_torso_des;
	_rot_left_foot_des = other_toroctrlstate->_rot_left_foot_des;
	_rot_right_foot_des = other_toroctrlstate->_rot_right_foot_des;
	_kplcom = other_toroctrlstate->_kplcom;
	_kvlcom = other_toroctrlstate->_kvlcom;
	_kpacom = other_toroctrlstate->_kpacom;
	_kvacom = other_toroctrlstate->_kvacom;
	_kpj = other_toroctrlstate->_kpj;
	_kvj = other_toroctrlstate->_kvj;
	_kpwleftf = other_toroctrlstate->_kpwleftf;
	_kvwleftf = other_toroctrlstate->_kvwleftf;
	_kvwrightf = other_toroctrlstate->_kvwrightf;
	_kpwrightf = other_toroctrlstate->_kpwrightf;
	_kpfeetdistance = other_toroctrlstate->_kpfeetdistance;
	_balance_counter = other_toroctrlstate->_balance_counter;
	_stable_counter = other_toroctrlstate->_stable_counter;
	_jump_counter = other_toroctrlstate->_jump_counter;
	_controller_counter = other_toroctrlstate->_controller_counter;
}

// ---------------------------- ToroRuntimeModel --------------------------- //

// ctor
ToroRuntimeModel::ToroRuntimeModel (const std::string& path_to_model_file):
// initialize base class
Model::ModelInterface(path_to_model_file, Model::rbdl, Model::urdf)
{
	
	// initialize dynamically sized variables
	_dof = dof();
	_act_dof = dof() - 6; // for 6 virtual joints

	// generalized model
	_gj.setZero(_dof);

	// underactuation dynamics
	_selection_mat.setZero(_act_dof, _dof);
	_actuated_space_projection.setZero(_act_dof, _dof);
	_actuated_space_inertia.setZero(_act_dof, _act_dof);

	// double support contact model
	_null_actuated_space_projection_contact.setZero(_act_dof, _act_dof);
	_actuated_space_projection_contact_both.setZero(_act_dof, _dof);
	_actuated_space_inertia_contact_both.setZero(_act_dof, _act_dof);
	_actuated_space_inertia_contact_inv_both.setZero(_act_dof, _act_dof);
	_feet_internal_forces_selection_mat.setZero(6,12);
	_feet_internal_forces_selection_mat(0,3) = 1; // left foot x moment
	_feet_internal_forces_selection_mat(1,5) = 1; // left foot z moment
	_feet_internal_forces_selection_mat(2,9) = 1; // right foot x moment
	_feet_internal_forces_selection_mat(3,11) = 1; // right foot z moment
	_feet_internal_forces_selection_mat(4,1) = -1; // internal tension. default is in y direction
	_feet_internal_forces_selection_mat(4,7) = 1; // internal tension
	_feet_internal_forces_selection_mat(5,4) = -1; // internal plane torsion
	_feet_internal_forces_selection_mat(5,10) = 1; // internal plane torsion (about y)
	_feet_null_projector_both.setZero(_act_dof+6, _act_dof);
	_feet_internal_force_projected_both_gravity.setZero(6);
	_feet_null_projector_both_inv.setZero(_act_dof, _act_dof+6);
	_J_c_both.setZero(12, _dof);
	_L_contact_both.setZero(12, 12);
	_N_contact_both.setZero(_dof, _dof);
	_SN_contact_both.setZero(_act_dof, _dof);
	_F_contact.setZero(12);

	// com kinematics
	_J0_com.setZero(6, _dof);
	_Jv_com.setZero(3, _dof);
	_Jw_torso.setZero(3, _dof);
	_rot_torso.setZero();

	// feet kinematics
	_Jw_left_foot.setZero(3, _dof);
	_Jw_right_foot.setZero(3, _dof);
	_rot_left_foot.setZero();
	_rot_right_foot.setZero();
	_J0_left_foot.setZero(6, _dof);
	_J0_right_foot.setZero(6, _dof);
	_Jv_left_foot.setZero(3, _dof);
	_Jv_right_foot.setZero(3, _dof);
	_left_foot_frame_pos_world.setZero(); // position of left foot frame
	_right_foot_frame_pos_world.setZero(); // position of right foot frame
	_J_feet_tension.setZero(1, _dof);
	_feet_distance = 0.0;
	_left_to_right_foot_unit_vec << 0, -1, 0; // default unit vector
	_pos_support_centroid.setZero();


	// fall task dynamics
	_J_inter_foot_distance.setZero(1, _dof);
	_J_com_zmp_x_err.setZero(1, _dof);
	_J_fall_task.setZero(8, _dof);
	_L_fall_task.setZero(8, 8);
	_M_inv_sel.setZero(_dof, _act_dof);
	_N_fall_task.setZero(_act_dof, _act_dof);
	_left_foot_ang_err.setZero();
	_right_foot_ang_err.setZero();
	_fall_task_err_acc.setZero(8);
	_fall_lin_task_err.setZero(2);
	_fall_ang_task_err.setZero(6);
	_fall_task_vel.setZero(8);
	_F_fall_task.setZero(8);

	// link names
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
	_link_names.assign(arr, arr + sizeof(arr) / sizeof(arr[0]));
}

// dtor
ToroRuntimeModel::~ToroRuntimeModel() {
	delete _model_internal;
}

// update balance kinematics
void ToroRuntimeModel::updateBalanceKinematicModel() {
	// COM position
	comPosition(_com_pos);

	// foot positions, rotations
	position(_left_foot_frame_pos_world, _left_foot_name, _left_foot_pos_local);
	rotation(_rot_left_foot, _left_foot_name);
	// cout << "rot_left_foot " << endl << _rot_left_foot << endl;
	position(_right_foot_frame_pos_world, _right_foot_name, _right_foot_pos_local);
	rotation(_rot_right_foot, _right_foot_name);

	// inter foot distance, direction vector
	_left_to_right_foot_unit_vec = (_right_foot_frame_pos_world - _left_foot_frame_pos_world);
	_feet_distance = _left_to_right_foot_unit_vec.norm();
	_left_to_right_foot_unit_vec.normalize();
	// cout << "left_to_right_foot_unit_vec " << _left_to_right_foot_unit_vec.transpose() << endl;
	// cout << "feet_distance " << _feet_distance << endl;

	// update support polygon centroid
	_pos_support_centroid.setZero();
	// TODO: use computed convex hull when actually in contact
	_pos_support_centroid[1] = (_left_foot_frame_pos_world[1] + _right_foot_frame_pos_world[1])*0.5;
	_pos_support_centroid[0] = (_left_foot_frame_pos_world[0] + _right_foot_frame_pos_world[0])*0.5;
	// cout << "Estimated ZMP position: " << _pos_support_centroid.transpose() << endl;
	// cout << "Actual COM position: " << _com_pos.transpose() << endl;
}

// overriden parent class function: updateModel
void ToroRuntimeModel::updateModel() {
	// parent update model
	Model::ModelInterface::updateModel();

	// update balance kinematics
	updateBalanceKinematicModel();

	// update other articulated body model parts
	gravityVector(_gj, Vector3d(0.0, 0.0, -3.00)); //TODO: expose the gravity vector to application

	// free space inertial projections
	_actuated_space_inertia = (_M_inv.block(6,6,_act_dof, _act_dof)).inverse();
	_actuated_space_projection = _actuated_space_inertia * (_M_inv.block(6,0,_act_dof,_dof));
	// cout << _M_inv.block(6,0,act_dof, dof) << endl;

	// get the task Jacobians
	// - com Jv
	comLinearJacobian(_Jv_com);

	// - Jv, Jw at feet
	J_0(_J0_left_foot, _left_foot_name, _left_foot_pos_local);
	_Jv_left_foot = _J0_left_foot.block(0,0,3,_dof);
	_Jw_left_foot = _J0_left_foot.block(3,0,3,_dof);
	// cout << "Jv_left_foot " << endl << _Jv_left_foot << endl;
	J_0(_J0_right_foot, _right_foot_name, _right_foot_pos_local);
	_Jv_right_foot = _J0_right_foot.block(0,0,3,_dof);
	_Jw_right_foot = _J0_right_foot.block(3,0,3,_dof);

	// - J feet tension
	_J_feet_tension = _left_to_right_foot_unit_vec.transpose()*(_Jv_right_foot - _Jv_left_foot);
	// cout << "feet_internal_forces_selection_mat " << endl << feet_internal_forces_selection_mat << endl;

	// update torso angular parameters
	Jw(_Jw_torso, _torso_name);
	// cout << "Jw_torso " << endl << _Jw_torso << endl;
	rotation(_rot_torso, _torso_name);

	// update feet_internal_forces_selection_mat with currect inter-foot direction vector
	if (_feet_distance > 0.1) {
		_feet_internal_forces_selection_mat.block(4,0,1,3) = _left_to_right_foot_unit_vec.transpose();
		_feet_internal_forces_selection_mat.block(4,6,1,3) = -_left_to_right_foot_unit_vec.transpose();
		_feet_internal_forces_selection_mat.block(5,3,1,3) = _left_to_right_foot_unit_vec.transpose();
		_feet_internal_forces_selection_mat.block(5,9,1,3) = -_left_to_right_foot_unit_vec.transpose();
	}

	// contact projections for both feet
	_J_c_both << _J0_left_foot, _J0_right_foot;
	_L_contact_both = (_J_c_both*_M_inv*_J_c_both.transpose()).inverse();
	_N_contact_both = 
		MatrixXd::Identity(_dof, _dof) - _M_inv*_J_c_both.transpose()*_L_contact_both*_J_c_both;
	_SN_contact_both = _N_contact_both.block(6, 0, _act_dof, _dof);
	_actuated_space_inertia_contact_inv_both = _SN_contact_both*_M_inv*_SN_contact_both.transpose();
	_actuated_space_inertia_contact_both = pseudoinverse(_actuated_space_inertia_contact_inv_both); // pseudo inverse of the inverse which is guaranteed to be singular
	_actuated_space_projection_contact_both = _M_inv*_SN_contact_both.transpose()*_actuated_space_inertia_contact_both;
	_feet_null_projector_both.block(0,0,_act_dof,_act_dof) = _actuated_space_inertia_contact_inv_both;
	_feet_null_projector_both.block(_act_dof,0,6,_act_dof) = 
		_feet_internal_forces_selection_mat*_L_contact_both*_J_c_both*(_M_inv.block(0,6,_dof,_act_dof));
	_feet_null_projector_both_inv = pseudoinverse(_feet_null_projector_both);
	_feet_internal_force_projected_both_gravity = _feet_internal_forces_selection_mat*(_L_contact_both*_J_c_both*_M_inv)*_gj;
	// cout << "J_c_both " << endl << _J_c_both << endl;
	// cout << "L_contact_both " << endl << _L_contact_both << endl;
	// cout << "robot->_M_inv " << endl << _M_inv << endl;
	// cout << "N_contact_both " << endl << _N_contact_both << endl;
	// cout << "J_c_both*N_contact_both " << endl << _J_c_both*_N_contact_both << endl;
	// cout << "SN_contact_both " << endl << _SN_contact_both << endl;
	// cout << "actuated_space_inertia_contact_inv_both " << endl << _actuated_space_inertia_contact_inv_both << endl;
	// cout << "actuated_space_inertia_contact_both " << endl << _actuated_space_inertia_contact_both << endl;
	// cout << "actuated_space_projection_contact_both " << endl << _actuated_space_projection_contact_both << endl;
	// cout << "bar(SNs_both)*SNs_both = Ns_both " << endl << _actuated_space_projection_contact_both*_SN_contact_both << endl;
	// cout << "feet_null_projector_both " << endl << _feet_null_projector_both << endl;
	// cout << "feet_null_projector_both_inv " << endl << _feet_null_projector_both_inv << endl;

	// update fall task dynamics
	_J_inter_foot_distance = _J_feet_tension;
	_J_com_zmp_x_err = Vector3d(1.0, 0, 0).transpose()*((_Jv_left_foot + _Jv_right_foot)*0.5 - _Jv_com);
	// TODO: ^^ fix x vector above to be in plane perpendicular to the feet vector
	_J_fall_task << _J_inter_foot_distance, _J_com_zmp_x_err, _Jw_left_foot, _Jw_right_foot;
	// cout << "J_fall_task " << endl << _J_fall_task << endl;
	_M_inv_sel = _M_inv.block(0,6,_dof,_act_dof);
	_L_fall_task = (_J_fall_task*_M_inv_sel*_actuated_space_inertia*_M_inv_sel.transpose()*_J_fall_task.transpose()).inverse();
	// cout << "L_fall_task " << endl << _L_fall_task << endl;
	_N_fall_task =
		MatrixXd::Identity(_act_dof,_act_dof) - _actuated_space_projection*_J_fall_task.transpose()*_L_fall_task*_J_fall_task*_M_inv_sel;

	// update double stance task dynamics
	// - set the task Jacobian, project through the contact null-space
	_J_task.setZero(6, _dof);
	_J_task << _Jv_com, _Jw_torso;
	// cout << "Jv_com " << endl << _Jv_com << endl;
	_J_task = _J_task * _N_contact_both;
	_L_task = (_J_task*_M_inv*_J_task.transpose()).inverse();
	// cout << "J_task " << endl << _J_task << endl;
	// cout << "L_task_inv " << endl << _J_task*_M_inv*_J_task.transpose() << endl;
	// cout << "L_task " << endl << _L_task << endl;
	_null_actuated_space_projection_contact =
		MatrixXd::Identity(_act_dof,_act_dof) - _actuated_space_projection_contact_both.transpose()*_J_task.transpose()*_L_task*_J_task*_M_inv*_SN_contact_both.transpose();
}

// compute com position
void ToroRuntimeModel::comPosition(Eigen::Vector3d& ret_robot_com) const{
	ret_robot_com.setZero();
	double mass;
	double robot_mass = 0.0;
	Vector3d center_of_mass_local;
	Vector3d center_of_mass_world;
	Matrix3d inertia;
	// down cast _model_internal to rbdl model
	auto rbdl_model = dynamic_cast<Model::RBDLModel*>(_model_internal);
	for (string link_name: _link_names) {
		rbdl_model->getLinkMass(mass, center_of_mass_local, inertia, link_name);
		rbdl_model->position(center_of_mass_world, link_name, center_of_mass_local, _q);
		ret_robot_com += center_of_mass_world*mass;
		robot_mass += mass;
	}
	ret_robot_com = ret_robot_com/robot_mass;
}

// compute com linear Jacobian
void ToroRuntimeModel::comLinearJacobian(Eigen::MatrixXd& ret_Jv) const {
	ret_Jv.setZero(3, _dof);
	MatrixXd link_J0;
	link_J0.setZero(6, _dof);
	double mass;
	double robot_mass = 0.0;
	Vector3d center_of_mass_local;
	Matrix3d inertia;
	// down cast _model_internal to rbdl model
	auto rbdl_model = dynamic_cast<Model::RBDLModel*>(_model_internal);
	for (string link_name: _link_names) {
		rbdl_model->getLinkMass(mass, center_of_mass_local, inertia, link_name);
		rbdl_model->J_0(link_J0, link_name, center_of_mass_local, _q);
		ret_Jv += link_J0.block(0,0,3,_dof)*mass;
		robot_mass += mass;
	}
	ret_Jv = ret_Jv/robot_mass;
}

// --------------------------- ToroJumpController -------------------------- //

// ctor
ToroJumpController::ToroJumpController (const std::string& robot_fname) { 
	//TODO: ^^do we need the robot name here for introspection?
	// initialize robot model
	_robot = new ToroRuntimeModel (robot_fname);

	// initialize dynamic vectors
	_tau_act.setZero(_robot->_act_dof);
	_temp_tau.setZero(_robot->_dof);
	_tau_act_passive.setZero(_robot->_act_dof);

	// set initial controller state
	_state = new ToroJumpControllerState();
}

// dtor
ToroJumpController::~ToroJumpController() {
	delete _robot;
}

// update controller state and compute control torques
void ToroJumpController::controllerStateIs (ControllerState* state) {
	// dynamic cast state to ToroJumpControllerState
	auto ret_state = dynamic_cast<ToroJumpControllerState*>(state);

	// TODO: set q, dq, contact point list to model. this also updates critical parts of the model immediately
	_robot->_q = ret_state->_q;
	_robot->_dq = ret_state->_dq;

	// periodially update full model
	if (true) { // always update the balance kinematics model
		_robot->updateBalanceKinematicModel();

		// some times there is a bug where the point list does not get updated correctly. So we have
		// to add a hack to ensure that the list is empty when we are sure that the robot is not on the 
		// ground
		if (_robot->_right_foot_frame_pos_world[2] > 0.1 && _robot->_left_foot_frame_pos_world[2] > 0.1) {
			ret_state->_left_foot_point_list.clear();
			ret_state->_left_foot_force_list.clear();
			ret_state->_right_foot_point_list.clear();
			ret_state->_right_foot_force_list.clear();
		}
	}
	if (ret_state->_controller_counter % 10 == 1) {
		_robot->updateModel();
	}

	// TODO: compute control torques based on new state
	_torques.setZero(_robot->_dof);

	// update control counter
	ret_state->_controller_counter++;

	// upcast and update state. This should be the last operation/
	_state->copy(ret_state);
}

// ---------------------------------- utils -------------------------------- //
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
