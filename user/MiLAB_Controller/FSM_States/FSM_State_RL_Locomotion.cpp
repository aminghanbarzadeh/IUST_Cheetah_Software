/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

 #include "FSM_State_RL_Locomotion.h"
 #include <Utilities/Utilities_print.h>
 #include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
 #include <Utilities/Timer.h>
 /**
  * Constructor for the FSM State that passes in state specific info to
  * the generic FSM State constructor.
  *
  * @param _controlFSMData holds all of the relevant control data
  */
 template <typename T>
FSM_State_RL_Locomotion<T>::FSM_State_RL_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RL_LOCOMOTION, "RL_LOCOMOTION"){

    this->turnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;
    
    // Initialize GRF and footstep locations to 0s
    this->footFeedForwardForces = Mat34<T>::Zero();
    this->footstepLocations = Mat34<T>::Zero();
    _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    _wbc_data = new LocomotionCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(1000.);
 }
 
 template <typename T>
 void FSM_State_RL_Locomotion<T>::onEnter() {
   // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
//   cMPCOld->initialize();
//   this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
//   printf("[FSM LOCOMOTION] On Enter\n");
 }
 

 /**
  * Calls the functions to be executed on each control loop iteration.
  */
 template <typename T>
 void FSM_State_RL_Locomotion<T>::run() {
    LocomotionControlStep();
 }

 extern rc_control_settings rc_control;


/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_RL_Locomotion<T>::checkTransition() {
  // Get the next state
//   iter++;

  // Switch FSM control mode 0,4,6
  if(locomotionSafe()) {
    switch ((int)this->_data->controlParameters->control_mode) {
      case K_RL_LOCOMOTION:
        break;

      case K_BALANCE_STAND:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::BALANCE_STAND;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_PASSIVE:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::PASSIVE;
        // Transition time is immediate
        this->transitionDuration = 0.0;
        break;


      case K_RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        break;


      default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_LOCOMOTION << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
  } else {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
  }


  // Return the next state name to the FSM
  return this->nextStateName;
}
/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_RL_Locomotion<T>::transition() {
  // Switch FSM control mode 0,6
  switch (this->nextStateName) {
    case FSM_StateName::BALANCE_STAND:
      LocomotionControlStep();

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }

      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;


    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition locomotion"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}
template <typename T>
void FSM_State_RL_Locomotion<T>::LocomotionControlStep() {
    Vec3<T> q_backup[4];
    Vec3<T> qd_backup[4];
    Vec3<T> tau_backup[4];
    // Mat3<T> Kp_backup[4];
    // Mat3<T> Kd_backup[4];
    // printf("hereeeeee\n");
  for(int leg(0); leg<4; ++leg){
    q_backup[leg] = this->_data->_legController->datas[leg].q;
    qd_backup[leg] = this->_data->_legController->datas[leg].qd;
    tau_backup[leg] = this->_data->_legController->datas[leg].tauEstimate;
    auto& seResult = this->_data->_stateEstimator->getResult();

    


    // pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    // vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    // Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    // Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

//   if(this->_data->userParameters->use_wbc > 0.9){// && !cMPCOld->standingMPC){
//     _wbc_data->pBody_des = cMPCOld->pBody_des;
//     _wbc_data->vBody_des = cMPCOld->vBody_des;
//     _wbc_data->aBody_des = cMPCOld->aBody_des;

//     _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
//     _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;
    
//     for(size_t i(0); i<4; ++i){
//       _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
//       _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
//       _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
//       _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i]; 
//     }
//     _wbc_data->contact_state = cMPCOld->contact_state;
//     _wbc_ctrl->run(_wbc_data, *this->_data);
//   }

}
 
 /**
  * Cleans up the state information on exiting the state.
  */
 template <typename T>
 void FSM_State_RL_Locomotion<T>::onExit() {
   // Nothing to clean up when exiting
 }
 
template<typename T>
bool FSM_State_RL_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 30;
  const T max_pitch = 30;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] >= -0.05) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > 0.25)) {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 10.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}
 // template class FSM_State_RL_Locomotion<double>;
 template class FSM_State_RL_Locomotion<float>;
 