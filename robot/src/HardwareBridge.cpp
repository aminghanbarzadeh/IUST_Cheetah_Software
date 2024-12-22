/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
#ifdef linux 

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"

#include "HardwareBridge.h"
//#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"
#include "rt/rt_spi.h"
#include "rt/rt_vectornav.h"
#include "rt/rt_ethercat.h"
#include "Utilities/Utilities_print.h"
#include "rt/rt_joystick_interface.h"
#define USE_MICROSTRAIN
//#define IMU_DEBUG_SHOW
//#define SPI_DEBUG_SHOW
//#define JPOS_CTRL
//#define SPI_CTRL
//#define LOWLEVEL_CTRL
//#define CMPC_CTRL
#define IUST_CTRL

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char* reason, bool printErrno) {
  printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

  if (printErrno) {
    printf("Error: %s\n", strerror(errno));
  }

  exit(-1);
}

/*!
 * All hardware initialization steps that are common between Cheetah 3 and Mini Cheetah
 */
void HardwareBridge::initCommon() {
  printf("[HardwareBridge] Init stack\n");
  prefaultStack();
  printf("[HardwareBridge] Init scheduler\n");
  setupScheduler();
  if (!_interfaceLCM.good()) {
    initError("_interfaceLCM failed to initialize\n", false);
  }

  printf("[HardwareBridge] Subscribe LCM\n");
  _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);
  _interfaceLCM.subscribe("interface_request", &HardwareBridge::handleControlParameter, this);

  printf("[HardwareBridge] Start interface LCM handler\n");
  _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
}

/*!
 * Run interface LCM
 */
void HardwareBridge::handleInterfaceLCM() {
  while (!_interfaceLcmQuit)
    _interfaceLCM.handle();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
void HardwareBridge::prefaultStack() {
  printf("[Init] Prefault stack...\n");
  volatile char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}

/*!
 * Configures the scheduler for real time priority
 */
void HardwareBridge::setupScheduler() {
  printf("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    initError("sched_setscheduler failed.\n", true);
  }
}

/*!
 * LCM Handler for gamepad message
 */
void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  _gamepadCommand.set(msg);
}

/*!
 * Receive RC with SBUS
 */
void HardwareBridge::run_sbus() {
    // printf("--------------------------------------*****\n");
    if (_port > 0) {
        // printf("AFTERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR\n");
        //int x = receive_sbus(_port);
        // printf("77777777777777777777777777777777777777777\n");
        //std::cout<<x<<std::endl;
        // printf("88888888888888888888888888888888888888888\n");
        // int x = receive_sbus(_port);
        int x = receive_data(_port);
        if (x) {
            // sbus_packet_complete();
            update_joystick();
        } else  printf("[HARDWARE BRIDGE] Receive sbus failed.\n");
    }
    else  printf("[HARDWARE BRIDGE] Run sbus failed, port<0\n");
}

/*!
 * LCM Handler for control parameters
 */
void HardwareBridge::handleControlParameter(
    const lcm::ReceiveBuffer* rbuf, const std::string& chan,
    const control_parameter_request_lcmt* msg) {
  (void)rbuf;
  (void)chan;
  if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) {
    // nothing to do!
    printf(
        "[HardwareBridge] Warning: the interface has run a ControlParameter "
        "iteration, but there is no new request!\n");
    // return;
  }

  // sanity check
  s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
  if (nRequests != 1) {
    printf("[ERROR] Hardware bridge: we've missed %ld requests\n",
           nRequests - 1);
  }

  switch (msg->requestKind) {
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
      if(!_userControlParameters) {
        printf("[Warning] Got user param %s, but not using user parameters!\n",
               (char*)msg->name);
      } else {
        std::string name((char*)msg->name);
        ControlParameter& param = _userControlParameters->collection.lookup(name);

        // type check
        if ((s8)param._kind != msg->parameterKind) {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(
                  (ControlParameterValueKind)msg->parameterKind));
        }

        // do the actual set
        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);

        // respond:
        _parameter_response_lcmt.requestNumber =
            msg->requestNumber;  // acknowledge that the set has happened
        _parameter_response_lcmt.parameterKind =
            msg->parameterKind;  // just for debugging print statements
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
        //for debugging print statements
        strcpy((char*)_parameter_response_lcmt.name,
               name.c_str());  // just for debugging print statements
        _parameter_response_lcmt.requestKind = msg->requestKind;

        printf("[User Control Parameter] set %s to %s\n", name.c_str(),
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind)
                   .c_str());
      }
    } break;

    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
      std::string name((char*)msg->name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(
                (ControlParameterValueKind)msg->parameterKind));
      }

      // do the actual set
      ControlParameterValue v;
      memcpy(&v, msg->value, sizeof(v));
      param.set(v, (ControlParameterValueKind)msg->parameterKind);

      // respond:
      _parameter_response_lcmt.requestNumber =
          msg->requestNumber;  // acknowledge that the set has happened
      _parameter_response_lcmt.parameterKind =
          msg->parameterKind;  // just for debugging print statements
      memcpy(_parameter_response_lcmt.value, msg->value, 64);
      //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
      //for debugging print statements
      strcpy((char*)_parameter_response_lcmt.name,
             name.c_str());  // just for debugging print statements
      _parameter_response_lcmt.requestKind = msg->requestKind;

      printf("[Robot Control Parameter] set %s to %s\n", name.c_str(),
             controlParameterValueToString(
                 v, (ControlParameterValueKind)msg->parameterKind)
                 .c_str());

    } break;

    default: {
      throw std::runtime_error("parameter type unsupported");
    }
    break;
  }
  _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}

/*
 * ======================================================================================
 * Following class functions are specifically defined for Mini Cheetah
 */
MiniCheetahHardwareBridge::MiniCheetahHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)
    : HardwareBridge(robot_ctrl), _spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255)) {
  _load_parameters_from_file = load_parameters_from_file;
}

/*!
 * Main method for Mini Cheetah hardware
 */
void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  if(_load_parameters_from_file) {
    printf("[Hardware Bridge] Loading parameters from file...\n");

    try {
      _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");
    } catch(std::exception& e) {
      printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
      exit(1);
    }

    if(!_robotParams.isFullyInitialized()) {
      printf("Failed to initialize all robot parameters\n");
      exit(1);
    }

    printf("Loaded robot parameters\n");

    if(_userControlParameters) {
      try {
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml");
      } catch(std::exception& e) {
        printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
        exit(1);
      }

      if(!_userControlParameters->isFullyInitialized()) {
        printf("Failed to initialize all user parameters\n");
        exit(1);
      }

      printf("Loaded user parameters\n");
    } else {
      printf("Did not load user parameters because there aren't any\n");
    }
  } else {
    printf("[Hardware Bridge] Loading parameters over LCM...\n");
    while (!_robotParams.isFullyInitialized()) {
      printf("[Hardware Bridge] Waiting for robot parameters...\n");
      usleep(1000000);
    }

    if(_userControlParameters) {
      while (!_userControlParameters->isFullyInitialized()) {
        printf("[Hardware Bridge] Waiting for user parameters...\n");
        usleep(1000000);
      }
    }
  }



  printf("[Hardware Bridge] Got all parameters, starting up!\n");

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_sharedMemory().simToRobot.gamepadCommand;
  _robotRunner->spiData = &_spiData;
  _robotRunner->spiCommand = &_spiCommand;
  _robotRunner->robotType = RobotType::MINI_CHEETAH;
  _robotRunner->vectorNavData = &_vectorNavData;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

  _firstRun = false;

  // init control thread

  statusTask.start();

  // spi Task start
  PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(
      &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);
  spiTask.start();

  // microstrain
  if(_microstrainInit)
    _microstrainThread = std::thread(&MiniCheetahHardwareBridge::runMicrostrain, this);

  // robot controller start
  _robotRunner->start();

  // visualization start
  PeriodicMemberFunction<MiniCheetahHardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

  // rc controller
  _port = init_sbus(false);
  PeriodicMemberFunction<HardwareBridge> sbusTask(
      &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
  sbusTask.start();

  // temporary hack: microstrain logger
  PeriodicMemberFunction<MiniCheetahHardwareBridge> microstrainLogger(
      &taskManager, .001, "microstrain-logger", &MiniCheetahHardwareBridge::logMicrostrain, this);
  microstrainLogger.start();

  for (;;) {
    usleep(1000000);
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

void MiniCheetahHardwareBridge::runMicrostrain() {
  while(true) {
    _microstrainImu.run();

#ifdef USE_MICROSTRAIN
    _vectorNavData.accelerometer = _microstrainImu.acc;
    _vectorNavData.quat[0] = _microstrainImu.quat[1];
    _vectorNavData.quat[1] = _microstrainImu.quat[2];
    _vectorNavData.quat[2] = _microstrainImu.quat[3];
    _vectorNavData.quat[3] = _microstrainImu.quat[0];
    _vectorNavData.gyro = _microstrainImu.gyro;
#endif
  }


}

void MiniCheetahHardwareBridge::logMicrostrain() {
  _microstrainImu.updateLCM(&_microstrainData);
  _microstrainLcm.publish("microstrain", &_microstrainData);
}

void Cheetah3HardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
  printf("[Cheetah 3 Hardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) {
    printf("Vectornav failed to initialize\n");
    printf_color(PrintColor::Red, "****************\n"
                                  "**  WARNING!  **\n"
                                  "****************\n"
                                  "  IMU DISABLED  \n"
                                  "****************\n\n");
    //initError("failed to initialize vectornav!\n", false);
  }
}

/*!
 * Initialize Mini Cheetah specific hardware
 */
void MiniCheetahHardwareBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
#ifndef USE_MICROSTRAIN
  printf("[MiniCheetahHardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) {
    printf("Vectornav failed to initialize\n");
    //initError("failed to initialize vectornav!\n", false);
  }
#endif

  init_spi();
  _microstrainInit = _microstrainImu.tryInit(0, 460800);//921600); // lord设置921600,
}

/*!
 * Run Mini Cheetah SPI
 */
void MiniCheetahHardwareBridge::runSpi() {
  spi_command_t* cmd = get_spi_command();
  spi_data_t* data = get_spi_data();

  memcpy(cmd, &_spiCommand, sizeof(spi_command_t));
  spi_driver_run();
  memcpy(&_spiData, data, sizeof(spi_data_t));

  _spiLcm.publish("spi_data", data);
  _spiLcm.publish("spi_command", cmd);
}

/*
 * ======================================================================================
 * Following class functions are specifically defined for Cheetah 3
 */
void Cheetah3HardwareBridge::runEcat() {
  rt_ethercat_set_command(_tiBoardCommand);
  rt_ethercat_run();
  rt_ethercat_get_data(_tiBoardData);

  publishEcatLCM();
}

void Cheetah3HardwareBridge::publishEcatLCM() {
  for(int leg = 0; leg < 4; leg++) {
    ecatCmdLcm.x_des[leg] = _tiBoardCommand[leg].position_des[0];
    ecatCmdLcm.y_des[leg] = _tiBoardCommand[leg].position_des[1];
    ecatCmdLcm.z_des[leg] = _tiBoardCommand[leg].position_des[2];
    ecatCmdLcm.dx_des[leg] = _tiBoardCommand[leg].velocity_des[0];
    ecatCmdLcm.dy_des[leg] = _tiBoardCommand[leg].velocity_des[1];
    ecatCmdLcm.dz_des[leg] = _tiBoardCommand[leg].velocity_des[2];
    ecatCmdLcm.kpx[leg] = _tiBoardCommand[leg].kp[0];
    ecatCmdLcm.kpy[leg] = _tiBoardCommand[leg].kp[1];
    ecatCmdLcm.kpz[leg] = _tiBoardCommand[leg].kp[2];
    ecatCmdLcm.kdx[leg] = _tiBoardCommand[leg].kd[0];
    ecatCmdLcm.kdy[leg] = _tiBoardCommand[leg].kd[1];
    ecatCmdLcm.kdz[leg] = _tiBoardCommand[leg].kd[2];
    ecatCmdLcm.enable[leg] = _tiBoardCommand[leg].enable;
    ecatCmdLcm.zero_joints[leg] = _tiBoardCommand[leg].zero;
    ecatCmdLcm.fx_ff[leg] = _tiBoardCommand[leg].force_ff[0];
    ecatCmdLcm.fy_ff[leg] = _tiBoardCommand[leg].force_ff[1];
    ecatCmdLcm.fz_ff[leg] = _tiBoardCommand[leg].force_ff[2];
    ecatCmdLcm.tau_abad_ff[leg] = _tiBoardCommand[leg].tau_ff[0];
    ecatCmdLcm.tau_hip_ff[leg] = _tiBoardCommand[leg].tau_ff[1];
    ecatCmdLcm.tau_knee_ff[leg] = _tiBoardCommand[leg].tau_ff[2];
    ecatCmdLcm.q_des_abad[leg] = _tiBoardCommand[leg].q_des[0];
    ecatCmdLcm.q_des_hip[leg] = _tiBoardCommand[leg].q_des[1];
    ecatCmdLcm.q_des_knee[leg] = _tiBoardCommand[leg].q_des[2];
    ecatCmdLcm.qd_des_abad[leg] = _tiBoardCommand[leg].qd_des[0];
    ecatCmdLcm.qd_des_hip[leg] = _tiBoardCommand[leg].qd_des[1];
    ecatCmdLcm.qd_des_knee[leg] = _tiBoardCommand[leg].qd_des[2];
    ecatCmdLcm.kp_joint_abad[leg] = _tiBoardCommand[leg].kp_joint[0];
    ecatCmdLcm.kp_joint_hip[leg] = _tiBoardCommand[leg].kp_joint[1];
    ecatCmdLcm.kp_joint_knee[leg] = _tiBoardCommand[leg].kp_joint[2];
    ecatCmdLcm.kd_joint_abad[leg] = _tiBoardCommand[leg].kd_joint[0];
    ecatCmdLcm.kd_joint_hip[leg] = _tiBoardCommand[leg].kd_joint[1];
    ecatCmdLcm.kd_joint_knee[leg] = _tiBoardCommand[leg].kd_joint[2];
    ecatCmdLcm.max_torque[leg] = _tiBoardCommand[leg].max_torque;
  }

  for(int leg = 0; leg < 4; leg++) {
    ecatDataLcm.x[leg] = _tiBoardData[leg].position[0];
    ecatDataLcm.y[leg] = _tiBoardData[leg].position[1];
    ecatDataLcm.z[leg] = _tiBoardData[leg].position[2];
    ecatDataLcm.dx[leg] = _tiBoardData[leg].velocity[0];
    ecatDataLcm.dy[leg] = _tiBoardData[leg].velocity[1];
    ecatDataLcm.dz[leg] = _tiBoardData[leg].velocity[2];
    ecatDataLcm.fx[leg] = _tiBoardData[leg].force[0];
    ecatDataLcm.fy[leg] = _tiBoardData[leg].force[1];
    ecatDataLcm.fz[leg] = _tiBoardData[leg].force[2];
    ecatDataLcm.q_abad[leg] = _tiBoardData[leg].q[0];
    ecatDataLcm.q_hip[leg] = _tiBoardData[leg].q[1];
    ecatDataLcm.q_knee[leg] = _tiBoardData[leg].q[2];
    ecatDataLcm.dq_abad[leg] = _tiBoardData[leg].dq[0];
    ecatDataLcm.dq_hip[leg] = _tiBoardData[leg].dq[1];
    ecatDataLcm.dq_knee[leg] = _tiBoardData[leg].dq[2];
    ecatDataLcm.tau_abad[leg] = _tiBoardData[leg].tau[0];
    ecatDataLcm.tau_hip[leg] = _tiBoardData[leg].tau[1];
    ecatDataLcm.tau_knee[leg] = _tiBoardData[leg].tau[2];
    ecatDataLcm.tau_des_abad[leg] = _tiBoardData[leg].tau_des[0];
    ecatDataLcm.tau_des_hip[leg] = _tiBoardData[leg].tau_des[1];
    ecatDataLcm.tau_des_knee[leg] = _tiBoardData[leg].tau_des[2];
    ecatDataLcm.loop_count_ti[leg] = _tiBoardData[leg].loop_count_ti;
    ecatDataLcm.ethercat_count_ti[leg] = _tiBoardData[leg].ethercat_count_ti;
    ecatDataLcm.microtime_ti[leg] = _tiBoardData[leg].microtime_ti;
  }

  _ecatLCM.publish("ecat_cmd", &ecatCmdLcm);
  _ecatLCM.publish("ecat_data", &ecatDataLcm);
}

/*!
 * Send LCM visualization data
 */
void HardwareBridge::publishVisualizationLCM() {
  cheetah_visualization_lcmt visualization_data;
  for (int i = 0; i < 3; i++) {
    visualization_data.x[i] = _mainCheetahVisualization.p[i];
  }

  for (int i = 0; i < 4; i++) {
    visualization_data.quat[i] = _mainCheetahVisualization.quat[i];
    visualization_data.rgba[i] = _mainCheetahVisualization.color[i];
  }

  for (int i = 0; i < 12; i++) {
    visualization_data.q[i] = _mainCheetahVisualization.q[i];
  }

  _visualizationLCM.publish("main_cheetah_visualization", &visualization_data);
}

Cheetah3HardwareBridge::Cheetah3HardwareBridge(RobotController *rc) : HardwareBridge(rc),  _ecatLCM(getLcmUrl(255)) {

}

void Cheetah3HardwareBridge::run() {
  initCommon();
  initHardware();

  printf("[Hardware Bridge] Loading parameters over LCM...\n");
  while (!_robotParams.isFullyInitialized()) {
    printf("[Hardware Bridge] Waiting for robot parameters...\n");
    usleep(1000000);
  }

  if(_userControlParameters) {
    while (!_userControlParameters->isFullyInitialized()) {
      printf("[Hardware Bridge] Waiting for user parameters...\n");
      usleep(1000000);
    }
  }

  printf("[Hardware Bridge] Got all parameters, starting up!\n");

  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->tiBoardData = _tiBoardData;
  _robotRunner->tiBoardCommand = _tiBoardCommand;
  _robotRunner->robotType = RobotType::CHEETAH_3;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
  _robotRunner->vectorNavData = &_vectorNavData;

  _robotRunner->init();
  _firstRun = false;

  // init control thread

  statusTask.start();

  rt_ethercat_init();
  // Ecat Task start
  PeriodicMemberFunction<Cheetah3HardwareBridge> ecatTask(
      &taskManager, .001, "ecat", &Cheetah3HardwareBridge::runEcat, this);
  ecatTask.start();

  // robot controller start
  _robotRunner->start();

  // visualization start
  PeriodicMemberFunction<Cheetah3HardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

// rc controller disabled for now
// sbus remoter
  _port = init_sbus(false);  // Not Simulation
  PeriodicMemberFunction<HardwareBridge> sbusTask(&taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
  sbusTask.start();


  for (;;) {
    usleep(100000);
    taskManager.printStatus();
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

/*
 * ======================================================================================
 * Following class functions are specifically defined for IUST robot
 */
IUSTrobotHardwareBridge::IUSTrobotHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)
        : HardwareBridge(robot_ctrl), _spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255)) {
    _load_parameters_from_file = load_parameters_from_file;
}
/*!
 * Initialize IUST specific hardware
 */
void IUSTrobotHardwareBridge::initHardware() {
    _vectorNavData.quat << 1, 0, 0, 0;
    printf("[IUSTHardware] I am using hardwareBridge:)\n");
#ifndef USE_MICROSTRAIN
    printf("[IUSTHardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) {
    printf("Vectornav failed to initialize\n");
    //initError("failed to initialize vectornav!\n", false);
  }
#endif
    CANable.init_can();
    _microstrainInit = _microstrainImu.tryInit(0,460800 );//921600
}
/*!
 * Main method for IUST robot hardware
 */
void IUSTrobotHardwareBridge::run() {
    initCommon();
    initHardware();

    if(_load_parameters_from_file) {
        printf("[Hardware Bridge] Loading parameters from file...\n");

        try {
            _robotParams.initializeFromYamlFile(THIS_COM "config/iust-robot-parameters.yaml");
        } catch(std::exception& e) {
            printf("[Hardware Bridge] Failed to initialize robot parameters from yaml file: %s\n", e.what());
            exit(1);
        }

        if(!_robotParams.isFullyInitialized()) {
            printf("[Hardware Bridge] Failed to initialize all robot parameters\n");
            exit(1);
        }

        printf("[Hardware Bridge] Loaded robot parameters\n");

        if(_userControlParameters) {
            try {
                #ifdef CMPC_CTRL
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/iust-user-parameters-full.yaml");
                std::string yamlName = "iust-user-parameters.yaml";
                printf("[Hardware Bridge] Loaded user parameters from yaml file: %s\n", yamlName.c_str());
                #endif
                #ifdef JPOS_CTRL
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/jpos-user-parameters.yaml");
                std::string yamlName = "jpos-user-parameters.yaml";
                printf("[Hardware Bridge] Loaded user parameters from yaml file: %s\n", yamlName.c_str());
                #endif
                #ifdef SPI_CTRL
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/no-parameters.yaml");
                std::string yamlName = "no-parameters.yaml";
                printf("[Hardware Bridge] Loaded user parameters from yaml file: %s\n", yamlName.c_str());
                #endif
                #ifdef LOWLEVEL_CTRL
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/lowlevel-user-parameters.yaml");
                std::string yamlName = "lowlevel-user-parameters.yaml";
                printf("[Hardware Bridge] Loaded user parameters from yaml file: %s\n", yamlName.c_str());
                #endif
                #ifdef IUST_CTRL
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/iust-user-parameters.yaml");
                std::string yamlName = "iust-user-parameters.yaml";
                printf("[Hardware Bridge] Loaded user parameters from yaml file: %s\n", yamlName.c_str());
                #endif

            } catch(std::exception& e) {
                printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
                exit(1);
            }

            if(!_userControlParameters->isFullyInitialized()) {
                printf("[Hardware Bridge] Failed to initialize all user parameters\n");
                exit(1);
            }

            printf("[Hardware Bridge] Loaded user parameters\n");
        } else {
            printf("[Hardware Bridge] Did not load user parameters because there aren't any\n");
        }
    } else {
        printf("[Hardware Bridge] Loading parameters over LCM...\n");
        while (!_robotParams.isFullyInitialized()) {
            printf("[Hardware Bridge] Waiting for robot parameters...\n");
            usleep(1000000);
        }

        if(_userControlParameters) {
            while (!_userControlParameters->isFullyInitialized()) {
                printf("[Hardware Bridge] Waiting for user parameters...\n");
                usleep(1000000);
            }
        }
    }



    printf("[Hardware Bridge] Got all parameters, starting up!\n");

    _robotRunner = new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

    _robotRunner->driverCommand = &_gamepadCommand;
    _robotRunner->spiData = &_spiData;
    _robotRunner->spiCommand = &_spiCommand;
    _robotRunner->robotType = RobotType::IUST;
    _robotRunner->vectorNavData = &_vectorNavData;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData = &_visualizationData;
    _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

    _firstRun = false;

    // init control thread

    statusTask.start();

    // spi Task start
    PeriodicMemberFunction<IUSTrobotHardwareBridge> canTask(
            &taskManager, .002, "can", &IUSTrobotHardwareBridge::runCAN, this);
    canTask.start();

    // microstrain
    if(_microstrainInit)
        _microstrainThread = std::thread(&IUSTrobotHardwareBridge::runMicrostrain, this);

    // robot controller start
    _robotRunner->start();

    // visualization start
    PeriodicMemberFunction<IUSTrobotHardwareBridge> visualizationLCMTask(
            &taskManager, .0167, "lcm-vis",
            &IUSTrobotHardwareBridge::publishVisualizationLCM, this);
    visualizationLCMTask.start();

    // rc controller
    _port = init_sbus(false);  // Not Simulation
    PeriodicMemberFunction<HardwareBridge> sbusTask(
            &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
    sbusTask.start();

    // temporary hack: microstrain logger
    PeriodicMemberFunction<IUSTrobotHardwareBridge> microstrainLogger(
            &taskManager, .001, "microstrain-logger", &IUSTrobotHardwareBridge::logMicrostrain, this);
    microstrainLogger.start();

    for (;;) {
        usleep(1000000);
        // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
    }
}

void IUSTrobotHardwareBridge::runMicrostrain() {
    printf("[HardwareBridge] Start microstrain\n");
    u64 imu_times=0;

    while (true) {
        _microstrainImu.run();

        #ifdef USE_MICROSTRAIN
        _vectorNavData.accelerometer = _microstrainImu.acc;
        _vectorNavData.quat[0] = _microstrainImu.quat[1];
        _vectorNavData.quat[1] = _microstrainImu.quat[2];
        _vectorNavData.quat[2] = _microstrainImu.quat[3];
        _vectorNavData.quat[3] = _microstrainImu.quat[0];
        _vectorNavData.gyro = _microstrainImu.gyro;
        #endif
        imu_times++;
        #ifdef IMU_DEBUG_SHOW
        if(imu_times%1000==0)
        {
            printf("Iteration stamp:\t%d\n",(int)imu_times);
            printf("--------------------------------------------------\n");
            printf("ACC = [%f, %f, %f]\n", _imuData.accelerometer[0], _imuData.accelerometer[1], _imuData.accelerometer[2]);
            printf("QUAT = [%f, %f, %f, %f]\n", _imuData.quat[0], _imuData.quat[1], _imuData.quat[2], _imuData.quat[3]);
            printf("GYRO =[%f, %f, %f]\n", _imuData.gyro[0], _imuData.gyro[1], _imuData.gyro[2]);
            printf("--------------------------------------------------\n");
        }
        #endif
    }
}

void IUSTrobotHardwareBridge::logMicrostrain() {
    _microstrainImu.updateLCM(&_microstrainData);
    _microstrainLcm.publish("microstrain", &_microstrainData);
}
/*!
 * Run IUST Cheetah SPI
 */
void IUSTrobotHardwareBridge::runCAN() {
    std::cout << ":)))\n";
    actuator_command_t* cmd = CANable.get_can_command(); 
    actuator_response_t* data = CANable.get_can_data();

    // Send and Receive data and commands through CAN hardware for leg-level controller
    CANable.can_send_receive(cmd, data);

    #ifdef SPI_DEBUG_SHOW
    if(spi_times%1000==0)
        {
            printf("Iteration stamp:\t%d\n",(int)spi_times);
            printf("--------------------DATA--------------------------\n");
            printf("ABAD Q = [%f, %f, %f, %f]\n", data->q_abad[0],data->q_abad[1],data->q_abad[2],data->q_abad[3]);
            printf("HIP  Q = [%f, %f, %f, %f]\n", data->q_hip[0],data->q_hip[1],data->q_hip[2],data->q_hip[3]);
            printf("KNEE Q = [%f, %f, %f, %f]\n", data->q_knee[0],data->q_knee[1],data->q_knee[2],data->q_knee[3]);
            printf("ABAD QD = [%f, %f, %f, %f]\n", data->qd_abad[0],data->qd_abad[1],data->qd_abad[2],data->qd_abad[3]);
            printf("HIP  QD = [%f, %f, %f, %f]\n", data->qd_hip[0],data->qd_hip[1],data->qd_hip[2],data->qd_hip[3]);
            printf("KNEE QD = [%f, %f, %f, %f]\n", data->qd_knee[0],data->qd_knee[1],data->qd_knee[2],data->qd_knee[3]);
            printf("-------------------COMMAND------------------------\n");
            printf("ABAD DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_abad[0],_spiCommand.q_des_abad[1],_spiCommand.q_des_abad[2],_spiCommand.q_des_abad[3]);
            printf("HIP  DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_hip[0],_spiCommand.q_des_hip[1],_spiCommand.q_des_hip[2],_spiCommand.q_des_hip[3]);
            printf("KNEE DES Q = [%f, %f, %f, %f]\n", _spiCommand.q_des_knee[0],_spiCommand.q_des_knee[1],_spiCommand.q_des_knee[2],_spiCommand.q_des_knee[3]);
            printf("KP A= [%f, %f, %f, %f]\n", _spiCommand.kp_abad[0],_spiCommand.kp_abad[1],_spiCommand.kp_abad[2],_spiCommand.kp_abad[3]);
            printf("KP H= [%f, %f, %f, %f]\n", _spiCommand.kp_hip[0],_spiCommand.kp_hip[1],_spiCommand.kp_hip[2],_spiCommand.kp_hip[3]);
            printf("KP K= [%f, %f, %f, %f]\n", _spiCommand.kp_knee[0],_spiCommand.kp_knee[1],_spiCommand.kp_knee[2],_spiCommand.kp_knee[3]);
            printf("KD A= [%f, %f, %f, %f]\n", _spiCommand.kd_abad[0],_spiCommand.kd_abad[1],_spiCommand.kd_abad[2],_spiCommand.kd_abad[3]);
            printf("KD H= [%f, %f, %f, %f]\n", _spiCommand.kd_hip[0],_spiCommand.kd_hip[1],_spiCommand.kd_hip[2],_spiCommand.kd_hip[3]);
            printf("KD K= [%f, %f, %f, %f]\n", _spiCommand.kd_knee[0],_spiCommand.kd_knee[1],_spiCommand.kd_knee[2],_spiCommand.kd_knee[3]);
            printf("ABAD DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_abad_ff[0],_spiCommand.tau_abad_ff[1],_spiCommand.tau_abad_ff[2],_spiCommand.tau_abad_ff[3]);
            printf("HIP  DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_hip_ff[0],_spiCommand.tau_hip_ff[1],_spiCommand.tau_hip_ff[2],_spiCommand.tau_hip_ff[3]);
            printf("KNEE DES T = [%f, %f, %f, %f]\n", _spiCommand.tau_knee_ff[0],_spiCommand.tau_knee_ff[1],_spiCommand.tau_knee_ff[2],_spiCommand.tau_knee_ff[3]);

            printf("--------------------------------------------------\n");
        }
    #endif

    // Nobody subscribe following lcm
    _spiLcm.publish("spi_data", data);
    _spiLcm.publish("spi_command", cmd);
}

#endif
