#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/flux_observer/FluxObserverSensor.h>
#include <RTTStream.h>

RTTStream rtt;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(15, 0.1664, 16.0, 0.00036858);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// current sensor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

FluxObserverSensor sensor = FluxObserverSensor(motor);

// Commander interface constructor
Commander command = Commander(rtt);

void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char* cmd){ command.motor(&motor,cmd); }

void setup(){
	pinMode(LED_BUILTIN, OUTPUT);

    //sensor.init();
	motor.linkSensor(&sensor);

	// driver config
	driver.voltage_power_supply = 18;
	driver.pwm_frequency = 16000;
	driver.dead_zone = 0.03;
	driver.init();

	// link the driver to the current sense
    current_sense.linkDriver(&driver);

	motor.linkDriver(&driver);  // link driver
	motor.voltage_sensor_align  = 2;                            // aligning voltage
	motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
	motor.controller            = MotionControlType::torque;    // set motion control loop to be used
	motor.torque_controller     = TorqueControlType::foc_current;

	if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
		if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
		// When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
		    motor.voltage_limit = driver.voltage_power_supply * 0.54;
		}else{
		    motor.voltage_limit = driver.voltage_power_supply * 0.58;
		}
	}else{
		// For openloop angle and velocity modes, use very small limit
		motor.voltage_limit = driver.voltage_power_supply * 0.05;
	}

    SimpleFOCDebug::enable(&rtt);
	//motor.useMonitoring(rtt);
	motor.monitor_downsample = 100; // set downsampling can be even more > 100
  	motor.monitor_variables =  _MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 2; //!< monitor outputs decimal places
    
	// init motor hardware
	motor.init();

	// current sense init hardware
	current_sense.skip_align = true;
	current_sense.init();
	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	// align sensor and start FOC
	motor.sensor_direction= Direction::CW;
    motor.zero_electric_angle = 0;
	motor.initFOC();

	// set the initial motor target
	motor.target = 0; // unit depends on control mode 

	// add target command T
	command.add('T',doTarget, "target angle");
	command.add('M',doMotor,"my motor motion");
	_delay(100);
}

float target = 0;
LowPassFilter LPF_target(0.5);  //  the higher the longer new values need to take effect
PhaseCurrent_s currents;
float angle_el;

void loop(){

	if (motor.controller == MotionControlType::torque || motor.controller == MotionControlType::angle || motor.controller == MotionControlType::velocity){
		if (motor.torque_controller == TorqueControlType::foc_current || motor.torque_controller == TorqueControlType::dc_current){
		// When current sensing is used, reduce the voltage limit to have enough low side ON time for phase current sampling  
		    motor.voltage_limit = driver.voltage_power_supply * 0.54;
		}else{
		    motor.voltage_limit = driver.voltage_power_supply * 0.58;
		}
	}else{
		// For openloop angle and velocity modes, use very small limit
		motor.voltage_limit = driver.voltage_power_supply * 0.05;
	}

	motor.loopFOC();
	motor.move(LPF_target(target));
	//motor.monitor();
	command.run();
}