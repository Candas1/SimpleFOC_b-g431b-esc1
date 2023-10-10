#include <Arduino.h>
#include <SimpleFOC.h>
#include <RTTStream.h>

RTTStream rtt;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(15, 0.1664, 16.0, 0.00036858);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// current sensor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Commander interface constructor
Commander command = Commander(rtt);

void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doMotor(char* cmd){ command.motor(&motor,cmd); }

void setup(){
	pinMode(LED_BUILTIN, OUTPUT);

	// driver config
	driver.voltage_power_supply = 26; // 3.6 * BAT_CELLS; // power supply voltage [V]
	driver.pwm_frequency = 16000;
	driver.dead_zone = 0.03;
	driver.init();

	// link the driver to the current sense
    current_sense.linkDriver(&driver);

	motor.linkDriver(&driver);  // link driver
	motor.voltage_sensor_align  = 2;                            // aligning voltage
	motor.foc_modulation        = FOCModulationType::SpaceVectorPWM; // Only with Current Sense
	motor.controller            = MotionControlType::velocity_openloop;    // set motion control loop to be used
	motor.torque_controller     = TorqueControlType::foc_current;
	
	motor.voltage_limit = driver.voltage_power_supply * 0.05;
    motor.flux_linkage = 0.00138;

    SimpleFOCDebug::enable(&rtt);
	//motor.useMonitoring(rtt);
	motor.monitor_downsample = 100; // set downsampling can be even more > 100
  	motor.monitor_variables =  _MON_VEL;// _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents _MON_TARGET | _MON_VEL | _MON_ANGLE |
	motor.monitor_decimals = 2; //!< monitor outputs decimal places
    

	// init motor hardware
	motor.init();

	// current sense init hardware
	//current_sense.skip_align = true;
	current_sense.init();
	// link the current sense to the motor
	motor.linkCurrentSense(&current_sense);

	// align sensor and start FOC
	//motor.sensor_direction= Direction::CW;
    //motor.zero_electric_angle = 0;     // use the real value!
	//motor.motion_downsample = 4;
	//motor.initFOC();

	// set the initial motor target
	motor.target = 0; // unit depends on control mode 

	// add target command T
	command.add('T', doTarget, "target angle");
	command.add('M',doMotor,"my motor motion");
	_delay(100);
}

float target = 0.01;
LowPassFilter LPF_target(0.5);  //  the higher the longer new values need to take effect
PhaseCurrent_s currents;
float angle_el;

void loop(){

	static int i = 0;
	static int millis_prev = 0;
	i ++;

	if(millis()-millis_prev>1000){
		//rtt.print("Main frq: "); rtt.println(i);
		i=0;
		millis_prev = millis();
	}
	digitalWrite(LED_BUILTIN, millis()%2000>1000);

	//motor.loopFOC();
	motor.move(target);
	//motor.monitor();
	command.run();

    // read current phase currents
    motor.PhaseCurrent = current_sense.getPhaseCurrents();

    // calculate clarke transform
    motor.ABCurrent = current_sense.getABCurrents(motor.PhaseCurrent);

	float Ts = ( _micros() - motor.observer_timestamp) * 1e-6f; 
	motor.flux_a = _constrain( motor.flux_a + (motor.ABVoltage.alpha - motor.phase_resistance * motor.ABCurrent.alpha) * Ts -
				motor.phase_inductance * (motor.ABCurrent.alpha - motor.ABCurrent_prev.alpha),-motor.flux_linkage, motor.flux_linkage);
	motor.flux_b = _constrain( motor.flux_b + (motor.ABVoltage.beta - motor.phase_resistance * motor.ABCurrent.beta) * Ts -
				motor.phase_inductance * (motor.ABCurrent.beta - motor.ABCurrent_prev.beta) ,-motor.flux_linkage, motor.flux_linkage);
	motor.ABCurrent_prev = motor.ABCurrent;
	motor.observer_timestamp = _micros();

	float tmp_angle_el = atan2(motor.flux_b,motor.flux_a);
	// Handle wraparound
	tmp_angle_el += tmp_angle_el < 0 ? _2PI : ( tmp_angle_el >= _2PI ? -_2PI : 0 );  
	angle_el = tmp_angle_el;

}