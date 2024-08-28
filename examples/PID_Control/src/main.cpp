#include <Arduino.h>

//TEST
#include <hal/gpio_hal.h>

#include <DShotRMT.h>

// USB serial port needed for this example
const int USB_SERIAL_BAUD = 115200;
#define USB_Serial Serial

// Define the GPIO pin connected to the motor and the DShot protocol used
const auto MOTOR01_PIN = 23;
const auto MOTOR02_PIN = 18;
const auto DSHOT_MODE = DSHOT600;

// Define the failsafe and initial throttle values
const auto FAILSAFE_THROTTLE = 0;
const auto INITIAL_THROTTLE = 48;

//is the handle trigger pin
#define TRIGGER_PIN 17
#define MAG_PIN 5
//SCL on the PCB
#define POT_PIN 33
#define SUPPLY_PIN 25

DShotRMT anESC(MOTOR02_PIN);//MOTOR01_PIN
DShotRMT anotherESC(MOTOR01_PIN);//MOTOR02_PIN

//good practice: declare vars and methods, in the header file, and define methods in the source file.
//implement PID control for the ESC speed
class PIDController {

	public:

	PIDController(
		int32_t start_output_offset,
		int32_t max_lim = DSHOT_THROTTLE_MAX,
		int32_t min_lim = DSHOT_THROTTLE_MIN,
		float p_const = 0.2,
		float i_const = 0.2,
		float d_const = 0.2
	) {
		if(start_output_offset > max_lim)
			start_output_offset = max_lim;
		

		this->start_output_offset = start_output_offset;
		this->max_lim = max_lim;
		this->min_lim = min_lim;

		this->p_const = p_const;
		this->i_const = i_const;
		this->d_const = d_const;

	};

	~PIDController() {
	};


	int32_t tick(uint32_t target_rpm, uint32_t curr_rpm, uint32_t delta_micros) {

		//small RPM = big throttle value, direct acting

		delta_micros = 200; //for testing

		//determine PID values


		//i don't know why I have to add the 0.0 in order for this variable to stick.
		float pval = (float)target_rpm - (float)curr_rpm;


		//if our value is increasing/decreasing and we limited the output, don't continue integration
		if(
			!(
			(pval > 0.0 && has_limited_high) ||
			(pval < 0.0 && has_limited_low)
			)
		) {
			ival += pval * i_const * (float)delta_micros;
		}

		float dval = pval - last_pval;
		last_pval = pval;

		//sum and multiply by constants
		int32_t pid_out = (int32_t)(
			pval * p_const * (float)delta_micros
			+ ival //integral constant is applied above
			+ dval * d_const * (float)delta_micros
		) + start_output_offset;

		//limit output
		if(pid_out > max_lim) {
			pid_out = max_lim;
			has_limited_high = true;
			has_limited_low = false;
		} else if (pid_out < min_lim) {
			pid_out = min_lim;
			has_limited_high = false;
			has_limited_low = true;
		} else {
			has_limited_high = false;
			has_limited_low = false;
		}

		return(pid_out);
	
	}

	//public so we can change this depending on what speed we want to start at
	int32_t start_output_offset;

	private:

	//vars
	float p_const;
	float i_const;
	float d_const;

	int32_t max_lim = DSHOT_THROTTLE_MAX;
	int32_t min_lim = DSHOT_THROTTLE_MIN;

	//data retainers
	float last_pval = 0;
	float ival = 0;

	bool has_limited_high = false;
	bool has_limited_low = false;

};


PIDController* controller;

int loop_count = 0;
uint32_t last_millis = 0;

//time when we sent the last packet
uint32_t last_micros_sent = 0;

//time when we got the last packet
uint32_t last_micros_got = 0;

//persistent PID values
uint32_t last_good_rpm = 0; //feedback RPM
uint32_t esc_throttle = DSHOT_THROTTLE_MIN; //value written to the ESC
uint32_t target_rpm = 0; //RPM we want the ESCs to run at



void setup()
{

	USB_Serial.begin(USB_SERIAL_BAUD);
	anESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);
	anotherESC.begin(DSHOT_MODE, ENABLE_BIDIRECTION, 14);

	pinMode(TRIGGER_PIN, INPUT_PULLUP);
	pinMode(SUPPLY_PIN, OUTPUT);
	digitalWrite(SUPPLY_PIN, HIGH);

	pinMode(POT_PIN, INPUT);

	controller = new PIDController(
		DSHOT_THROTTLE_MIN, //start output
		DSHOT_THROTTLE_MAX, //min output lim
		DSHOT_THROTTLE_MIN, //max output lim
		0.0012, //p
		0.00015, //i
		0.002 //d
	);

	Serial.println("starting");
	
}


void loop()
{

	uint16_t rpm_set_now = 400;
	//start halted
	controller->start_output_offset = 0;
	
	//throttle control sub-section
	{
		//update our setpoint
		if(digitalRead(MAG_PIN) == 0) {
			uint16_t tpot_value = analogRead(POT_PIN);
			uint16_t pot_value = abs(tpot_value - pot_value) > 50 ? tpot_value : pot_value;

			target_rpm = map(pot_value, 0, 4095, 0, 32200);
			if(target_rpm > 32200) target_rpm = 32200;
			if(target_rpm < 0) target_rpm = 0;
			Serial.println(target_rpm);

		}
		//execute our setpoint
		if(digitalRead(TRIGGER_PIN) == 0) {
			rpm_set_now = target_rpm;

			//start full throttle (we should possible ramp this if we're not shooting at high RPMS)
			controller->start_output_offset = DSHOT_THROTTLE_MAX;
		}

	}
	

	//ESC control section
	{
		uint32_t dshot_rpm;
		dshot_get_packet_exit_mode_t response = anESC.get_dshot_packet(&dshot_rpm);

		//good timeout values:
		//dshot300: 300micros
		//dshot600: 150micros
		//using timeout values shorter than this will result in our favorite uninitialized errors (and maybe the `~` to boot) because we're trying to send while getting stuff back
		//it also seems that we can't just up and send a frame right back as soon as we get one, because the ESC doesn't always respond right away.

		//resend a packet as soon as we get a successful rpm value or if we've timed out.
		uint32_t mcro = micros();

		//we got a good value back, update our PID loop
		if(response == DECODE_SUCCESS) {
			esc_throttle = (uint16_t)controller->tick(rpm_set_now, dshot_rpm, mcro - last_micros_got);

			last_micros_got = mcro;

			//Serial.printf("%d | %d || %d\n", dshot_rpm, rpm_set_now, esc_throttle);

		}

		//for some reason, it's stepping on it. I'm going to force regular intervals
		//if(response == DECODE_SUCCESS || mcro - last_micros_sent > 150) {
		if(mcro - last_micros_sent > 200) {
			anESC.send_dshot_value(esc_throttle);
			last_micros_sent = mcro;
		}

	}


	

	/*
	//old control stuff


	// if(Serial.available())
	// {
	// 	while(Serial.available()){Serial.read();}

	// 	Serial.printf("Ready for RX\n");
	// 	anESC.send_dshot_value(INITIAL_THROTTLE);
	// }

	static bool tick = false;
	if(millis() % 2 == 0 && tick == false)
	{

		if(loopCount < 1600)
		{
			anESC.send_dshot_value(INITIAL_THROTTLE);
			anotherESC.send_dshot_value(INITIAL_THROTTLE);
		}
		else if(loopCount < 2000)
		{

			anESC.send_dshot_value(100);
			anotherESC.send_dshot_value(100);
		}
		else
		{
			anESC.send_dshot_value(100);
			anotherESC.send_dshot_value(100);
		}

		tick = true;
		loopCount += 1;


		if(loopCount % 10 == 0)
		{
			uint32_t rpm_1 = 0;
			extended_telem_type_t telem = TELEM_TYPE_ERPM; //telemetry argument is optional
			int error_a = anESC.get_dshot_packet(&rpm_1, &telem);
			uint32_t rpm_2 = 0;
			int error_b = anotherESC.get_dshot_packet(&rpm_2);



			Serial.printf("%10d, %10.3f || %10d, %10.3f\n",
				rpm_1, anESC.get_telem_success_rate(),
				rpm_2, anotherESC.get_telem_success_rate());

			//Serial.printf("%10d, %10d, %10.3f\n",
			//	error_a, rpm_1, anESC.get_telem_success_rate());
		}



	}
	else if(millis() % 2 != 0)
	{
		tick = false;
	}

	*/


}


