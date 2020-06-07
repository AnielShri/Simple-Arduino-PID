//	----------------------------------------------------------------------------+
//	Closed loop PID demo 
//
//	based on: 
//	https://www.teachmemicro.com/arduino-pid-control-tutorial/
//	----------------------------------------------------------------------------+


//	----------------------------------------------------------------------------+
//	constants
//	----------------------------------------------------------------------------+

/* hardware <-> software mapping */
const byte PWM_PIN = 6;
const byte ENCODER_PIN = 3;
const byte ENABLE_PIN = A5;

/* interval configurations */
const int ENCODER_INTERVAL = 33;
const int ENABLE_INTERVAL = 1000;
const int SERIAL_INTERVAL = 100;
const int PID_INTERVAL = 50;

/* normalize encoder count to rotation speed */
const float ENCODER_RATIO = ((1000 / ENCODER_INTERVAL) / 10) * 1.8;	// 56Hz -> 100%

/* PID controller configurations */
const float SET_POINT = 100.0;						// setpoint = 100%
const float PID_DT = (float)PID_INTERVAL / 1000.0;	// ms -> seconds

const float Kp = 1;
const float Ki = 2;		
const float Kd = 0.05;

//	----------------------------------------------------------------------------+
//	variables
//	----------------------------------------------------------------------------+

/* interval variables */
unsigned long encoder_prev_time;
unsigned long enable_prev_time;
unsigned long serial_prev_time;
unsigned long pid_prev_time;

/* encoder variables */
unsigned int enc_count;
float enc_freq;
float enc_avg;
float enc_window[3];
byte enc_window_index;

byte enable_state;

/* PID controller variables*/
float error_integral;
float error_prev;
float pid_output;

//	----------------------------------------------------------------------------+
//	encoder interrupt
//	----------------------------------------------------------------------------+
void encoder_isr()
{
	enc_count++;
}

//	----------------------------------------------------------------------------+
//	initialization
//	----------------------------------------------------------------------------+
void setup() 
{
	pinMode(PWM_PIN, OUTPUT);
	pinMode(ENCODER_PIN, INPUT_PULLUP);

	Serial.begin(115200);
	attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoder_isr, RISING);
}

//	----------------------------------------------------------------------------+
//	infinite loop
//	----------------------------------------------------------------------------+
void loop() 
{
	unsigned long now = millis();

	/* convert encoder ticks to speed */
	if (now >= encoder_prev_time + ENCODER_INTERVAL)
	{
		encoder_prev_time = now;

		enc_freq = enc_count * ENCODER_RATIO;
		enc_count = 0;

		/* calculate moving average */
		enc_window[enc_window_index] = enc_freq;
		enc_window_index += 1;
		if (enc_window_index > 2) enc_window_index = 0;
		enc_avg = (enc_window[0] + enc_window[1] + enc_window[2])/3;

	}

	/* PID control */
	else if (now >= pid_prev_time + PID_INTERVAL)
	{
		pid_prev_time = now;

		float error = SET_POINT - enc_avg;

		error_integral += error * PID_DT;
		if(error_integral > 255) error_integral = 255;

		float error_dt = (error - error_prev) / PID_DT;
		error_prev = error;

		pid_output = Kp*error + Ki*error_integral + Kd*error_dt;
		
		if (pid_output > 255) pid_output = 255;
		else if (pid_output < 0) pid_output = 0;

		analogWrite(PWM_PIN, pid_output * enable_state);
	}

	/* for graphing data */
	else if (now >= serial_prev_time + SERIAL_INTERVAL)
	{
		serial_prev_time = now;	

		if (enable_state == 1)
		{
//			Serial.print(pid_output, 2);
//			Serial.print("\t");
			Serial.println(enc_avg, 2);
		}
	}

	/* motor ON/OFF */
	else if (now >= enable_prev_time + ENABLE_INTERVAL)
	{
		enable_prev_time = now;
		
		int val = analogRead(ENABLE_PIN);
		if (val >= 1000)
		{
			enable_state = 1;
		}
		else
		{
			enable_state = 0;

			/* reset stored PID values */
			error_integral = 0;
			error_prev = 0;
		}
	}
	
}
