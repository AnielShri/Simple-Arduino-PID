//	----------------------------------------------------------------------------+
//	constants
//	----------------------------------------------------------------------------+

/* hardware <-> software mapping */
const byte PWM_PIN = 6;
const byte ANALOG_PIN = A2;
const byte ENCODER_PIN = 3;
const byte ENABLE_PIN = A5;

/* interval configurations */
const int ENCODER_INTERVAL = 25;
const int ANALOG_INTERVAL = 250;
const int ENABLE_INTERVAL = 1000;
const int SERIAL_INTERVAL = 50;

/* normalize encoder count to rotation speed */
const float ENCODER_RATIO = ((1000 / ENCODER_INTERVAL) / 10) * 1.8;

//	----------------------------------------------------------------------------+
//	variables
//	----------------------------------------------------------------------------+

/* interval variables */
unsigned long encoder_prev_time;
unsigned long analog_prev_time;
unsigned long enable_prev_time;
unsigned long serial_prev_time;

/* encoder variables */
volatile int enc_count;
float enc_freq;
float enc_avg;
float enc_window[3];
byte enc_window_index;

bool enable_state;

//	----------------------------------------------------------------------------+
//	encoder isr
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
		if(enc_window_index > 2) enc_window_index = 0;
		enc_avg = (enc_window[0] + enc_window[1] + enc_window[2])/3;

	}

	/* for graphing data */
	else if (now >= serial_prev_time + SERIAL_INTERVAL)
	{
		serial_prev_time = now;		
		Serial.print(enc_freq, 2);		
		Serial.print("\t");		
		Serial.println(enc_avg, 2);
	}

	/* motor ON/OFF */
	else if (now >= enable_prev_time + ENABLE_INTERVAL)
	{
		enable_prev_time = now;
		
		int val = analogRead(ENABLE_PIN);
		if (val >= 1000)
		{
			enable_state = true;
		}
		else
		{
			enable_state = false;
		}
	}	

	/* manual control */
	else if (now >= analog_prev_time + ANALOG_INTERVAL)
	{
		analog_prev_time = now;

		if (enable_state == true)
		{
			int val = analogRead(ANALOG_PIN);
			analogWrite(PWM_PIN, val / 4);
		}
		else
		{
			analogWrite(PWM_PIN, 0);
		}
	}

	
}
