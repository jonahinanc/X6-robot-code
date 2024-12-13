/* 
  this File contains all Variables and functions for the Servo movement and the gaits 
*/

//Define Max and Min Pulsewidth of Batan Servo (Feet)
#define datan_servo_min 133 //30
#define datan_servo_max 517 //150 degrees
//Define the max and min Signal for feedback of Datan Servo 
#define datan_feedback_min 540 //30 degrees 
#define datan_feedback_max 2329 //150 degrees 

//Define Max and Min Pulsewidth of DSS Servo (Spine and Shoulder)
#define savoex_servo_min 196 // 30°
#define savoex_servo_max 530 // 150°
//Define the max and min Signal for feedback of DSS Servo 
#define savoex_feedback_min 555 //30 degrees (placeholder value !!)
#define savoex_feedback_max 1111 //150 degrees (placeholder value !!)

// savoex servos need a different pulsewidth when controlled through ESP and not PCA
#define savoex_spine_min 832 // 30
#define savoex_spine_max 2140 // 150

//Define Max and Min Pulsewidth of DMS Servo (Wrist)
#define dms_servo_min 400
#define dms_servo_max 155
//Define the max and min Signal for feedback of DMS Servo 
#define dms_feedback_min 3076 //30 degrees 
#define dms_feedback_max 1612 //150 degrees 

//Define Motor Slot Number on PCA9685 (Servo Shield)
const int rff = 15; // right front foot 
const int lfs = 11; //left front shoulder
const int rfs = 10; //right front shoulder 
const int lff = 14; //left front foot
const int rhf = 1; //right hind foot
const int rhs = 5; //right hind shoulder 
const int lhs = 4; //left hind shoulder 
const int lhf = 0; //left hind foot
const int lfa = 13; //Left front Wrist Angle 
const int rfa = 12; //Right Front Wrist Angle 
const int lha = 3; //Left hind Wrist angle 
const int rha = 2; //Right hind Wrist Angle
const int lfsa = 9; // left front shoulder abduction
const int rfsa = 8; // right front shoulder abduction
const int lhsa = 6; // left hind shoulder abduction
const int rhsa = 7; // right hind shoulder abduction

// Define GPIOs on ESP
Servo hsp;
Servo fsp;
const int f_s = 27; //front spine 
const int h_s = 25; //hind spine 

//Define Feedback pins for Servos 
const int f_s_feed = GPIO_NUM_34; //front spine 
const int h_s_feed = GPIO_NUM_35; //hind spine 
const int lfs_feed = GPIO_NUM_34; //left front shoulder !!!!!!!!!!!!!CHANGE
const int rfs_feed = GPIO_NUM_32; //right front shoulder
const int rfa_feed = GPIO_NUM_33; //Right Front Wrist Angle  
// const int lfa_feed = GPIO_NUM_33; //Left front Wrist Angle
const int rff_feed = GPIO_NUM_39; 
const int lhf_feed = GPIO_NUM_36;

//Input Variables for motion --> adjustable in web interface  
int shoulder_angle = 0; // shoulder angle that fits onto the diameter
int rom_spine = 10; //range of motion of spine
int rom_limb = 10; //range of motion of legs 
int rom_feet = 75; //range of motion of feet 
int rom_shoulders = 10; // range of motion of shoulders
int dynamic = 1; //defines which dynamic to use (1 = sigmoid, 2 = sinusoid) 
int gait = 0;  // defines which gait to use (0 = no gait, 1 = regular gait, 2 turn gait (NOT YET IMPLEMENTED)) 
int speed_val = 15; //(15 Standard) Speed of leg and spine (changes the delay between each increment in millis)
int speed_val_foot = 15; // defines the speed for lifting the feet (changes the delay between each increment in millis)
int number_of_steps = 11; //number of full steps to be taken
int foot_center = 0; //defines the offset of the angle of the feet to the vertical to the ground axis
int front_leg_center = 0; //defines the center where the middle of the range of motion lies (frontlegs)
int hind_leg_center = 0; //defines the center where the middle of the range of motion lies (backlegs)
int front_wrist_angle = 0; //defines the orientation of the front foot( + for positive angle from axis, - for negative angle from axis) 
int hind_wrist_angle = 0; //defines the orientation of the hind foot ( + for positive angle from axis, - for negative angle from axis) 
int dynamic_wrist_angle = 1; //defines if dynamically counteracting Wrist angle is used (0 = not used, 1 = counteracting by angle,  2 = counteracting by reading analog pins (NOT YET IMPLEMENTED)) 
int wrist_offset = 0; // set offset for wrists (in case of sag or other)

//Input Variables for motion (not adjustable in web interface) 
int resolution_dynamic_functions = 50; //increments for the dynamic functions 
int rom_wrist_angle = rom_spine + rom_limb; 

//information variables  
int climb_incline = 90;
String claw_angle = "10", surface = "coarse_carpet", toe_angle = "0", robot = "X5_Mini";
float leg_length = 72.5, spine_length = 169.5, spine_middle = 69.5, should_offset = 45; 
float spine_front = (spine_length - spine_middle) / 2;

//Variables for Data Collection 
float elapsed_time[11] =  {0.0}; //time of step in s 
int stride[11] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
int distance[11] = {0}; //sum covered distance in mm
// accelerometer/gyro
float peak_acc_xaxis[11] = {0.0};
float peak_acc_yaxis[11] = {0.0};
float peak_acc_zaxis[11] = {0.0};
float angle_yaxis[11] = {0.0}; // acc at end of stride ---> for calculation of angle
float angle_zaxis[11] = {0.0}; // acc at end of stride ---> for calculation of angle
float xaxis_during_stride[10] = {0.0};
float yaxis_during_stride[10] = {0.0};
float zaxis_during_stride[10] = {0.0};
float xaxis_sum, yaxis_sum, zaxis_sum;
//current measurements
float mean_current[11] = {0.0}; //average current consumption per stride in A
float max_current [11] = {0.0}; //displays maximal current spike in a stride in A 
float current_during_stride [10] = {0.0}; //stores current values of a stride, size displays number of measurements per stride
float curr_sum;
int curr_index = 0; 
//Servo Success in % e.g. did Servo reach the desired position 
int lff_suc[11] = {0}; // left front foot 
int lfs_suc[11] = {0}; // left front shoulder
int rfs_suc[11] = {0}; // right front shoulder 
int rff_suc[11] = {0}; // right front foot
int f_s_suc[11] = {0}; // front spine 
int h_s_suc[11] = {0}; // hind spine 
int rhf_suc[11] = {0}; // right hind foot
int rhs_suc[11] = {0}; // right hind shoulder 
int lhs_suc[11] = {0}; // left hind shoulder 
int lhf_suc[11] = {0}; // left hind foot
int lfa_suc[11] = {0}; // Left front Wrist Angle 
int rfa_suc[11] = {0}; // right Front Wrist Angle 
int lha_suc[11] = {0}; // left hind Wrist angle 
int rha_suc[11] = {0}; // right hind Wrist Angle
int lfsa_suc[11] = {0}; // left front shoulder abduction
int rfsa_suc[11] = {0}; // right front shoulder abudction
int lhsa_suc[11] = {0}; // left hind shoulder abduction
int rhsa_suc[11] = {0}; // right hind shoulder abudction

bool interrupt_mid_stride = false; 

//function prototypes 
void move_motor(int motor_num, int angle); // move Servo certain angle 
void home_pos(); // returns all Servos to home position  
int dynamic_movement_spine(int increment);  // dynamic function for spine 
int dynamic_movement_legs(int increment); // dynamic function for legs
int dynamic_movement_feet(int increment);
int dynamic_movement_shoulders(int increment); // dynamic function for abduction/adduction
int dynamic_movement_wrist(int increment);
void gait1();
int sensor_to_angle(int motor_num, int feedback_pin); // convert sensor reading to angle 

//Home Positions of Servos (hh offset home position, h = position after change via webpage)
static int hh_lff = 86; //Front left foot
int h_lff = hh_lff; 
static int hh_lfs = 108; //Front left shoulder 
int h_lfs = hh_lfs; 
static int hh_rfs = 96; //Front right shoulder
int h_rfs = hh_rfs; 
static int hh_rff = 80; //Front right foot
int h_rff = hh_rff; 
static int h_fs = 126; //Spine Front change to 90
static int h_hs = 116; //Spine Hind change to 90
static int hh_rhf = 75; //Hind right foot
int h_rhf = hh_rhf; 
static int hh_rhs = 76; //Hind right shoulder
int h_rhs = hh_rhs; 
static int hh_lhs = 113;  //Hind left shoulder
int h_lhs = hh_lhs; 
static int hh_lhf = 82; //Hind left foot
int h_lhf = hh_lhf; 
static int hh_lfa = 80; //Front Left Wrist Angle ---
int h_lfa = hh_lfa; 
static int hh_rfa = 95; //Front Right Wrist Angle 
int h_rfa = hh_rfa; 
static int hh_lha = 83; //Hind left Wrist Angle
int h_lha = hh_lha; 
static int hh_rha = 66; //Hind right Wrist Angle
int h_rha = hh_rha;
static int hh_lfsa = 90; // left front shoulder abduction
int h_lfsa = hh_lfsa;
static int hh_rfsa = 113; // right front shoulder abduction
int h_rfsa = hh_rfsa;
static int hh_lhsa = 88; // left hind shoulder abduction
int h_lhsa = hh_lhsa;
static int hh_rhsa = 115; // right hind shoulder abudction
int h_rhsa = hh_rhsa;

void gait1()
{ // gait for regular forward movement
    // reset all arrays
    for (size_t i = 0; i <= 10; i++)
    {
        elapsed_time[i] = 0.0;
        distance[i] = 0;
        peak_acc_xaxis[i] = 0.0;
        peak_acc_yaxis[i] = 0.0;
        peak_acc_zaxis[i] = 0.0;
        angle_yaxis[i] = 0.0;
        angle_zaxis[i] = 0.0;
        mean_current[i] = 0.0;
        max_current[i] = 0.0;
        // succ
    }
    for (size_t i = 0; i <= 9; i++)
    {
        xaxis_during_stride[i] = 0.0;
        yaxis_during_stride[i] = 0.0;
        zaxis_during_stride[i] = 0.0;
        current_during_stride[i] = 0.0;
    }
    // calibration vals for accelerometer angle calc --> offset not being used currently
    //   y_ang_offset = get_accel(4);
    //   z_ang_offset = get_accel(5);
    //   x_acc_offset = get_accel(1);
    //   y_acc_offset = get_accel(2);
    //   z_acc_offset = get_accel(3);

    //phi wrist correction
    // float phi = 10; 
    float phi = (((rom_limb + rom_spine) / 2) + 10)/2; 
   // Serial.println(phi);



        // intial  (half) right step starting from home position
/*     for (int i = 0; i < resolution_dynamic_functions; i++)
    {
        move_motor(lff, h_lff + dynamic_movement_feet(i)); // lift two across feet
        move_motor(rhf, h_rhf - dynamic_movement_feet(i));
        delay(speed_val_foot);
    } */

    /* for (int i = 0; i < resolution_dynamic_functions; i++)
    {
        move_motor(lfsa, h_lfsa - dynamic_movement_shoulders(i)); // lift shoulders
        move_motor(rhsa, h_rhsa - dynamic_movement_shoulders(i));
        delay(speed_val);
    } */

    for (int i = 0; i < resolution_dynamic_functions; i++)
    {
        move_motor(lff, h_lff + dynamic_movement_feet(i)); // lift two across feet
        move_motor(rhf, h_rhf - dynamic_movement_feet(i));

        move_motor(f_s, h_fs + (0.5 * dynamic_movement_spine(i))); // bend body via spine
        move_motor(h_s, h_hs - (0.5 * dynamic_movement_spine(i)));

        move_motor(lfs, h_lfs - (0.5 * dynamic_movement_legs(i))); // move front shoulders
        move_motor(rfs, h_rfs - (0.5 * dynamic_movement_legs(i)));

        move_motor(rhs, h_rhs + (0.5 * dynamic_movement_legs(i))); // move back shoulders
        move_motor(lhs, h_lhs + (0.5 * dynamic_movement_legs(i)));

        move_motor(lfsa, h_lfsa - dynamic_movement_shoulders(i)); // lift shoulders
        move_motor(rhsa, h_rhsa - dynamic_movement_shoulders(i));

        if (dynamic_wrist_angle == 1)
        {
            // Serial.println((-roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2)))); 
            move_motor(lfa, h_lfa + (0.5 * dynamic_movement_legs(i)) + (0.5 * dynamic_movement_spine(i)) - 0.5 * dynamic_movement_wrist(i)); // -rfl- roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); // left front      rotate wrists
            move_motor(rfa, h_rfa + (0.5 * dynamic_movement_legs(i)) + (0.5 * dynamic_movement_spine(i)) - 0.5 * dynamic_movement_wrist(i)); // -rfr- roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); // right front
            move_motor(lha, h_lha - (0.5 * dynamic_movement_legs(i)) - (0.5 * dynamic_movement_spine(i)) + 0.5 * dynamic_movement_wrist(i)); // +rhl+ roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); // left hind
            move_motor(rha, h_rha - (0.5 * dynamic_movement_legs(i)) - (0.5 * dynamic_movement_spine(i)) + 0.5 * dynamic_movement_wrist(i)); // +rhr+ roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); // right hind
            //Serial.println((0.5 * dynamic_movement_legs(i)) + (0.5 * dynamic_movement_spine(i)));
            //Serial.println(+ (-0.5 * dynamic_movement_legs(i)) - (0.5 * dynamic_movement_spine(i)));
        }
        delay(speed_val);
    }

    for (int i = 0; i < resolution_dynamic_functions; i++)
    {
        move_motor(lfsa, (h_lfsa - rom_shoulders) + dynamic_movement_shoulders(i)); // drop shoulders
        move_motor(rhsa, (h_rhsa - rom_shoulders) + dynamic_movement_shoulders(i));
        delay(speed_val);
    }

    for (int i = 0; i < resolution_dynamic_functions; i++)
    {
        move_motor(lff, (h_lff + rom_feet) - dynamic_movement_feet(i)); // put feet down
        move_motor(rhf, (h_rhf - rom_feet) + dynamic_movement_feet(i));
        delay(speed_val_foot);
    }


    // Serial.println("Done with half right step");
    // delay(3000); 

    //------------------------------------------------------------------------------------------------------------ Real Gait

    float start_time = millis() / 1000.0;
    int first_dist = get_dist();

    //   //Correcting the Wrist angle (tilt angle in between steps (psi))
    //   double alpha = (PI/180) * 0.5 * rom_spine;
    //   double beta = (PI/180) * 0.5 * rom_limb;
    //   double delta = atan((2*cos(alpha + beta)* leg_length + 2*cos(alpha)*should_offset)/(2*cos(alpha)*spine_front + spine_middle));
    //   double psi_delta = atan((leg_length + should_offset)/(spine_front + 0.5* spine_middle));
    //   double psi_max = psi_delta - delta;
    //   psi_max = psi_max *(180 / PI);

    for (step_val; step_val <= (number_of_steps - 1); step_val++) //
    {
        // left step 1-11 -------------------------------------------------------------------- LEFT

        //Serial.println("Starting with left step");

 /*        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lhf, h_lhf + dynamic_movement_feet(i)); // lift two across feet
            move_motor(rff, h_rff - dynamic_movement_feet(i));
            delay(speed_val_foot);
        } */

        /* for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(rfsa, h_rfsa + dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(lhsa, h_lhsa + dynamic_movement_shoulders(i));
            delay(speed_val);
        } */

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lhf, h_lhf + dynamic_movement_feet(i)); // lift two across feet
            move_motor(rff, h_rff - dynamic_movement_feet(i));

            move_motor(f_s, (h_fs + rom_spine) - dynamic_movement_spine(i)); // bend body
            move_motor(h_s, (h_hs - rom_spine) + dynamic_movement_spine(i));

            move_motor(lfs, (h_lfs - rom_limb) + dynamic_movement_legs(i)); // move frontlegs
            move_motor(rfs, (h_rfs - rom_limb) + dynamic_movement_legs(i));

            move_motor(rhs, (h_rhs + rom_limb) - dynamic_movement_legs(i)); // move backlegs
            move_motor(lhs, (h_lhs + rom_limb) - dynamic_movement_legs(i));

            move_motor(rfsa, h_rfsa + dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(lhsa, h_lhsa + dynamic_movement_shoulders(i));

            if (dynamic_wrist_angle == 1)
            {
                move_motor(lfa, h_lfa + rom_spine + rom_limb - wrist_offset - dynamic_movement_spine(i) - dynamic_movement_legs(i) + dynamic_movement_wrist(i)); //+lfl+ 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); // rotate wrists
                move_motor(rfa, h_rfa + rom_spine + rom_limb - wrist_offset - dynamic_movement_spine(i) - dynamic_movement_legs(i) + dynamic_movement_wrist(i)); //+lfr+ 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //
                move_motor(lha, h_lha - rom_spine - rom_limb + wrist_offset + dynamic_movement_spine(i) + dynamic_movement_legs(i) - dynamic_movement_wrist(i)); //-lhl- 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //
                move_motor(rha, h_rha - rom_spine - rom_limb + wrist_offset + dynamic_movement_spine(i) + dynamic_movement_legs(i) - dynamic_movement_wrist(i)); //-lhr- 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //
                //Serial.println(-rom_spine - rom_limb + dynamic_movement_spine(i) + dynamic_movement_legs(i));
                //Serial.println(rom_spine + rom_limb - dynamic_movement_spine(i) - dynamic_movement_legs(i));
            }

            if (i == 9 || i == 18 || i == 27 || i == 36 || i == 45) // get current/accel equally spaced throughout 50 increments
            {
                current_during_stride[curr_index] = get_current(); // index 0-4

                // Serial.println("Current during stride: ");
                // Serial.println(current_during_stride[curr_index]);

                // gyro
                xaxis_during_stride[curr_index] = get_accel(1);
                yaxis_during_stride[curr_index] = get_accel(2);
                zaxis_during_stride[curr_index] = get_accel(3);

                curr_index++; // increment the current index
            }
            delay(speed_val);
        }

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(rfsa, (h_rfsa + rom_shoulders) - dynamic_movement_shoulders(i)); // drop shoulders
            move_motor(lhsa, (h_lhsa + rom_shoulders) - dynamic_movement_shoulders(i));
            delay(speed_val);
        }

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lhf, (h_lhf + rom_feet) - dynamic_movement_feet(i)); // drop feet
            move_motor(rff, (h_rff - rom_feet) + dynamic_movement_feet(i));
            delay(speed_val_foot);
        }

        //Serial.println("Done with left step");
        //delay(3000);

        int measured_dist;
        measured_dist = sensor.readRangeSingleMillimeters();

        get_dist(); // to detect if end of track/fall mid stride
        if (interrupt == true)
        {
            Serial.println("Gait interrupted mid stride");
            Serial.println(measured_dist);
            step_val = 200;
            interrupt_mid_stride = true;
            break;
        } 

        // Success (currently only measured for the left step and not for the feet (not enough pins))
        //  float fs_diff_angle =  ((h_fs + rom_spine) - rom_spine) - abs(((h_fs + rom_spine) - rom_spine) - sensor_to_angle(f_s, f_s_feed));
        //  f_s_suc[step_val] = roundf(100 * fs_diff_angle / ((h_fs + rom_spine) - rom_spine));

        // float hs_diff_angle = ((h_hs - rom_spine) + rom_spine) - abs(((h_hs - rom_spine) + rom_spine) - sensor_to_angle(h_s, h_s_feed));
        // h_s_suc[step_val] = roundf(100 * hs_diff_angle / ((h_hs - rom_spine) + rom_spine));

        // float lfs_diff_angle = ((h_lfs + rom_limb) - rom_limb) - abs(((h_lfs + rom_limb) - rom_limb) - sensor_to_angle(lfs, lfs_feed));
        // lfs_suc[step_val] = roundf(100 * lfs_diff_angle / ((h_lfs + rom_limb) - rom_limb));

        // float rfs_diff_angle = ((h_rfs + rom_limb) - rom_limb) - abs(((h_rfs + rom_limb) - rom_limb) - sensor_to_angle(rfs, rfs_feed));
        // rfs_suc[step_val] = roundf(100 * rfs_diff_angle / ((h_rfs + rom_limb) - rom_limb));

        // float rff_diff_angle = ((h_lhf + rom_feet) - rom_feet) - abs(((h_lhf + rom_feet) - rom_feet) - sensor_to_angle(rff, rff_feed));
        // rff_suc[step_val] = roundf(100 * rff_diff_angle / ((h_lhf + rom_feet) - rom_feet));

        // float lhf_diff_angle = ((h_lhf + rom_feet) - rom_feet) - abs(((h_lhf + rom_feet) - rom_feet) - sensor_to_angle(lhf, lhf_feed));
        // lhf_suc[step_val] = roundf(100 * lhf_diff_angle / ((h_lhf + rom_feet) - rom_feet));

        // wrists --> include psi angle !!!
        //  float rfa_diff_angle = h_rfa - abs(h_rfa - sensor_to_angle(rfa, rfa_feed));
        //  rfa_suc[step_val] = roundf(100 * rfa_diff_angle / h_rfa);

        // float lfa_diff_angle = h_lfa - abs(h_lfa - sensor_to_angle(lfa, lfa_feed));
        // lfa_suc[step_val] = roundf(100 * lfa_diff_angle / h_lfa);

        // right step 1-11 --------------------------------------------------------------------- RIGHT

        //Serial.println("Starting with right step");
/*         for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lff, h_lff + dynamic_movement_feet(i)); // lift feet
            move_motor(rhf, h_rhf - dynamic_movement_feet(i));
            delay(speed_val_foot);
        } */

        /* for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lfsa, h_lfsa - dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(rhsa, h_rhsa - dynamic_movement_shoulders(i));
            delay(speed_val);
        } */

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lff, h_lff + dynamic_movement_feet(i)); // lift feet
            move_motor(rhf, h_rhf - dynamic_movement_feet(i));

            move_motor(f_s, (h_fs - rom_spine) + dynamic_movement_spine(i)); // bend body
            move_motor(h_s, (h_hs + rom_spine) - dynamic_movement_spine(i));

            move_motor(lfs, (h_lfs + rom_limb) - dynamic_movement_legs(i)); // move frontlegs
            move_motor(rfs, (h_rfs + rom_limb) - dynamic_movement_legs(i));

            move_motor(rhs, (h_rhs - rom_limb) + dynamic_movement_legs(i)); // move backlegs
            move_motor(lhs, (h_lhs - rom_limb) + dynamic_movement_legs(i));

            move_motor(lfsa, h_lfsa - dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(rhsa, h_rhsa - dynamic_movement_shoulders(i));

            if (dynamic_wrist_angle == 1)
            {
                move_motor(lfa, h_lfa - rom_spine - rom_limb + wrist_offset + dynamic_movement_spine(i) + dynamic_movement_legs(i) - dynamic_movement_wrist(i)); //-rfl- 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //- (psi_max - abs(psi_max - i * (psi_max / 200)))); // rotate wrists
                move_motor(rfa, h_rfa - rom_spine - rom_limb + wrist_offset + dynamic_movement_spine(i) + dynamic_movement_legs(i) - dynamic_movement_wrist(i)); //-rfr- 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //- (psi_max - abs(psi_max - i * (psi_max / 200))));
                move_motor(lha, h_lha + rom_spine + rom_limb - wrist_offset - dynamic_movement_spine(i) - dynamic_movement_legs(i) + dynamic_movement_wrist(i)); //+rhl+ 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //- (psi_max - abs(psi_max - i * (psi_max / 200))));
                move_motor(rha, h_rha + rom_spine + rom_limb - wrist_offset - dynamic_movement_spine(i) - dynamic_movement_legs(i) + dynamic_movement_wrist(i)); //+rhr+ 2 * roundf((phi / 2) * cos((PI / resolution_dynamic_functions) * i + PI) + (phi / 2))); //- (psi_max - abs(psi_max - i * (psi_max / 200))));
                //Serial.println(- rom_spine - rom_limb + dynamic_movement_spine(i) + dynamic_movement_legs(i));
                //Serial.println(+ rom_spine + rom_limb - dynamic_movement_spine(i) - dynamic_movement_legs(i));
                //Serial.println(- rom_limb - rom_spine + dynamic_movement_legs(i) + dynamic_movement_spine(i));
                //Serial.println(+ rom_limb + rom_spine - dynamic_movement_legs(i) - dynamic_movement_spine(i));
            }

            if (i == 9 || i == 18 || i == 27 || i == 36 || i == 45)
            {

                current_during_stride[curr_index] = get_current(); // index 5 - 9

                // gyro
                xaxis_during_stride[curr_index] = get_accel(1);
                yaxis_during_stride[curr_index] = get_accel(2);
                zaxis_during_stride[curr_index] = get_accel(3);

                curr_index++; // increment the current index
            }
            delay(speed_val);
        }

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lfsa, (h_lfsa - rom_shoulders) + dynamic_movement_shoulders(i)); // drop shoulders
            move_motor(rhsa, (h_rhsa - rom_shoulders) + dynamic_movement_shoulders(i)); 
            delay(speed_val);
        }

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lff, (h_lff + rom_feet) - dynamic_movement_feet(i)); // drop feet
            move_motor(rhf, (h_rhf - rom_feet) + dynamic_movement_feet(i));
            delay(speed_val_foot);
        }

        // Serial.println("Done with right step");
        // delay(3000);

        // read Sensors/get data
        //------------------------------------------------------------------

        elapsed_time[step_val] = (millis() / 1000.0) - start_time;

        // //find max and calc mean current
        max_current[step_val] = current_during_stride[0];

        for (size_t i = 0; i < (sizeof(current_during_stride) / sizeof(current_during_stride[0])); i++) // iterate over array from index 0-9
        {
            // Serial.println("Current array: ");
            // Serial.println(current_during_stride[i]);
            if (max_current[step_val] < current_during_stride[i])
            {
                max_current[step_val] = current_during_stride[i];
            }
        }
        // Serial.println("max current");
        // Serial.println(max_current[step_val]);

        curr_sum = 0.0;
        for (size_t i = 0; i < (sizeof(current_during_stride) / sizeof(current_during_stride[0])); i++)
        {
            curr_sum += current_during_stride[i];
        }

        mean_current[step_val] = curr_sum / (sizeof(current_during_stride) / sizeof(current_during_stride[0]));
        curr_index = 0;

        // accelerometer
        // find max acceleration for each axis
        peak_acc_xaxis[step_val] = xaxis_during_stride[0];
        peak_acc_yaxis[step_val] = yaxis_during_stride[0];
        peak_acc_zaxis[step_val] = zaxis_during_stride[0];

        for (size_t i = 0; i < (sizeof(xaxis_during_stride) / sizeof(xaxis_during_stride[0])); i++)
        {

            // Serial.println("z during stride: ");
            // Serial.println(zaxis_during_stride[i] - z_acc_offset);
            // check if we have a new peak value for x-axis
            if (peak_acc_xaxis[step_val] < (xaxis_during_stride[i]))
            {
                peak_acc_xaxis[step_val] = (xaxis_during_stride[i]);
                // peak_acc_xaxis[step_val] = (xaxis_during_stride[i] - x_acc_offset);
            }

            // check if we have a new peak value for y-axis
            if (peak_acc_yaxis[step_val] < (yaxis_during_stride[i]))
            {
                peak_acc_yaxis[step_val] = (yaxis_during_stride[i]);
                // peak_acc_yaxis[step_val] = (yaxis_during_stride[i] - y_acc_offset);
            }

            // check if we have a new peak value for z-axis
            if (peak_acc_zaxis[step_val] < (zaxis_during_stride[i]))
            {
                peak_acc_zaxis[step_val] = (zaxis_during_stride[i]);
                // peak_acc_zaxis[step_val] = (zaxis_during_stride[i] - z_acc_offset);
            }
        }

        // Serial.println("Peak acc vals (xyz): ");
        // Serial.println(peak_acc_xaxis[step_val]);
        // Serial.println(peak_acc_yaxis[step_val]);
        // Serial.println(peak_acc_zaxis[step_val]);

        // calc pitch, roll
        angle_yaxis[step_val] = get_accel(4); // heading
        angle_zaxis[step_val] = get_accel(5); // topple

        // Serial.println(angle_yaxis[step_val]);
        // Serial.println(angle_zaxis[step_val]);
        // server.handleClient(); //needed if live table used ?!

        // Calculate actual distance covered
        int raw_dist;
        int temp_dist;

         if (step_val == 0)
        {
            raw_dist = get_dist();
            if (interrupt == true)
            {
                distance[step_val] = first_dist - raw_dist;
                step_val = 200;
                break;
            }
            else
            {
                distance[step_val] = first_dist - raw_dist;
            }
        }

        if (step_val > 0 && step_val < 199)
        {
            temp_dist = get_dist();
            if (interrupt == true)
            {
                distance[step_val] = raw_dist - temp_dist;
                step_val = 200;
                break;
            }
            else
            {
                distance[step_val] = raw_dist - temp_dist;
                raw_dist = temp_dist;
            }
        }
    } 

    // Serial.println(interrupt_mid_stride);

    if (interrupt_mid_stride == true)
    {
        // final half right step to go to home
        //  Serial.println("final half right step to go to home ");
/*         for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lff, h_lff + dynamic_movement_feet(i)); // lift two across feet
            move_motor(rhf, h_rhf - dynamic_movement_feet(i));
            delay(speed_val_foot);
        } */

        /* for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lfsa, h_lfsa - dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(rhsa, h_rhsa - dynamic_movement_shoulders(i));
            delay(speed_val);
        } */

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lff, h_lff + dynamic_movement_feet(i)); // lift two across feet
            move_motor(rhf, h_rhf - dynamic_movement_feet(i));

            move_motor(f_s, (h_fs - rom_spine) + 0.5 * dynamic_movement_spine(i)); // bend body
            move_motor(h_s, (h_hs + rom_spine) - 0.5 * dynamic_movement_spine(i));

            move_motor(lfs, (h_lfs + rom_limb) - 0.5 * dynamic_movement_legs(i)); // move frontlegs
            move_motor(rfs, (h_rfs + rom_limb) - 0.5 * dynamic_movement_legs(i));

            move_motor(rhs, (h_rhs - rom_limb) + 0.5 * dynamic_movement_legs(i)); // move backlegs
            move_motor(lhs, (h_lhs - rom_limb) + 0.5 * dynamic_movement_legs(i));

            move_motor(lfsa, h_lfsa - dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(rhsa, h_rhsa - dynamic_movement_shoulders(i));

            if (dynamic_wrist_angle == 1)
            {
                move_motor(lfa, h_lfa - rom_spine - rom_limb + wrist_offset + 0.5 * dynamic_movement_spine(i) + 0.5 * dynamic_movement_legs(i) - 0.5 * dynamic_movement_wrist(i)); // rotate wrists
                move_motor(rfa, h_rfa - rom_spine - rom_limb + wrist_offset + 0.5 * dynamic_movement_spine(i) + 0.5 * dynamic_movement_legs(i) - 0.5 * dynamic_movement_wrist(i));
                move_motor(lha, h_lha + rom_spine + rom_limb - wrist_offset - 0.5 * dynamic_movement_spine(i) - 0.5 * dynamic_movement_legs(i) + 0.5 * dynamic_movement_wrist(i));
                move_motor(rha, h_rha + rom_spine + rom_limb - wrist_offset - 0.5 * dynamic_movement_spine(i) - 0.5 * dynamic_movement_legs(i) + 0.5 * dynamic_movement_wrist(i));
            }
            delay(speed_val);
        }
        
        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lfsa, (h_lfsa - rom_shoulders) + dynamic_movement_shoulders(i)); // drop shoulders
            move_motor(rhsa, (h_rhsa - rom_shoulders) + dynamic_movement_shoulders(i)); 
            delay(speed_val);
        }

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lff, (h_lff + rom_feet) - dynamic_movement_feet(i)); // put feet down
            move_motor(rhf, (h_rhf - rom_feet) + dynamic_movement_feet(i));
            delay(speed_val_foot);
        }

        // Serial.println("Done with half right step");
    }

    else
    {
        // final half left step to get into home position
        // Serial.println("final half left step to go to home ");
/*         for (int i = 0; i <= resolution_dynamic_functions; i++)
        {
            move_motor(lhf, h_lhf + dynamic_movement_feet(i)); // lift two across feet
            move_motor(rff, h_rff - dynamic_movement_feet(i));
            delay(speed_val_foot);
        } */

        /* for (int i = 0; i <= resolution_dynamic_functions; i++)
        {
            move_motor(rfsa, h_rfsa + dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(lhsa, h_lhsa + dynamic_movement_shoulders(i));
            delay(speed_val);
        } */

        for (int i = 0; i <= resolution_dynamic_functions; i++)
        {
            move_motor(lhf, h_lhf + dynamic_movement_feet(i)); // lift two across feet
            move_motor(rff, h_rff - dynamic_movement_feet(i));

            move_motor(f_s, (h_fs + rom_spine) - (0.5 * dynamic_movement_spine(i))); // bend body via spine
            move_motor(h_s, (h_hs - rom_spine) + (0.5 * dynamic_movement_spine(i)));

            move_motor(lfs, (h_lfs - rom_limb) + (0.5 * dynamic_movement_legs(i))); // move front shoulder
            move_motor(rfs, (h_rfs - rom_limb) + (0.5 * dynamic_movement_legs(i)));

            move_motor(rhs, (h_rhs + rom_limb) - (0.5 * dynamic_movement_legs(i))); // move back shoulder
            move_motor(lhs, (h_lhs + rom_limb) - (0.5 * dynamic_movement_legs(i)));

            move_motor(rfsa, h_rfsa + dynamic_movement_shoulders(i)); // lift shoulders
            move_motor(lhsa, h_lhsa + dynamic_movement_shoulders(i));

            if (dynamic_wrist_angle == 1)
            {
                move_motor(lfa, h_lfa + rom_spine + rom_limb - wrist_offset - 0.5 * dynamic_movement_legs(i) - 0.5 * dynamic_movement_spine(i) + 0.5 * dynamic_movement_wrist(i)); // rotate wrists
                move_motor(rfa, h_rfa + rom_spine + rom_limb - wrist_offset - 0.5 * dynamic_movement_legs(i) - 0.5 * dynamic_movement_spine(i) + 0.5 * dynamic_movement_wrist(i));
                move_motor(lha, h_lha - rom_spine - rom_limb + wrist_offset + 0.5 * dynamic_movement_legs(i) + 0.5 * dynamic_movement_spine(i) - 0.5 * dynamic_movement_wrist(i));
                move_motor(rha, h_rha - rom_spine - rom_limb + wrist_offset + 0.5 * dynamic_movement_legs(i) + 0.5 * dynamic_movement_spine(i) - 0.5 * dynamic_movement_wrist(i));
            }
            delay(speed_val);
        }

        for (int i = 0; i <= resolution_dynamic_functions; i++)
        {
            move_motor(rfsa, (h_rfsa + rom_shoulders) - dynamic_movement_shoulders(i)); // drop shoulders
            move_motor(lhsa, (h_lhsa + rom_shoulders) - dynamic_movement_shoulders(i));
            delay(speed_val);
        }

        for (int i = 0; i < resolution_dynamic_functions; i++)
        {
            move_motor(lhf, (h_lhf + rom_feet) - dynamic_movement_feet(i)); // drop feet
            move_motor(rff, (h_rff - rom_feet) + dynamic_movement_feet(i));
            delay(speed_val_foot);
        }

        // Serial.println("Done with final half step");
    }

    delay(500);
    Serial.println("All Steps done");
    step_val = 0;
    gait = 0; // resets gait variable
    interrupt = false;
    interrupt_mid_stride = false;
    home_pos();
    delay(2000);
    home_pos();
}

void move_motor(int motor_num, int angle){
    
  int pulsewidth; 

// Feet
  if (motor_num == 0 || motor_num == 1 || motor_num == 14 || motor_num == 15){
    pulsewidth = map(angle, 30, 150, datan_servo_min, datan_servo_max); 
    pwm.setPWM(motor_num, 0, pulsewidth); 
    // Serial.print("Servo Num Datan: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
  }

  // Wrist
  else if (motor_num == 2 || motor_num == 3 || motor_num == 12 || motor_num == 13){
    pulsewidth = map(angle, 0, 150, dms_servo_min, dms_servo_max); 
    pwm.setPWM(motor_num, 0, pulsewidth); 
    // Serial.print("Servo Num DMS: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle);
  }

  // Shoulders
  else if( 4 <= motor_num && motor_num <= 11){
    pulsewidth = map(angle, 30, 150, savoex_servo_min, savoex_servo_max);
    pwm.setPWM(motor_num, 0, pulsewidth);
    // Serial.print("Servo Num Savoex: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
  }

    // Spine
else if (motor_num == 25){
    pulsewidth = map(angle, 30, 150, savoex_spine_min, savoex_spine_max);
    hsp.writeMicroseconds(pulsewidth);
    // Serial.print("Spine angle: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
    // Serial.print("spine pulsewidth: "); 
    // Serial.println(pulsewidth); 
}

else {
    pulsewidth = map(angle, 30, 150, savoex_spine_min, savoex_spine_max);
    fsp.writeMicroseconds(pulsewidth);
    // Serial.print("Servo Num Savoex: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
}
}

void home_pos(){
move_motor(lff, h_lff); //moves all feet to the ground
move_motor(rff, h_rff);
move_motor(lhf, h_lhf);
move_motor(rhf, h_rhf);
  delay(500);
//moves all shoulders into a right angle to the body
move_motor(lfs, h_lfs);
move_motor(rfs, h_rfs);
move_motor(lhs, h_lhs);
move_motor(rhs, h_rhs);
  delay(500);
// shoulder abudction joints
move_motor(lfsa, h_lfsa);
move_motor(rfsa, h_rfsa);
move_motor(lhsa, h_lhsa);
move_motor(rhsa, h_rhsa);
  delay(500);
move_motor(f_s, h_fs); //moves the spine in the right angle to the body 
move_motor(h_s, h_hs);
  delay(500);
move_motor(lfa, h_lfa); //moves the Wrist Angles into the right angle to the body 
move_motor(rfa, h_rfa);
move_motor(lha, h_lha);
move_motor(rha, h_rha);
  delay(500);

Serial.println("Robot in home pos");
}

int dynamic_movement_spine(int increment){
  int motor_angle; 
  if (dynamic = 1)
  {
    motor_angle = roundf(2 * rom_spine / (1 + exp(-0.2 * increment + 5)));
  }
  if (dynamic == 2){
    motor_angle = roundf(rom_spine * cos((PI/resolution_dynamic_functions) * increment + PI) + rom_spine); 
  }
  return motor_angle; 

}

int dynamic_movement_legs(int increment){ 
  int motor_angle; 

  if (dynamic ==  1) {
    motor_angle = roundf(2 * rom_limb / (1 + exp(-0.2 * increment + 5)));
  }
  if (dynamic == 2){
    motor_angle = roundf(rom_limb * cos((PI/resolution_dynamic_functions) * increment + PI) + rom_limb); 
  }
  return motor_angle; 
}

int dynamic_movement_feet(int increment){
  int motor_angle; 
  if (dynamic == 1){
    motor_angle = roundf(rom_feet / (1 + exp(-0.2 * increment + 5)));
  }
  if (dynamic == 2){
    motor_angle = roundf(rom_feet * cos((PI/resolution_dynamic_functions) * increment + PI) + rom_feet); 
  }
  return motor_angle; 
}

int dynamic_movement_shoulders(int increment){ 
  int motor_angle; 

  if (dynamic ==  1) {
    motor_angle = roundf(rom_shoulders / (1 + exp(-0.2 * increment + 5)));
  }
  if (dynamic == 2){
    motor_angle = roundf(rom_shoulders * cos((PI/resolution_dynamic_functions) * increment + PI) + rom_shoulders); 
  }
  return motor_angle; 
}

int dynamic_movement_wrist(int increment){ 
  int motor_angle; 

  if (dynamic ==  1) {
    motor_angle = roundf(2 * wrist_offset / (1 + exp(-0.2 * increment + 5)));
  }
  if (dynamic == 2){
    motor_angle = roundf(wrist_offset * cos((PI/resolution_dynamic_functions) * increment + PI) + wrist_offset); 
  }
  return motor_angle; 
}

int sensor_to_angle(int motor_num, int feedback_pin){
  int feedback_val = analogRead(feedback_pin); 
  // Serial.println(feedback_val); 
  int angle; 

// Feet
  if (motor_num == 0 || motor_num == 1 || motor_num == 14 || motor_num == 15){
    angle = map(feedback_val, datan_feedback_min, datan_feedback_max, 30, 150); 
    // Serial.print("Servo Num Datan: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
  }

  // Wrist 
  else if (motor_num == 2 || motor_num == 3 || motor_num == 12 || motor_num == 13){
    angle = map(feedback_val, dms_feedback_min, dms_feedback_max, 0, 270);
    // Serial.print("Servo Num DMS: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle);
  }

// Shoulders
  else if (4 <= motor_num && motor_num <= 11){
    angle = map(feedback_val, savoex_feedback_min, savoex_feedback_max, 30, 150); 
    // Serial.print("Servo Num Savoex: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
  }

// Spine
  else if( motor_num == 25 || motor_num == 27){
    angle = map(feedback_val, savoex_feedback_min, savoex_feedback_max, 30, 150); 
    // Serial.print("Servo Num Savoex: "); 
    // Serial.println(motor_num);
    // Serial.print("Angle: ");   
    // Serial.println(angle); 
  }

  return angle; 
}