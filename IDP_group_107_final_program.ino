//IDP group 107 final code
// main line - this refers to the white line which loops around the whole map (excluding the break inside tunnel)
// on the main line refers to the main line being between the middle line sensors (meaning robot can then follow the main line)
//including relevant motor libraries
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>
//setting which pins correspond to which variables
#define SL_S 0 //line sensor side left pin
#define SR_S 8 //line sensor side right pin
#define R_S 2 // line sensor middle right pin
#define L_S 1 // line sensor middle left pin
#define hall_p 4 // hall sensor pin
#define green_LED 5 // green LED pin 
#define red_LED 6 // red LED pin 
#define amber_LED 7 // amber LED pin 
#define trigPin_front  13 // Ultrasound trig pin
#define echoPin_front  12 // Ultrasound echo pin
#define side_ds A0 // side distance sensor output signal pin
#define push A2 // push button signal pin
int count = 0; // initialising variable used to check which boxes the robot has passed on returning the block
char is_magnet; //initialises the variable used to identify if the robot is magnetic or not
//initialising motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* Motor2 = AFMS.getMotor(2);
using namespace std;

//assignning relevant pins, starting motors and their speeds, begining serial monitor
void setup() {
    // setting up motors and their speeds
    AFMS.begin();
    Motor1->setSpeed(200);//left is 200
    Motor2->setSpeed(187);//right is 187 due to cocked orientation of one of the wheels this was optimal for straight movement
    //defining the line sensors as inputs
    pinMode(R_S, INPUT);
    pinMode(L_S, INPUT);
    pinMode(SL_S, INPUT);
    pinMode(SR_S, INPUT);
    //defining the LEDs as outputs
    pinMode(green_LED, OUTPUT);
    pinMode(red_LED, OUTPUT);
    pinMode(amber_LED, OUTPUT);
    pinMode(side_ds, INPUT); // Setting the distance sensor output signal as input pin
    pinMode(trigPin_front, OUTPUT); // Sets the Ultrasound trig Pin as an Output
    pinMode(echoPin_front, INPUT); // Sets the Ultrasoudn echo Pin as an Input
    Serial.begin(9600); //begins serial monitor
}

//movement functions
void forward() {
    //sets motors running in the forward direction
    //Backward used due to how mechanical team put the motors on the chasis
    Motor1->run(BACKWARD);
    Motor2->run(BACKWARD);
}

void backward() {
    //sets motors running in the backward direction
    //FORWARD used due to how mechanical team put the motors on the chasis
    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
}

void park() {
    //stops the motors
    Motor1->run(RELEASE);
    Motor2->run(RELEASE);
}

void turn(char directionT) {
    // turns the car in a given direction
    // arguments: a - anticlockwise, c - clockwise
    if (directionT == 'a') {
        //anticlockwise turn
        Motor2->run(BACKWARD);
        Motor1->run(FORWARD);
    }
    else if (directionT == 'c') {
        //clockwise turn
        Motor1->run(BACKWARD);
        Motor2->run(FORWARD);
    }
    else {
        //error check for invalid turn input.
        park();
        Serial.println("Value entered to turn is not valid: ");
        Serial.print(directionT);
    }
}

void line_follow() {
    //line follow function used inside loops
    if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 0)) { forward(); }   //if Right Sensor and Left Sensor are at White color then it will call forword function
    if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0)) { park(); turn('c'); delay(60); } //if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
    if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1)) { park(); turn('a'); delay(60); }  //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
}
//end of movement functions 

//funtion to calculate the distance infront of robot using ultrasonic sensor​
int calculateFD() {
    //calculates distance read by the front Ultrasonic distance sensor
    long duration;
    int distance;
    digitalWrite(trigPin_front, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin_front, HIGH);
    delayMicroseconds(10);
    // turns of trig pin
    digitalWrite(trigPin_front, LOW);
    duration = pulseIn(echoPin_front, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance = duration * 0.034 / 2; // converts to cm
    return distance; // returns distance of front of car from objects
}

//function to get the robot on the main line from the starting position​
void startup() {
    //robot travels forward till it touches the starting box
    while (digitalRead(SL_S) == 0) {
        forward();
    }
    //turning to get a better angle relative to the main line. Robot still inside the main box 
    backward(); delay(100); park(); turn('a'); delay(400); park(); // these values were found through testing
    //moving forward till side left sensor reaches the edge of starting box
    while (digitalRead(SL_S) == 0) {
        forward();
        delay(25);
        digitalRead(SL_S);
        park();
        delay(5);
    }
    //robot then turns to get a closer angle to parrallel with main line then moves forward slightly outisde of the starting box
    turn('a'); delay(200); forward(); delay(300); // again optimal values found through testing
    //forward till right sensor detects white line - this line should be main line 
    while (digitalRead(R_S) == 0) {
        forward();
    }
    //again turnign to get closer to parrallel with main line
    backward(); delay(150); park(); turn('a'); delay(150); //values found through testing
    //moves forward till the left middle line sensor is touching a white line.
    while (digitalRead(L_S) == 0) {
        forward();
    }
    // line should now be between middle 2 line sensors - and so it can line follow correctly
    park();
    //robot stops at this point
}

//function to get robot from on main line and between the starting and green boxes to on the main line infront of the tunnel
void past_green_box() {
    //robot will follow the line till the SL_S hits the white line coming out from the green box to the main line
    while (digitalRead(SL_S) == 0) {
        line_follow();
    }
    //robot then moves forward till the SR_S touches a white line (this should be the main line)
    while (digitalRead(SR_S) == 0) {
        forward();
    }
    // robot should now be at curved section infront of tunnel
    // moving back and reangling to be more parrallel with the main line
    park(); backward(); delay(150); turn('c'); delay(100); park(); // optimal values found through testing
    //moving forward till the right middle sensor is touching line and so the main line should be between the middle sensors
    while (digitalRead(R_S) == 0) {
        forward(); delay(30);
        digitalRead(R_S);
        park(); delay(20); //making sure robot doesnt go too fast and miss line
    }
    // robot should now be on the main line and infront of the tunnel
}

//function to get through the tunnel by keeping a set distance form side distance sensor and the wall whilst moving forward
void traverse_tunnel() {
    //initialising distance variables
    //below values determined through testing
    int threshold = 400; // this value means we are outside the tunnel
    int side_upper = 350; // value determining how far we want to go away from the wall
    int side_lower = 310; // value determining how close we want to get to the wall 
    int walldistance = 340; // setting an inital value for the wall distance variable 

    // while the side distance sensor tells us a value lower than threshold this means we are inside the tunnel and so need to loop till outside
    while (walldistance < threshold) {
        walldistance = analogRead(side_ds);
        // we want this to loop while we are inside the tunnel
        Serial.println("Inside the tunnel! ");
        Serial.print(walldistance); // print statements to monitor during unit testing

        //if the distance from the tunnel wall is within a certain range, the car is going forward through the tunnel
        if (side_lower < walldistance < side_upper) 
        { forward(); delay(20); park(); delay(10); }

        //if the distance is too close to the tunnel wall, turn the robot away from the wall and forward slightly
        if (side_lower > walldistance) 
        { park(); turn('a'); delay(10); forward(); delay(5); } 

        //if the distance is too far from the tunnel wall, turn the robot the towards the tunnel wall and forward slightly
        if (walldistance < side_upper) 
        { park(); turn('c'); delay(10); forward(); delay(5); }
    }
    park();
    //robot should now be outisde tunnel
}

//f​unction used to reattach onto the main line when outside the tunnel - either direction 
void reattach_to_main(char direction) {
    if (direction == 'p') //argument p denotes pickup so the robot is coming out tunnel onto the block side of the map
    {
        // we want robot to move forward till the side right sensor detects a line
        while (digitalRead(SR_S) == 1) {
            forward(); delay(30);
            digitalRead(SR_S);
            park(); delay(10); // make sure robot doesn't move too fast 
        }
        // we now want to turn till the side left sensor is on the main line as the main line could be either side of middle line sensors 
        while (digitalRead(SL_S) == 0) {
            turn('c'); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        // now we know main line is between SL_S and L_S
        // Turning till R_S touches the main line. Meaning robot is on main line
        while (digitalRead(R_S) == 0) {
            turn('a'); delay(30);
            digitalRead(R_S);
            park(); delay(10);
        }
        //robot is now reattached to main line
    }
    if (direction == 'd') //argument d denotes dropoff so robot is coming out tunnel to the side to drop off the block
    {
        // we want robot to move forward till the side left sensor detects a line
        while (digitalRead(SL_S) == 0) {
            forward(); delay(30);
            digitalRead(SL_S);
            park(); delay(10); // make sure robot doesn't move too fast 
        }
        // we now want to turn till the side right sensor is on the main line as the main line could be either side of middle line sensors
        while (digitalRead(SR_S) == 0) {
            turn('a'); delay(30);
            digitalRead(SR_S);
            park(); delay(10);
        }
        // now we know main line is between SR_S and R_S
        // Turning till L_S touches the main line. Meaning robot is on main line
        while (digitalRead(L_S) == 0) {
            turn('c'); delay(30);
            digitalRead(SR_S);
            park(); delay(10);
        }
        //robot is now reattached to main line
    }
    park();
    // robot should now be on the main line either side of the tunnel depending on argument
    }
//function to get up to stop infront of the 1st block using the front Ultrasound and detect whether the block is magnetic​ then switch on the corresponding LED
void next_to_block() {
    int block1st_infront = 2; // value for when the robot is infront of the 1st block 
    int block_dist = 35; // placeholder for the distance infront of the robot
    // since 1st block is on the main line so the robot can simply follow the main line till a small distance is sensed infront of the robot
    while (block_dist > block1st_infront) {
        // line follows till the 1st block is detected
        line_follow(); // calls the line follow function once
        block_dist = calculateFD(); // checks value of distance infront of the robot and updates relevant variable
        // print statements for unit testing
        Serial.println("Moving up to block"); 
        Serial.println(block_dist);
    }
    park();
    //now robot should be parked infront of the 1st block
    // Robot then checks whether the block is magnetic or not
    // hall monitor gives digital reading due to comparator circuit
    if (digitalRead(hall_p) == 1) {
        Serial.println("Magnet detected"); // print statement used in testing
        //signaling which type of block robot has identified
        digitalWrite(red_LED, HIGH);
        delay(6000);
        is_magnet = 'y'; // 'y' is then stored in the relevant variable to remeber which box to drop off in (red, as magnetic)
    }
    else {
        Serial.println("no magnet"); // print statement used in testing
        //signaling which type of block robot has identified
        digitalWrite(green_LED, HIGH);
        delay(6000);
        is_magnet = 'n'; // 'n' is then stored in the relevant variable to remeber which box to drop off in (green, as non-magnetic)

    }
    //robot should now be parked infront of the block with a correct LED to signal which type of block has been identified
}

//function to ​turn 180 degrees and reattach to main line when 1st block collected
void turn_180_deg() {
    //initial small turn so middle right line sensor is past white line - robto should be roughly 22 degrees turned
    turn('a'); delay(170); park();  // this time was found through testing
    //now turning till the right middle sensor touches main line -- main line will then be inbetween the middle 2 line sensors
    while (digitalRead(R_S) == 0) {
        // while loop until 
        turn('a'); delay(30);
        digitalRead(R_S); // reading middle right sensor
        park(); delay(20); // small stop so robot doesn't turn fast enough to miss the line
    }
    //robot should now be turned 180 degrees and attached to the main line and towards the tunnel
}

//function to drop off block in required box then return to the starting box
void dropoff(char block_magnetic) {
    // whether block is magnetic or not we have to reach the white line coming from the green box (T-junction to green box)
    while (digitalRead(SR_S) == 0) {//we wnat to stop moving once SL_S hits the whit line going to the green box
        line_follow();
        digitalRead(SR_S);
    }
    // now depending if block is magnetic or not we need to drop block in green or red box
    if (block_magnetic == 'n') { // case where block is not magnetic so it can be dropped off in the green box
        Serial.println("non magnetic block so dropping off in green"); 
        //initaly turning robot more perpendicular to green box
        while (digitalRead(SL_S) == 0) {
            turn('c'); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        // side left sensor should now have hit main line
        //turning set amount then driving a small bit beyond main line- good values found through testing
        turn('c'); delay(120); forward(); delay(60); park();
        //robot now roughly perpendicular to green box and so it can drive forward to drop off the block
        while (digitalRead(SL_S) == 0) { //when side left sensor hits the green box this gives us a much more stable idea of how far forward the robot has gone
            forward(); delay(20);
            digitalRead(SL_S);
            park();delay(10);
        }
        //now moving forward to push block into the green square - time values found through testing
        forward(); delay(120); park();
        //block now inside green dropoff area
    }

    else { //block is magnetic so block must be dropped off in the red box
        //robot needs to get past 2 T junctions before it is on the main line infront of the red box
        int T_passed = 0; // we want the robot to pass 2 T junctions and reach the red square
        //This is done by looping twice the code to get beyond a T junction attaching to main line and line following till next T junction
        while (T_passed < 2) {
            //achieved by slight turn anticlockwise then forward movement before SL_S hits line
            turn('a'); delay(100); park(); // values found through testing
                while (digitalRead(SL_S) == 0) {
                    forward(); delay(20);
                    digitalRead(SL_S);
                    park(); delay(10);
                }
            //now main line between SL_S and L_S and robot past T junction
            //getting robot back onto main line
            turn('c'); delay(120); //angling robot towards main line
            while (digitalRead(R_S) == 0) {
                forward(); delay(20);
                digitalRead(R_S);
                park(); delay(10);
            }
            //robot should not be on main line
            //moving up to next T junction by line following until the SR_S senses junction
            while (digitalRead(SR_S) == 0) {
                line_follow();
                Serial.println("Moving between T junctions"); //print statement for testing
                digitalRead(SR_S);
            }
            T_passed++;
            //now robot should be stopped on a T junction
        }
        // robot should now be stopped on the T junction infront of the red box
        Serial.println("magnetic block so dropping off in red"); //print statement for testing
        //initaly turning robot more perpendicular to red box
        while (digitalRead(SL_S) == 0) {
            turn('c'); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        // side left sensor should now have hit main line
        //turning set amount then driving a small bit beyond main line- good values found through testing
        turn('c'); delay(120); forward(); delay(60); park();
        //robot now roughly perpendicular to red box and so it can drive forward to drop off the block
        while (digitalRead(SL_S) == 0) { //when side left sensor hits the green box this gives us a much more stable idea of how far forward the robot has gone
            forward(); delay(20);
            digitalRead(SL_S);
            park(); delay(10);
        }
        //now moving forward to push block into the red square - time values found through testing
        forward(); delay(120); park();
        //block now inside red dropoff area
    }
}

//function to return to the start. Takes an argument depending on if the robot is inside the red or green box
void return_to_start(char block_magnetic){
        if (block_magnetic == 'n') { //if block is non magnetic we are in the green box
        //reversing till we are sufficiently seperated from the block
        while (digitalRead(SL_S) == 0) {
            backward(); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        //SL_S now on the green line
        //we now want to reverse to main line to make sure we are sufficiently seperated
        //first backing a small amount to make sure the SL_S is beyond the green line
        backward(); delay(80); park();
        while (digitalRead(SL_S) == 0) {
            backward(); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        //SL_S now on main line
        //robot turns to angle towards the starting area
        turn('a'); delay(120);
        //robot now moves forward till sensors are past the main line
        forward(); delay(170); park();
        //moving robot forward till L_S indicates we are on the perimeter of the starting area
        while (digitalRead(L_S) == 0) {
            forward(); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        //finally moving forward till inside then stopping - good value found through testing
        forward(); delay(220); park();
    }

    else { // if block was magnetic then we will follow a very similar procedure
        //reversing till we are sufficiently seperated from the block
        while (digitalRead(SL_S) == 0) {
            backward(); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        //SL_S now on the red line
        //we now want to reverse to main line to make sure we are sufficiently seperated
        //first backing a small amount to make sure the SL_S is beyond the red line
        backward(); delay(80); park();
        while (digitalRead(SL_S) == 0) {
            backward(); delay(30);
            digitalRead(SL_S);
            park(); delay(10);
        }
        //SL_S now on main line
        //robot turns to angle towards the starting area
        turn('c'); delay(170);
        //robot now moves forward till sensors are past the main line
        forward(); delay(170); park();
        //moving robot forward till R_S indicates we are on the perimeter of the starting area
        while (digitalRead(R_S) == 0) {
            forward(); delay(30);
            digitalRead(R_S);
            park(); delay(10);
        }
        //finally moving forward till inside then stopping - good value found through testing
        forward(); delay(220); park();
    }
}

void loop() {
    //making sure LEDs off before starting 
    digitalWrite(red_LED, LOW);
    digitalWrite(amber_LED, LOW);
    digitalWrite(red_LED, LOW);
    // here we will call the different neccessary functions in the main loop
    while (analogRead(push) < 990) {Serial.println("Waiting on button press to begin"); delay(25);} // infinite loop so that robot will not start till push button pressed 
    startup(); // robot now on main line between the green and starting boxes
    past_green_box(); // robot now on main line infornt of the tunnel
    //calling the line follow function an adequete number of times to make sure the robot is inside the tunnel - values found through testing
    int steps_to_tunnel = 0;
    while (steps_to_tunnel < 6) {
        line_follow();
        steps_to_tunnel++;
    }
    traverse_tunnel(); // now outside tunnel
    reattach_to_main('p'); //robot now reattached to main line 
    next_to_block(); // robot now identified and infront of block
    //no grabber mechanism needs to be activated 
    turn_180_deg();//robot now attached to main line and faced towards tunnel
    // robot then needs to follow line till side distance sensor detects we are inside the tunnel
    int inside_tunnel = 370;
    while (analogRead(side_ds) > inside_tunnel) {
        line_follow();
        analogRead(side_ds);
    }
    //now should be inside the tunnel 
    traverse_tunnel(); // now traversing through tunnel and arrive outside on drop off side
    reattach_to_main('d'); //robot now reattached to main line outside the tunnel
    //now depending on if the block is magnetic or not 1 or 3 crosses must be passed - this is achieved by passing the is_magnet variable to the dropoff/return_to_start functions
    dropoff(is_magnet); //robot now dropped off block in relevant area
    return_to_start(is_magnet);
    park();
    //robot should now have returned to the starting area and finished its mission
}
