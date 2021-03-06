#include <Sparki.h>
#include <math.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0265 // meters/second
#define CYCLE_TIME 0.025 // Default 25ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1

/*
range or workable final angle (h_err) for each starting bearing error:

b_err: min_h_err to max_h_err
60:  -65 to 90
45:  -70 to 89
0:   -80 to 80
-45: -89 to 70
-60: -90 to 65
*/

// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART3;
//const int current_state = /

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)

// Wheel rotation vars
float left_speed_pct = 0;
float right_speed_pct = 0;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180/3.1415;
}

void setup() {
  pose_x = 0.0;
  pose_y = 0.0;
  //pose_theta = to_radians(45);
  pose_theta = 0.0;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15, 0.15, to_radians(-40));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = sqrt(pow((dest_pose_x - pose_x), 2) + pow((dest_pose_y - pose_y), 2));  // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void updateOdometry() {
  // TODO: Update pose_x, pose_y, pose_theta
   /*pose_x = pose_x + ROBOT_SPEED * CYCLE_TIME * cos(pose_theta);
   pose_y = pose_y + ROBOT_SPEED * CYCLE_TIME * sin(pose_theta);*/

    float d_left = left_speed_pct*CYCLE_TIME*ROBOT_SPEED;
    float d_right = right_speed_pct*CYCLE_TIME*ROBOT_SPEED;
    float d_theta = (d_right - d_left)/AXLE_DIAMETER;
    pose_x += cos(pose_theta + d_theta/2)*(d_left+d_right)/2;
    pose_y += sin(pose_theta + d_theta/2)*(d_left+d_right)/2;
    pose_theta += d_theta;

/* Trying to find accurate x,y value with math which does not work well.
    if (d_theta <= 0.0001){//As long as direction is close enough to straight...
        pose_x += cos(pose_theta)*(d_left);
        pose_y += sin(pose_theta)*(d_left);
    } else {
        float th = pose_theta - M_PI/2;
        float r_left = d_left/d_theta;
        float robot_to_center = AXLE_DIAMETER/2 + r_left;
        pose_y = pose_y + (cos(th) - cos(th+d_theta))*robot_to_center;
        pose_x = pose_x + (sin(th) - sin(th+d_theta))*robot_to_center;
        pose_theta += d_theta;
    }*/

    //pose_x = pose_x + cos(pose_theta)*((ROBOT_SPEED * CYCLE_TIME * left_speed_pct)/2  + (ROBOT_SPEED * CYCLE_TIME * right_speed_pct)/2);
    //pose_y = pose_y + sin(pose_theta)*((ROBOT_SPEED * CYCLE_TIME * left_speed_pct)/2  + (ROBOT_SPEED * CYCLE_TIME * right_speed_pct)/2);
    //pose_theta = pose_theta + (right_speed_pct * ROBOT_SPEED * CYCLE_TIME)/AXLE_DIAMETER - (left_speed_pct * ROBOT_SPEED * CYCLE_TIME)/AXLE_DIAMETER;

    // Bound theta //prob never used anyway
    if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
    if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

void displayOdometry() {

  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dtg: ");
  sparki.print(orig_dist_to_goal);
  sparki.print("ls: ");
  sparki.print(left_speed_pct);
  sparki.print("rs: ");
  sparki.print(right_speed_pct);
}


void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();

      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        pose_theta = pose_theta + (ROBOT_SPEED*CYCLE_TIME*1000)/(AXLE_DIAMETER/2);
        sparki.moveLeft();

      } else if (line_right < threshold) {
        pose_theta = pose_theta + (ROBOT_SPEED*CYCLE_TIME)/(AXLE_DIAMETER/2);

        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }
      updateOdometry();

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      b_err = atan((dest_pose_y - pose_y)/(dest_pose_x - pose_x));
      sparki.moveLeft(to_degrees(b_err));
      delay(1000);
      sparki.moveForward(orig_dist_to_goal*100);
      delay(1000);
      sparki.moveLeft(to_degrees(pose_theta-b_err+dest_pose_theta));
      delay(2000);

      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      updateOdometry();
      left_speed_pct = 0;
      right_speed_pct = 0;
      h_err = dest_pose_theta - pose_theta;
      float abs_err = fabs(h_err);
      float x_dot = sqrt(pow((dest_pose_x - pose_x), 2) + pow((dest_pose_y - pose_y), 2));
      //Goal not reached loop
      if (x_dot+abs_err/10 > 0.006){
          b_err = atan((dest_pose_y - pose_y)/(dest_pose_x - pose_x)) - pose_theta;
          //h_err = dest_pose_theta - pose_theta;
          //float x_dot = orig_dist_to_goal;
          //float theta_dot = 0.1*b_err + 0.01*h_err;
          //phi_l = (2*x_dot - theta_dot*AXLE_DIAMETER)/AXLE_DIAMETER;
          //phi_r = (2*x_dot + theta_dot*AXLE_DIAMETER)/AXLE_DIAMETER;
          //phi_l = (x_dot - (b_err*(0.01+x_dot) + h_err*0.025));
          //phi_r = (x_dot + (b_err*(0.01+x_dot) + h_err*0.025));
          float pct_traveled = x_dot/orig_dist_to_goal;
          //phi_l = (pct_traveled - b_err)*x_dot - h_err*0.01;
          //phi_r = (pct_traveled + b_err)*x_dot + h_err*0.01;
          //Make overcorrection berr with herr | correct berr | correct herr at terminal(near 1cm)
          phi_l = x_dot*x_dot*(pct_traveled/2 - (b_err-(2*h_err*pow(pct_traveled/2,2))-0)) - h_err*0.00003;
          phi_r = x_dot*x_dot*(pct_traveled/2 + (b_err-(2*h_err*pow(pct_traveled/2,2))-0)) + h_err*0.00003;
          // TODO: Implement solution using motorRotate and proportional feedback controller.
          // sparki.motorRotate function calls for reference:
          if(phi_l >= phi_r){
            left_speed_pct = 1.0;
            right_speed_pct = phi_r/phi_l;
          } else {
            right_speed_pct = 1.0;
            left_speed_pct = phi_l/phi_r;
          }

          //begin_time = millis(); //make sure motor run full amount of cycle time.
          if(left_speed_pct>=0) sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
          else sparki.motorRotate(MOTOR_LEFT, right_dir, -int(left_speed_pct*100.));
          if(right_speed_pct>=0) sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));
          else sparki.motorRotate(MOTOR_RIGHT, left_dir, -int(right_speed_pct*100.));

          sparki.RGB(RGB_ORANGE);
      } else { // goal reached
      sparki.RGB(RGB_GREEN);
      sparki.moveStop();
      delay(10000);
      }
      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME){
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  }
  else
    delay(10);
}
