//*****************************************************************************
//***********************   POINT_TO_POINT_CONTROL.CPP  ***********************
//***********************     Author: Livio Bisogni     ***********************
//*********************** © 2021 Turtley & Turtles Ltd. ***********************
//*****************************************************************************

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                                  INSTRUCTIONS

    Please read the attached `README.md` file.
_____________________________________________________________________________*/


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
--------------------------------- HEADER FILES --------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <termios.h>


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL CONSTANTS ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

const double TOLERANCE = 0.01;  // linear error tolerance; once linear error
                                // falls below this threshold, the turtle stops
const double KP_LINEAR  = 0.5;
const double KP_ANGULAR = 4.0;
const int    KEY_ESC    = 27;  // ASCII code equivalence
// Walls coordinates:
const double LEFT_WALL   = 0.0;   // left wall
const double BOTTOM_WALL = 0.0;   // bottom wall
const double RIGHT_WALL  = 11.0;  // right wall
const double TOP_WALL    = 11.0;  // top wall


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL VARIABLES ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

turtlesim::Pose       currentPose;  // current pose
geometry_msgs::Pose2D goalPose;     // target pose
bool stop = true;  // flag to stop the turtle when it's set to 'true'


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
----------------------- CALLBACK & FUNCTION DEFINITIONS -----------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCURRENTPOSECALLBACK:     Get currrent pose
_____________________________________________________________________________*/

void getCurrentPoseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    currentPose.x     = msg->x;
    currentPose.y     = msg->y;
    currentPose.theta = msg->theta;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETGOALPOSECALLBACK:        Get goal pose
_____________________________________________________________________________*/

void getGoalPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    stop = false;  // Start loop

    goalPose.x = msg->x;
    goalPose.y = msg->y;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCH:      Get pressed key character; non-blocking function.
                Code adapted from:

                https://answers.ros.org/question/63491/keyboard-key-pressed/
_____________________________________________________________________________*/

char getch()
{
    fd_set         set;
    struct timeval timeout;
    int            rv;
    char           buff     = 0;
    int            len      = 1;
    int            filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec  = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN]  = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0) {
        // ROS_INFO("no_key_pressed"); // DEBUG
    } else
        read(filedesc, &buff, len);

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETANGULARERROR:        Get angular error between direction of the turtle
                            and direction to the desired pose
_____________________________________________________________________________*/

double getAngularError(turtlesim::Pose       current_pose,
                       geometry_msgs::Pose2D goal_pose)
{
    // Create linear error vector
    double E_x = goal_pose.x - current_pose.x;  // Error along X component
    double E_y = goal_pose.y - current_pose.y;  // Error along Y component

    // Get desired angle
    double desired_theta = atan2f(E_y, E_x);

    // Compute angular error
    double E_theta = desired_theta - current_pose.theta;

    return E_theta;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETLINEARERROR:     Get linear error from the turtles perspective.
                        Error only along turtle X axis
_____________________________________________________________________________*/

double getLinearError(turtlesim::Pose       current_pose,
                      geometry_msgs::Pose2D goal_pose)
{
    // Create linear error vector
    double E_x = goal_pose.x - current_pose.x;  // Error along X component
    double E_y = goal_pose.y - current_pose.y;  // Error along Y component
    double E_theta =
        getAngularError(current_pose, goal_pose);  // get angle between vectors

    // Projection error onto turtle X axis
    // double E_thetax = hypotf(E_x, E_y) * cos(E_theta); // CANC
    double E_thetax = hypot(E_x, E_y) * cos(E_theta);

    return E_thetax;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    ISINALLOWEDREGION:      Check whether a desired pose is in the allowed
                            region
_____________________________________________________________________________*/

bool isInAllowedRegion()
{
    return (goalPose.x >= LEFT_WALL && goalPose.x <= RIGHT_WALL &&
            goalPose.y >= BOTTOM_WALL && goalPose.y <= TOP_WALL);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MANAGETURTLEOUTOFRANGE:     Manage a desired pose outside of the allowed
                                region
_____________________________________________________________________________*/

void manageTurtleOutOfRange()
{
    printf("[WARN]: Goal pose out of range:\n");
    if (goalPose.x < LEFT_WALL)
        printf("\t\tgoalPose.x = %f < %.1f\n", goalPose.x, LEFT_WALL);
    if (goalPose.x > RIGHT_WALL)
        printf("\t\tgoalPose.x = %f > %.1f\n", goalPose.x, RIGHT_WALL);
    if (goalPose.y < BOTTOM_WALL)
        printf("\t\tgoalPose.y = %f < %.1f\n", goalPose.y, BOTTOM_WALL);
    if (goalPose.y > TOP_WALL)
        printf("\t\tgoalPose.y = %f > %.1f\n", goalPose.y, TOP_WALL);
    printf("Please select another goal pose.\n\n");
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MAIN:       Da main
_____________________________________________________________________________*/

int main(int argc, char **argv)
{
    int c = 0;  // key pressed (ASCII code)

    // Node creation
    ros::init(argc, argv, "point_to_point");
    ros::NodeHandle nh;

    // Subscriber for reading goal pose
    ros::Subscriber goalPose_sub =
        nh.subscribe("/turtle1/PositionCommand", 5, getGoalPoseCallback);

    // Subscriber for reading current (actual) pose
    ros::Subscriber currentPose_sub =
        nh.subscribe("/turtle1/pose", 5, getCurrentPoseCallback);

    // Publisher for velocity
    ros::Publisher velocity_pub =
        nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    // Initializing
    geometry_msgs::Twist CmdVel;            // Twist command
    double               linearError  = 0;  // Distance error
    double               angularError = 0;  // Angular error

    // Node frequency
    ros::Rate loop_rate(10);  // Frequency to run loops to (10 Hz)

    int messagesCounter_moving      = 1;
    int messagesCounter_waiting     = 0;
    int messagesCounter_goalReached = 1;

    // Execute until node and channel are ok, and esc key is not pressed
    while (ros::ok() && nh.ok() && c != KEY_ESC) {
        // When the flag 'stop' is set to false, keep moving the turtle with a
        // proportional controller.
        if (stop == false) {
            if (isInAllowedRegion()) {
                // moveTurtle(velocity_pub);

                if (messagesCounter_moving == 0)
                    printf(
                        "The turtle is moving. Current position:\t(%f, %f)\n",
                        currentPose.x, currentPose.y);
                messagesCounter_goalReached = 0;

                linearError  = getLinearError(currentPose, goalPose);
                angularError = getAngularError(currentPose, goalPose);

                if (linearError > 0) {
                    CmdVel.linear.x = KP_LINEAR * linearError;
                } else {
                    CmdVel.linear.x = 0;  // Don't move backwards
                }

                CmdVel.angular.z = KP_ANGULAR * angularError;
                velocity_pub.publish(CmdVel);

                messagesCounter_waiting = 0;

            } else {
                manageTurtleOutOfRange();
                messagesCounter_waiting = 0;
            }
        } else {
            if (messagesCounter_waiting == 0)
                printf("\nWaiting for a goal pose...\n");
            messagesCounter_waiting++;
            messagesCounter_moving = 1;
        }

        if (fabs(linearError) < TOLERANCE) {
            stop = true;
            if (messagesCounter_goalReached == 0)
                printf("Goal pose (%f, %f) reached!\n", goalPose.x, goalPose.y);
            messagesCounter_goalReached++;
            messagesCounter_moving = 0;
        }

        c = getch();  // read character (non-blocking)

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}