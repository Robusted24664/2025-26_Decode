package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import android.util.Log;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



public class RoRobot {


    ////////////////////////////////////
    // Hardware interface
    ////////////////////////////////////

    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontRight = null;

    public DcMotor flywheel = null;

    public DcMotor intakeFront = null;
    public DcMotor intakeLift = null;

    public Servo gate = null;
    public Servo kicker = null;

    public GoBildaPinpointDriver odo = null;


    ////////////////////////////////////
    // Important Constants
    ////////////////////////////////////

    // controlling the gate for shooting
    public static final double GATE_OPEN_POSITION = 0.5785;
    public static final double GATE_CLOSED_POSITION = 0.4783;
    public static final double GATE_CLOSE_DELAY = 0.25; //seconds

    // controlling the kicker for shooting
    public static final double KICKER_TOP_POSITION = 0.2484;
    public static final double KICKER_BOTTOM_POSITION = 0.68;
    public static final double KICKER_CLOSE_DELAY = 0.75; //seconds

    // pocketing balls to avoid pyramid (deprecated)
    public static final int BALL_INTO_INTAKE_DELTA = -300;
    public static final int BALL_OUT_OF_INTAKE_DELTA = 550;

    // speed of the flywheel
    public static final double SHOOTING_POWER = 0.498;

    // adjust for weight distribution to improve strafing
    public static final double BACK_DRIVE_MULTIPLIER = 1.1;
    public static final double FRONT_DRIVE_MULTIPLIER = 1.0;

    ////////////////////////////////////
    // State info
    ////////////////////////////////////


    ////////////////////////////////////
    // Robot position information
    public RoPose poseEst = RoPose.getTrivial();
    public static boolean isPoseBaseInitialized = false;
    public static RoPose poseBase = RoPose.getTrivial();
    public static double TRACK_WIDTH = 14.50; //inches

    ////////////////////////////////////
    // parent opmode
    public static LinearOpMode opMode = null;

    ////////////////////////////////////
    //driving parameters
    public enum DRIVE_DIRECTION {
        FORWARD,
        REVERSE
    }
    // default driving parameters:
    public static class DriveParameters{
        public double vCMax = 0.5;
        public double vDiffMax = 0.4;
        public double slowDistance = 15;
        public double slowAngleD = 10;
        public double threshold = 2;
        public double thresholdD = 2;
        public DRIVE_DIRECTION direction = DRIVE_DIRECTION.FORWARD;
    }

    ////////////////////////////////////
    // UTILITY: Kicker movement
    ////////////////////////////////////

    public void kickerUp() {
        kicker.setPosition(KICKER_TOP_POSITION);
    }
    public void kickerDown() {
        kicker.setPosition(KICKER_BOTTOM_POSITION);
    }

    ////////////////////////////////////
    // UTILITY: Gate movement
    ////////////////////////////////////

    public void gateOpen() {
        gate.setPosition(GATE_OPEN_POSITION);
    }
    public void gateClose() {
        gate.setPosition(GATE_CLOSED_POSITION);
    }

    ////////////////////////////////////
    // UTILITY: Flywheel control
    ////////////////////////////////////

    public void flywheelOn(){
        flywheel.setPower(SHOOTING_POWER);
    }

    public void flywheelOff(){
        flywheel.setPower(0);
    }

    ////////////////////////////////////
    // UTILITY: Intake control
    ////////////////////////////////////

    public void intakeOn(){
        intakeFront.setPower(1.0);
        intakeLift.setPower(0.5);
    }

    public void intakeOff(){
        intakeFront.setPower(0.0);
        intakeLift.setPower(0.0);
    }

    public void intakeReverse(){
        intakeFront.setPower(-1.0);
        intakeLift.setPower(-0.5);
    }

    ////////////////////////////////////
    // UTILITY: Firing in Auto
    ////////////////////////////////////

    public void autoFireSequence() {
        // sleep times are in milliseconds
        long gateAfterStart   = Math.round(1000*GATE_CLOSE_DELAY);
        long kickerAfterStart = Math.round(1000*KICKER_CLOSE_DELAY);
        long kickerAfterGate  = kickerAfterStart - gateAfterStart;
        gateOpen();
        opMode.sleep(gateAfterStart);
        gateClose();
        kickerUp();
        opMode.sleep(kickerAfterGate);
        kickerDown();
    }

    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Constructor
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Android Studio was complaining about there being no constructor.
    // So now there is a constructor, but it does nothing.
    //
    ////////////////////////////////////

    public RoRobot() {
        // do nothing
    }

    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Robot Initialization
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Call ONE of the initializers exactly ONCE at the beginning of the opmode.
    // Note: Must be a LinearOpMode
    //
    ////////////////////////////////////


    /*********************
     * initializeAuto
     *   - initialization for auto opmodes
     * .
     * Inputs:
     *  hardwareMap - hardwareMap from the parent opMode
     *  opMode      - parent opMode
     *********************/

    public void initializeAuto(HardwareMap hardwareMap, LinearOpMode opMode) {
        initializeGeneric(hardwareMap, opMode);
        odo.resetPosAndIMU();
        gateClose();
        kickerDown();

    }

    /*********************
     * initializeTeleOp
     *   - initialization for teleop opmodes
     * .
     * Inputs:
     *  hardwareMap - hardwareMap from the parent opMode
     *  opMode      - parent opMode
     *********************/

    public void initializeTeleOp(HardwareMap hardwareMap, LinearOpMode opMode) {
        initializeGeneric(hardwareMap, opMode);
    }


    /*********************
     * initializeGeneric
     *   - Generic initialization for all opmodes
     * .
     * Inputs:
     *  hardwareMap - hardwareMap from the parent opMode
     *  opMode      - parent opMode
     *********************/

    public void initializeGeneric(HardwareMap hardwareMap, LinearOpMode parentOpMode) {

        ////////////////////////////////////
        // save the parent opmode for later use

        opMode = parentOpMode;

        ////////////////////////////////////
        // Initialize the motors variables

        frontLeft  = getAndInitDriveMotor(hardwareMap, "frontLeft");
        backLeft   = getAndInitDriveMotor(hardwareMap, "backLeft");
        frontRight = getAndInitDriveMotor(hardwareMap, "frontRight");
        backRight  = getAndInitDriveMotor(hardwareMap, "backRight");

        flywheel   = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeLift  = hardwareMap.get(DcMotor.class, "intakeLift");

        ////////////////////////////////////
        // Initialize the servos

        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");

        ////////////////////////////////////
        // Initialize the pinpoint odometry computer

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-85.0, -140.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

    }

    ////////////////////////////////////
    // INITIALIZATION UTILITY
    ////////////////////////////////////

    /*********************
     * getAndInitDriveMotor
     *   - retrieve and initialization a drive motor
     * .
     * Inputs:
     *  hardwareMap - hardwareMap from the parent opMode
     *  name        - name for the motor in the hardware map
     * .
     * Note: all drive motors are initialized in the same way
     *********************/

    private DcMotor getAndInitDriveMotor(HardwareMap hardwareMap, String name) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // No encoders on drive motors
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // REPORTING / TELEMETRY
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////


    /*********************
     * reportPositionData
     *   - Tell the driver station where you think you are
     * .
     * Note: Put whatever you think would be helpful here. This is
     *   called during various of the loops that are intended for
     *   auto
     *********************/


    public void reportPositionData() {
        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
         */
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", poseEst.position.x, poseEst.position.y, poseEst.heading*180/Math.PI);
        opMode.telemetry.addData("Position", data);
        opMode.telemetry.addData("Status", odo.getDeviceStatus());

        opMode.telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        opMode.telemetry.update();
    }



    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // ODOMETRY
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // Odometry -- keeping track of where we are.
    //
    // This is VERY important for all the higher-level driving functions
    //
    ////////////////////////////////////

    /*********************
     * setPoseD
     * - tells the robot where it currently is on the field
     *   (with heading in degrees)
     * .
     * Inputs:
     *   x         -  x position on the field (in inches)
     *   y         -  y position on the field (in inches)
     *   headingD  -  heading (in degrees)
     *********************/

    public void setPoseD(double x, double y, double headingD) {
        //distances in inches
        RoPose currentPose = RoPose.RoPoseD(x, y, headingD);
        odo.update();
        Pose2D goBildaPos = odo.getPosition();
        double X=goBildaPos.getX(DistanceUnit.INCH);
        double Y=goBildaPos.getY(DistanceUnit.INCH);
        double heading=goBildaPos.getHeading(AngleUnit.RADIANS);
        RoPose relPoseEst = new RoPose(X, Y, heading);
        poseBase = currentPose.then(relPoseEst.inverse());
        isPoseBaseInitialized = true;
        updatePose();
    }

    /*********************
     * updatePose
     * - tells the robot to get updated pose information from the
     *   pinpoint odometry computer
     * .
     * Note: This sets poseEst, which is then used by all the
     *   higher-level driving functions as the "current" or "most
     *   recent" estimate of the robot's pose.
     *********************/

    public void updatePose() {
        odo.update();
        Pose2D goBildaPos = odo.getPosition();
        double x=goBildaPos.getX(DistanceUnit.INCH);
        double y=goBildaPos.getY(DistanceUnit.INCH);
        double heading=goBildaPos.getHeading(AngleUnit.RADIANS);
        RoPose relPoseEst = new RoPose(x, y, heading);
        poseEst = poseBase.then(relPoseEst);
    }



    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // BASIC TANK DRIVE
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // This is used by the higher-level modified Pure Pursuit functions
    // IMPORTANT: This is also used for turning!
    //
    ////////////////////////////////////

    /*********************
     * setLeftRightDrivePower
     *   - set the left and right motor powers
     * .
     * Inputs:
     *   leftPower  - requested forward velocity on left side of the robot
     *   rightPower - requested forward velocity on left side of the robot
     * .
     * Note: This function checks to see if either "power" is outside
     *   the standard FTC range of (-1,1).  If so, then it *scales*
     *   the velocities so that the bigger one is still in range.  If
     *   you just *clipped* the velocities, then the ratio between
     *   left and right powers would change, and that would change the
     *   rate of turning!
     *********************/
    public void setLeftRightDrivePower(double leftPower, double rightPower) {
        // find max absolute value if it is over 1.0
        double maxAbsoluteValue = 1.0;
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(leftPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(rightPower));

        // scale down
        leftPower  /= maxAbsoluteValue;
        rightPower /= maxAbsoluteValue;

        // set power levels
        frontLeft.setPower (leftPower);
        backLeft.setPower  (leftPower);
        backRight.setPower (rightPower);
        frontRight.setPower(rightPower);
    }

    /*********************
     * setForwardTurnDrivePower
     *   - set the average forward power and the right/left power difference
     * .
     * Inputs:
     *   vC    - requested forward velocity at center of robot
     *   vDiff - requested velocity difference between the two wheels
     *********************/
    public void setForwardTurnDrivePower(double vC, double vDiff) {
        double vR = vC+(vDiff/2);
        double vL = vC-(vDiff/2);
        setLeftRightDrivePower(-vL, -vR);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // BASIC MECANUM DRIVE
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // This is used in all the "strafe" functions
    //
    ////////////////////////////////////


    /*********************
     * strafeDrive
     *   - set the drive motors according to standard mecanum inputs
     * .
     * Inputs:
     *   drive  - average forward speed
     *   strafe - average left/right strafing speed
     *   turn   - turning speed
     *********************/

    public void strafeDrive(double drive, double strafe, double turn) {
        double backLeftPower     = drive + turn + strafe;
        double backRightPower    = drive - turn - strafe;
        double frontLeftPower    = drive + turn - strafe;
        double frontRightPower   = drive - turn + strafe;

        // adjust for weight distribution to improve strafing
        backRightPower  *= BACK_DRIVE_MULTIPLIER;
        backLeftPower   *= BACK_DRIVE_MULTIPLIER;
        frontRightPower *= FRONT_DRIVE_MULTIPLIER;
        frontLeftPower  *= FRONT_DRIVE_MULTIPLIER;

        // find max absolute value if it is over 1.0
        double maxAbsoluteValue = 1.0;
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(backLeftPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(backRightPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(frontLeftPower));
        maxAbsoluteValue = Math.max(maxAbsoluteValue, Math.abs(frontRightPower));

        // scale down so that maxAbsoluteValue becomes 1.0
        double multiplier = 1.0/maxAbsoluteValue;
        backLeft.setPower(backLeftPower*multiplier);
        backRight.setPower(backRightPower*multiplier);
        frontLeft.setPower(frontLeftPower*multiplier);
        frontRight.setPower(frontRightPower*multiplier);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // TURNING
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // These functions are for turning in place.
    //
    // The strafeToPoseXXX functions can do similar things, but these
    // functions do a better job of controlling the turn.
    //
    ////////////////////////////////////


    ////////////////////////////////////
    //
    // TURNING:   IN RADIANS
    //
    ////////////////////////////////////


    /*********************
     * turnToHeading
     *   - turn in place to a heading specified in RADIANS
     * .
     * Inputs:
     *   heading   - target heading (in radians)
     *   vDiffMax  - largest vDiff to allow
     *   slowAngle - start slowing down once robot heading is this close (in radians)
     *   threshold - stop once robot heading is this close (in radians)
     *   timeoutS  - timeout in seconds
     * .
     * Notes:
     * .
     *  - After slowAngle is met, robot attempts to slow down at a
     *    constant deceleration while continuing to turn toward the
     *    target heading.  The goal is to make a smooth stop without
     *    over-turning.
     * .
     *  - HOWEVER, when the robot gets to the threshold this function
     *    returns WITHOUT turning off the motors, assuming that that
     *    next function will take over control of the robot motion.
     *    This is to avoid jerky start-stop movements.  If the robot
     *    absolutely needs to stop at the end, use
     *    turnToHeadingAndStop.
     * .
     *********************/

    public void turnToHeading(
            double heading,
            double vDiffMax,
            double slowAngle,
            double threshold,
            double timeoutS) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutS) {
            opMode.telemetry.addData("time", timer.seconds());
            opMode.telemetry.update();

            updatePose();
            double dHeading = RoPose.normalizeAngle(heading - poseEst.heading);
            Log.d("turnToHeading", "target heading: " + heading);
            Log.d("turnToHeading", "pose heading: " + poseEst.heading);
            Log.d("turnToHeading", "dHeading: " + dHeading);
            if (Math.abs(dHeading) < threshold) {
                Log.d("turnToHeading", "turn complete");
                break;
            }
            double ratioSqrt = Math.sqrt(Math.abs(dHeading/slowAngle));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05/vDiffMax, 1);
            double vDiffMin = 0.4; //value of 0.3 was too small and the robot got stuck one turn
            double vDiff = vDiffMax * clippedSqrt * Math.signum(dHeading);
            if (Math.abs(vDiff)<vDiffMin) {
                vDiff = vDiffMin*Math.signum(dHeading);
            }
            String message = "Turning! ";
            message+=" poseEst: "+poseEst;
            message+=String.format(", (vDiff)=(%.2f)", vDiff);
            message+=String.format(", (dHeading, ratioSqrt, clippedSqrt)=(%.2f,%.2f,%.2f)", dHeading, ratioSqrt, clippedSqrt);
            Log.d("turnToHeading", message);
            setForwardTurnDrivePower(0, vDiff);

            reportPositionData();
        }
    }

    /*********************
     * turnToHeading
     *   - As above, except not timeoutS parameter (defaults to 90 seconds)
     *********************/

    public void turnToHeading(
            double heading,
            double vDiffMax,
            double slowAngle,
            double threshold) {
        double timeoutS = 90; //this is an excessively long timeout (in seconds)
        turnToHeading(heading, vDiffMax, slowAngle, threshold, timeoutS);
    }

    /*********************
     * turnToHeadingAndStop
     *   - turn in place to a heading specified in RADIANS
     *   - then STOP
     * .
     * Inputs:
     *   headingD   - target heading (in radians)
     *   vDiffMax   - largest vDiff to allow
     *   slowAngleD - start slowing down once robot heading is this close (in radians)
     *   thresholdD - stop once robot heading is this close (in radians)
     *********************/

    public void turnToHeadingAndStop(
            double heading,
            double vDiffMax,
            double slowAngle,
            double threshold) {
        turnToHeading(heading, vDiffMax, slowAngle, threshold);
        setLeftRightDrivePower(0,0);
    }

    ////////////////////////////////////
    //
    // TURNING:   IN DEGREES
    //
    ////////////////////////////////////

    /*********************
     * turnToHeadingD
     *   - turn in place to a heading specified in DEGREES
     * .
     * Inputs:
     *   headingD   - target heading (in degrees)
     *   vDiffMax   - largest vDiff to allow
     *   slowAngleD - start slowing down once robot heading is this close (in degrees)
     *   thresholdD - stop once robot heading is this close (in degrees)
     * .
     * Note: This is essentially the same as turnToHeading, but all
     *   angles are specified in degrees.
     *********************/

    public void turnToHeadingD(
            double headingD,
            double vDiffMax,
            double slowAngleD,
            double thresholdD) {
        double heading = headingD * Math.PI/180;
        double slowAngle = slowAngleD * Math.PI/180;
        double threshold = thresholdD * Math.PI/180;
        turnToHeading(heading, vDiffMax, slowAngle, threshold);
    }

    /*********************
     * turnToHeadingD
     *   - turn in place to a heading specified in DEGREES, using DRIVE PARAMETERS
     * .
     * Inputs:
     *   headingD   - target heading (in degrees)
     *   params     - preset drive parameters
     *********************/

    public void turnToHeadingD(
            double headingD,
            DriveParameters params) {
        turnToHeadingD(headingD, params.vDiffMax, params.slowAngleD, params.thresholdD);
    }

    /*********************
     * turnToHeadingDAndStop
     *   - turn in place to a heading specified in DEGREES
     *   - then STOP
     * .
     * Inputs:
     *   headingD   - target heading (in degrees)
     *   vDiffMax   - largest vDiff to allow
     *   slowAngleD - start slowing down once robot heading is this close (in degrees)
     *   thresholdD - stop once robot heading is this close (in degrees)
     * .
     * Note: This is essentially the same as turnToHeadingD, but the
     *   motors are told to stop at the end.
     *********************/

    public void turnToHeadingDAndStop(
            double headingD,
            double vDiffMax,
            double slowAngleD,
            double thresholdD) {
        turnToHeadingD(headingD, vDiffMax, slowAngleD, thresholdD);
        setLeftRightDrivePower(0,0);
    }

    /*********************
     * turnToHeadingDAndStop
     *   - turn in place to a heading specified in DEGREES, using DRIVE PARAMETERS
     *   - then STOP
     * .
     * Inputs:
     *   headingD   - target heading (in degrees)
     *   params     - preset drive parameters
     *********************/

    public void turnToHeadingDAndStop(
            double headingD,
            DriveParameters params) {
        turnToHeadingD(headingD, params);
        setLeftRightDrivePower(0,0);
    }


    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // HIGH-LEVEL   STRAFING
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // These are high-level functions for driving to a specified pose
    // using strafing / mecanum primitives.
    //
    // Basically, you tell it what pose to go to and the robot goes
    // there.
    //
    // In normal usage, you would tell it to go to a number of
    // waypoints in sequence without stopping, then stop at the end
    // when you get to the final destination.
    //
    // The robot will head straight for its destination, but it will
    // also turn toward the final target heading while it is driving!
    // The effect is not like anything a human would do.
    //
    // It can also be slow if you end up driving a long distance while
    // the robot is not turned in teh direction it is driving. A
    // better option is often to tell the robot to strafe drive almost
    // all the way to the end with a target heading that lines up with
    // the direction the robot needs to go, then only tell it to turn
    // to the final heading in the short distance from that waypoint
    // until the end.  With a bit of work, we could write a function
    // to do that automatically, but that isn't written yet so for now
    // we have to set the waypoints by hand.
    //
    ////////////////////////////////////


    ////////////////////////////////////
    //
    // STRAFING:   INNER LOOP CONTENTS (you probably don't call this directly)
    //
    ////////////////////////////////////


    /*********************
     * stepTowardPose
     * .
     *  - tells the motors what to do at this loop iteration in order
     *    to follow a straight line toward the target x,y position
     *    while also turning toward the target heading
     * .
     * Inputs:
     *   target     - target pose
     *   vCMax      - max allowed center-of-robot velocity
     *   vDiffMax   - max allowed left/right velocity difference
     * .
     * Returns:
     *   robotPoseInTargetPOV -- where the robot is from the point of
     *     view of the target.  This is returned as a convenience to
     *     the calling loop, as it may be helpful for deciding when to
     *     do things such as stop.
     * .
     * Notes:
     * .
     *   - Given current robot pose, determines how current motors
     *     should be set in order to eventually get to target pose
     * .
     *   - This function just tells the robot what to do at this
     *     moment in time.  In order to actually end up at (or near)
     *     the target, you will need to call this function repeatedly
     *     in a loop and decide when to stop.
     * .
     *********************/

    public RoPose stepStrafeTowardPose(
            RoPose target,
            double vCMax,
            double vDiffMax) {
        updatePose();
        RoPose drivingPose = this.poseEst;
        // getting the target's pose in robot coordinates.
        RoPose relPose = drivingPose.getRelativeRoPoseFor(target);
        RoPose robotPoseInTargetPOV = relPose.inverse();
        // setting the coordinates, make typing easier
        double x = relPose.position.x;
        double y = relPose.position.y;
        double heading = relPose.heading; // heading in radians

        double tenDegrees = 10*Math.PI/180;
        double weight = 10/tenDegrees; //this is the weight that makes 10 deg = 1 inch
        double relLength = relPose.length(weight);
        double weightedHeading = heading*weight;

        double vX = x/relLength*vCMax; //forward velocity (on robot's x-axis)
        double vDiff = weightedHeading/relLength*vDiffMax; //turning speed
        double vY = y/relLength*vCMax; //left strafe velocity (on robot's y-axis)

        String message = "StrafeDriving! ";
        message+=" poseEst: "+poseEst;
        message+=" relPose: "+relPose;
        message+=String.format(", (vX, vY, vDiff)=(%.2f,%.2f,%.2f)", vX, vY, vDiff);
        Log.d("stepStrafeTowardPose", message);

        strafeDrive(-vX, -vY, vDiff);

        reportPositionData();

        return robotPoseInTargetPOV;
    }

    /*********************
     * stepStrafeTowardPose
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target  - target pose
     *   params  - preset drive parameters
     *********************/

    public void stepStrafeTowardPose(
            RoPose target,
            DriveParameters params) {
        stepStrafeTowardPose(target, params.vCMax, params.vDiffMax);
    }


    ////////////////////////////////////
    //
    // STRAFING:   GO TO POSE AND STOP
    //
    ////////////////////////////////////


    /*********************
     * strafeToPoseAndStop
     *   - Sets up a loop to repeatedly call stepStrafeTowardPose
     *   - takes care of slowing down and stopping when the robot
     *      gets near the target
     * .
     * Inputs:
     *   target        - target pose
     *   vCMax         - max allowed center-of-robot velocity
     *   vDiffMax      - max allowed left/right velocity difference
     *   slowDistance  - distance to start slowing before stop
     *   threshold     - stop when robot gets this close
     * .
     *********************/

    public void strafeToPoseAndStop(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double slowDistance,
            double threshold) {
        double vC = vCMax;
        while (opMode.opModeIsActive()) {
            RoPose robotPoseInTargetPOV = stepStrafeTowardPose(target, vC, vDiffMax);
            double tenDegrees = 10*Math.PI/180;
            double weight = 10/tenDegrees; //this is the weight that makes 10 deg = 1 inch
            double relLength = robotPoseInTargetPOV.length(weight);
            double ratioSqrt = Math.sqrt(Math.abs(relLength/slowDistance));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05/vCMax, 1);
            vC = vCMax * clippedSqrt;
            if (relLength<threshold) {
                break;
            }
        }
        setLeftRightDrivePower(0,0);
    }

    /*********************
     * strafeToPoseAndStop
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target  - target pose
     *   params  - preset drive parameters
     *********************/

    public void strafeToPoseAndStop(
            RoPose target,
            DriveParameters driveParams) {
        strafeToPoseAndStop(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.slowDistance,
                driveParams.threshold);
    }


    ////////////////////////////////////
    //
    // STRAFING:   GO TO POSE AND CONTINUE
    //
    ////////////////////////////////////


    /*********************
     * strafeToPoseAndContinue
     *   - Sets up a loop to repeatedly call stepStrafeTowardPose
     *   - Does *NOT* slow down and stop at the end
     * .
     * Inputs:
     *   target        - target pose
     *   vCMax         - max allowed center-of-robot velocity
     *   vDiffMax      - max allowed left/right velocity difference
     *   threshold     - stop when robot gets this close
     * .
     * Note: When robot gets near target this function simply returns
     *  with the robot still moving.  This assumes the next function
     *  in the auto sets a new target to drive to, and that eventually
     *  one of them will stop.
     *********************/

    public void strafeToPoseAndContinue(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double threshold) {
        while (opMode.opModeIsActive()) { // to do!!! check if op mode is active.
            RoPose robotPoseInTargetPOV = stepStrafeTowardPose(target, vCMax, vDiffMax);
            double tenDegrees = 10*Math.PI/180;
            double weight = 10/tenDegrees; //this is the weight that makes 10 deg = 1 inch
            double relLength = robotPoseInTargetPOV.length(weight);
            if (relLength<threshold) {
                break;
            }
        }
    }

    /*********************
     * strafeToPoseAndContinue
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target  - target pose
     *   params  - preset drive parameters
     *********************/

    public void strafeToPoseAndContinue(
            RoPose target,
            DriveParameters driveParams) {
        strafeToPoseAndContinue(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.threshold);
    }

    /*********************
     * strafeToPoseAndContinueD
     * .
     *   - Sets up a loop to repeatedly call stepStrafeTowardPose
     * .
     *   - Does *NOT* slow down and stop at the end
     * .
     *   - this is basically the same as strafeToPoseAndContinue, but
     *     angles are in DEGREES.  Since the Pose class uses radians,
     *     this ends up meaning that the target pose is specified as
     *     three parameters instead of packaged into a Pose.
     * .
     * Inputs:
     *   target_x        - x coordinate of target pose
     *   target_y        - y coordinate of target pose
     *   targetHeadingD  - target heading (in degrees)
     *   vCMax           - max allowed center-of-robot velocity
     *   vDiffMax        - max allowed left/right velocity difference
     *   threshold       - stop when robot gets this close
     * .
     * Note: When robot gets near target this function simply returns
     *  with the robot still moving.  This assumes the next function
     *  in the auto sets a new target to drive to, and that eventually
     *  one of them will stop.
     *********************/

    public void strafeToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            double vCMax,
            double vDiffMax,
            double threshold) {
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndContinue(target, vCMax, vDiffMax, threshold);
    }

    /*********************
     * strafeToPoseAndContinue
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target_x        - x coordinate of target pose
     *   target_y        - y coordinate of target pose
     *   targetHeadingD  - target heading (in degrees)
     *   params          - preset drive parameters
     *********************/

    public void strafeToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndContinue(target, driveParams);
    }



    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // MODIFIED    PURE PURSUIT
    //
    ////////////////////////////////////////////////////////////////////////
    // #####################################################################
    ////////////////////////////////////////////////////////////////////////
    //
    // These are high-level functions for driving to a specified pose
    // using only tank-drive primitives.
    //
    // Basically, you tell it what pose to go to and the robot goes
    // there using only driving forward (or backward if you tell it
    // to), and turning either in place or while it drives.  NO
    // STRAFING.
    //
    // In normal usage, you would tell it to go to a number of
    // waypoints in sequence without stopping, then stop at the end
    // when you get to the final destination.
    //
    // This makes for smooth, curling paths, and if the the waypoints
    // are chosen carefully, it can be noticeably faster than using
    // the equivalent Strafing function to get to the same point.  On
    // the other hand, strafing paths are easier to set up, so you may
    // not feel the need to use these functions.
    //
    // Note: These functions were originally developed for
    //   Centerstage. That year the RoBusted robot was a tank drive
    //   and hand no odometry wheels (!), so it was necessary to
    //   develop these more elaborate driving solutions in order to
    //   have a good auto.
    //
    ////////////////////////////////////


    ////////////////////////////////////
    //
    // PURE PURSUIT:   INNER LOOP CONTENTS (you probably don't call this directly)
    //
    ////////////////////////////////////

    /*********************
     * stepTowardPose
     *   - tells the motors what to do at this loop iteration in order to follow the path
     * .
     * Inputs:
     *   target     - target pose
     *   vCMax      - max allowed center-of-robot velocity
     *   vDiffMax   - max allowed left/right velocity difference
     *   alignmentThreshold  - if heading is too far off target, just turn robot
     *   direction  - try to match pose by driving forward or in reverse? (They are different!)
     * .
     * Returns:
     *   robotPoseInTargetPOV -- where the robot is from the point of
     *     view of the target.  This is returned as a convenience to
     *     the calling loop, as it may be helpful for deciding when to
     *     do things such as stop.
     * .
     * Notes:
     * .
     *   - Given current robot pose, determines how current motors
     *     should be set in order to eventually get to target,
     *     following a modified pure pursuit algorithm.
     * .
     *   - However, near the end of the path, achieving something
     *     close to the right heading is often more important than
     *     achieving the right (x,y) position.  Once robot is within
     *     alignmentThreshold of the target, extra weight is given to
     *     the heading and less to the (x,y) position.
     * .
     *   - In either case, this function just tells the robot what to
     *     do at this moment in time.  In order to actually follow the
     *     pure-pursuit path, you will need to call this function
     *     repeatedly in a loop and decide when to stop.
     *********************/

    public RoPose stepTowardPose(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double alignmentThreshold,
            DRIVE_DIRECTION direction) {
        updatePose();
        RoPose drivingPose = this.poseEst;
        if (direction == DRIVE_DIRECTION.REVERSE) {
            drivingPose = drivingPose.thenRotate(Math.PI);
            Log.d("stepTowardPose", "in reverse");
        } else {
            Log.d("stepTowardPose", "going forward");
        }
        // getting the target's pose in robot coordinates.
        RoPose relPose = drivingPose.getRelativeRoPoseFor(target);
        RoPose robotPoseInTargetPOV = relPose.inverse();
        // setting the coordinates, make typing easier
        double x = relPose.position.x;
        double y = relPose.position.y;
        double targetDist = Math.sqrt(x*x + y*y);
        double trackWidth = TRACK_WIDTH;
        double pointDist = Math.min(targetDist/2, targetDist-trackWidth);
        RoPose pointPose = relPose.thenForward(-pointDist);
        double pointX = pointPose.position.x;
        double pointY = pointPose.position.y;
        double vC = 0;
        double vDiff = 0;
        if (2*pointX<Math.abs(pointY)) {
            // turn
            vDiff = vDiffMax * Math.signum(pointY);
            String message = "Turning! ";
            message+=" poseEst: "+poseEst;
            message+=" relPose: "+relPose;
            message+=String.format(", (vC, vDiff)=(%.2f,%.2f)", vC, vDiff);
            Log.d("stepTowardPose", message);

        } else {
            // drive
            vC = vCMax;
            if (robotPoseInTargetPOV.position.x>-alignmentThreshold) {
                double headingScale = 7.5;
                vDiff = relPose.heading*vC*headingScale;
            } else {
                vDiff = (vC * 2 * pointY * trackWidth) / (pointX * pointX + pointY * pointY);
            }
            String message = "Driving! ";
            message+=" poseEst: "+poseEst;
            message+=" relPose: "+relPose;
            message+=String.format(", (vC, vDiff)=(%.2f,%.2f)", vC, vDiff);
            message+=String.format(", (pointDist, pointX, pointY)=(%.2f,%.2f,%.2f)", pointDist, pointX, pointY);
            Log.d("stepTowardPose", message);
        }
        if (direction == DRIVE_DIRECTION.FORWARD) {
            setForwardTurnDrivePower(vC, vDiff);
        }
        else{
            setForwardTurnDrivePower(-vC, vDiff);
        }

        reportPositionData();

        return robotPoseInTargetPOV;
    }


    ////////////////////////////////////
    //
    // PURE PURSUIT:   GO TO POSE AND STOP
    //
    ////////////////////////////////////


    /*********************
     * goToPoseAndStop
     *   - Sets up a loop to repeatedly call stepTowardPose
     *   - takes care of slowing down and stopping when the robot
     *      gets near the target
     * .
     * Inputs:
     *   target        - target pose
     *   vCMax         - max allowed center-of-robot velocity
     *   vDiffMax      - max allowed left/right velocity difference
     *   slowDistance  - distance to start slowing before stop
     *   threshold     - stop when robot gets this close
     *   direction     - try to match pose by driving forward or in reverse? (They are different!)
     * .
     *********************/

    public void goToPoseAndStop(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double slowDistance,
            double threshold,
            DRIVE_DIRECTION direction) {
        double vC = vCMax;
        while (opMode.opModeIsActive()) {
            RoPose robotPoseInTargetPOV = stepTowardPose(target, vC, vDiffMax, slowDistance/2, direction);
            double x = robotPoseInTargetPOV.position.x;
            double y = robotPoseInTargetPOV.position.y;
            double targetDist = Math.sqrt(x*x+y*y);
            double ratioSqrt = Math.sqrt(Math.abs(targetDist/slowDistance));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05/vCMax, 1);
            vC = vCMax * clippedSqrt;
            if (x>-threshold) {
                break;
            }
        }
        setLeftRightDrivePower(0,0);
    }

    /*********************
     * goToPoseAndStop
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target   - target pose
     *   params   - preset drive parameters
     *********************/

    public void goToPoseAndStop(
            RoPose target,
            DriveParameters driveParams) {
        goToPoseAndStop(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.slowDistance,
                driveParams.threshold,
                driveParams.direction);
    }

    /*********************
     * goToPoseAndStopD
     * .
     *   - Sets up a loop to repeatedly call stepTowardPose
     * .
     *   - takes care of slowing down and stopping when the robot gets
     *     near the target
     * .
     *   - this is basically the same as goToPoseAndStop, but angles
     *     are in DEGREES.  Since the Pose class uses radians, this
     *     ends up meaning that the target pose is specified as three
     *     parameters instead of packaged into a Pose.
     * .
     * Inputs:
     *   target_x        - x coordinate of target pose
     *   target_y        - y coordinate of target pose
     *   targetHeadingD  - target heading (in degrees)
     *   vCMax           - max allowed center-of-robot velocity
     *   vDiffMax        - max allowed left/right velocity difference
     *   slowDistance    - distance to start slowing before stop
     *   threshold       - stop when robot gets this close
     *   direction       - try to match pose by driving forward or in reverse? (They are different!)
     * .
     *********************/

    public void goToPoseAndStopD(
            double target_x,
            double target_y,
            double targetHeadingD,
            double vCMax,
            double vDiffMax,
            double slowDistance,
            double threshold,
            DRIVE_DIRECTION direction) {
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndStop(target, vCMax, vDiffMax, slowDistance, threshold, direction);
    }

    /*********************
     * goToPoseAndStopD
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target_x        - x coordinate of target pose
     *   target_y        - y coordinate of target pose
     *   targetHeadingD  - target heading (in degrees)
     *   params          - preset drive parameters
     *********************/

    public void goToPoseAndStopD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndStop(target, driveParams);
    }


    ////////////////////////////////////
    //
    // PURE PURSUIT:   GO TO POSE AND CONTINUE
    //
    ////////////////////////////////////


    /*********************
     * goToPoseAndContinue
     *   - Sets up a loop to repeatedly call stepTowardPose
     *   - Does *NOT* slow down and stop at the end
     * .
     * Inputs:
     *   target        - target pose
     *   vCMax         - max allowed center-of-robot velocity
     *   vDiffMax      - max allowed left/right velocity difference
     *   threshold     - stop when robot gets this close
     *   direction     - try to match pose by driving forward or in reverse? (They are different!)
     * .
     * Note: When robot gets near target this function simply returns
     *  with the robot still moving.  This assumes the next function
     *  in the auto sets a new target to drive to, and that eventually
     *  one of them will stop.
     *********************/

    public void goToPoseAndContinue(
            RoPose target,
            double vCMax,
            double vDiffMax,
            double threshold,
            DRIVE_DIRECTION direction ) {
        while (opMode.opModeIsActive()) { // to do!!! check if op mode is active.
            RoPose robotPoseInTargetPOV = stepTowardPose(target, vCMax, vDiffMax, threshold, direction);
            double x = robotPoseInTargetPOV.position.x;
            if (x>-threshold) {
                break;
            }
        }
    }

    /*********************
     * goToPoseAndContinue
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target  - target pose
     *   params  - preset drive parameters
     *********************/

    public void goToPoseAndContinue(
            RoPose target,
            DriveParameters driveParams) {
        goToPoseAndContinue(target,
                driveParams.vCMax,
                driveParams.vDiffMax,
                driveParams.threshold,
                driveParams.direction);
    }

    /*********************
     * goToPoseAndContinueD
     * .
     *   - Sets up a loop to repeatedly call stepTowardPose
     * .
     *   - Does *NOT* slow down and stop at the end
     * .
     *   - this is basically the same as goToPoseAndContinue, but
     *     angles are in DEGREES.  Since the Pose class uses radians,
     *     this ends up meaning that the target pose is specified as
     *     three parameters instead of packaged into a Pose.
     * .
     * Inputs:
     *   target_x        - x coordinate of target pose
     *   target_y        - y coordinate of target pose
     *   targetHeadingD  - target heading (in degrees)
     *   vCMax           - max allowed center-of-robot velocity
     *   vDiffMax        - max allowed left/right velocity difference
     *   threshold       - stop when robot gets this close
     *   direction       - try to match pose by driving forward or in reverse? (They are different!)
     * .
     * Note: When robot gets near target this function simply returns
     *  with the robot still moving.  This assumes the next function
     *  in the auto sets a new target to drive to, and that eventually
     *  one of them will stop.
     *********************/

    public void goToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            double vCMax,
            double vDiffMax,
            double threshold,
            DRIVE_DIRECTION direction) {
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndContinue(target, vCMax, vDiffMax, threshold, direction);
    }

    /*********************
     * goToPoseAndContinueD
     *   - same as above, but using DRIVE PARAMETERS
     * .
     * Inputs:
     *   target_x        - x coordinate of target pose
     *   target_y        - y coordinate of target pose
     *   targetHeadingD  - target heading (in degrees)
     *   params          - preset drive parameters
     *********************/

    public void goToPoseAndContinueD(
            double target_x,
            double target_y,
            double targetHeadingD,
            DriveParameters driveParams) {
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndContinue(target, driveParams);
    }

}
