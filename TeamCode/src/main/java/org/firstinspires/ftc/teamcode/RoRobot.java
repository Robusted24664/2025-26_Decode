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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

public class RoRobot {
    public RoRobot(){
        //this constructor does nothing
    }
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontRight = null;
    public DcMotor flywheel = null;
    public DcMotor intakeFront = null;
    public DcMotor intakeLift = null;
    
    
    public Servo gate = null;
    public Servo kicker = null;
    
    public GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    
    // Robot position information
    public RoPose poseEst = RoPose.getTrivial();
    public static boolean isPoseBaseInitialized = false;
    public static RoPose poseBase = RoPose.getTrivial();
    public static double TRACK_WIDTH = 14.50; //inches


    

   
    
    
 
   


    public static LinearOpMode opMode;
    
    //driving parameters 
    public static final double TURBO_OVERSHOOT = 10;
    
    public enum DRIVE_DIRECTION {
        FORWARD,
        REVERSE
    }
    
    public static class DriveParameters{
        public double vCMax = 0.5;
        public double vDiffMax = 0.4;
        public double slowDistance = 15;
        public double slowAngleD = 10;
        public double threshold = 2;
        public double thresholdD = 2;
        public DRIVE_DIRECTION direction = DRIVE_DIRECTION.FORWARD;
    }
    
    
    public void initializeGeneric(HardwareMap hardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        // Initialize the hardware variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");
        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");
        
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        // Stop and reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //initialize the odometry pods
        odo.setOffsets(-85.0, -140.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        
        // Set motor direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    
        //Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); 
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    public void initializeAuto(HardwareMap hardwareMap, LinearOpMode opMode){
        initializeGeneric(hardwareMap, opMode);
        odo.resetPosAndIMU();
    }
    
    /*
    public void zeroArm(){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    */
    
    public void initializeTeleOp(HardwareMap hardwareMap, LinearOpMode opMode){
        initializeGeneric(hardwareMap, opMode);
    }
    
    public void setRobotDrivePower(double leftPower, double rightPower){
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);
    }
    public void strafeDrive(double drive, double strafe, double turn){
        double backLeftPower     = drive + turn + strafe;
        double backRightPower    = drive - turn - strafe;
        double frontLeftPower    = drive + turn - strafe;
        double frontRightPower   = drive - turn + strafe;
        // back seems a little slow
        backRightPower*= 1.1;
        backLeftPower*= 1.1;
        // find max absolute value
        double maxAbsoluteValue = Math.max(
            Math.max(
                Math.abs(backLeftPower),
                Math.abs(backRightPower)
            ),
            Math.max(
                Math.abs(frontLeftPower),
                Math.abs(frontRightPower)
            )
        );
        double multiplier = maxAbsoluteValue > 1.0 ? 1.0/maxAbsoluteValue : 1.0;
        backLeft.setPower(backLeftPower*multiplier);
        backRight.setPower(backRightPower*multiplier);
        frontLeft.setPower(frontLeftPower*multiplier);
        frontRight.setPower(frontRightPower*multiplier);
    }
    
    /*
                robot.strafeDrive(drive, strafe, turn);
            double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
*/
    
    /* setMotorPowers 
    Inputs:
     vC    - requested forward velocity at center of robot
     vDiff - requested velocity difference between the two wheels
    Notes: 
    - velocities are given in the range (-1,1) as per FTC motor power parameter
    - after converting into left and right velocities, this function checks to 
      see if either is outside the range (-1,1).  If so, then it *scales* the
      velocities so that the bigger one is still in range.  If you just *clipped*
      the velocities, then the rate of turning would change!
    */
    public void setMotorPowers(double vC, double vDiff){
        double vR = vC+(vDiff/2);
        double vL = vC-(vDiff/2);
        double vRAbs = Math.abs(vR);
        double vLAbs = Math.abs(vL);
        double vAbsMax = Math.max(vRAbs,vLAbs);
        if (vAbsMax > 1){
            // if one of them >1, then we have to scale it down
            vR=vR/vAbsMax;
            vL=vL/vAbsMax;
        }
        setRobotDrivePower(-vL, -vR);
    }
    
    /* Dont need for decode
    public void setArmPosition(double power, int position){
        if (opMode.opModeIsActive()){
            arm.setPower(power);
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    */
    
    /* Dont need for decode
    public void waitForArm(int threshold) {
        while (opMode.opModeIsActive()) {
            int delta = arm.getCurrentPosition() - arm.getTargetPosition();
            if (Math.abs(delta) < threshold){
                break;
            } else {
                opMode.sleep(50);
            }
        }
    }
    */
    
    /* Dont need for decode
    public void setWristPosition(double power, int position){
        if (opMode.opModeIsActive()){
            wristLeft.setPower(power);
            wristRight.setPower(power);
            wristLeft.setTargetPosition(position);
            wristRight.setTargetPosition(position);
            wristLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wristRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    
    public void openClaw(){
        claw.setPosition(CLAW_OPEN_POSITION);
    }
    public void closeClaw(){
        claw.setPosition(CLAW_CLOSED_POSITION);
    }
    public void closeSampleSideways(){
        claw.setPosition(CLAW_SIDEWAYS_POSITION);
    }
    //12/6/2024 lucas added
    public void wideOpenClaw(){
        claw.setPosition(0.35);
    }
    
    public void setIntakePower(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(-power);
    }
    
    public void setIntakePosition(double intakePos){
        intakePos = Range.clip(intakePos, INTAKE_MIN, INTAKE_MAX);
        intakePosition.setPosition(intakePos);
    }
    
    public void setIntakeRotationPower(double power){
        intakeRotation.setPower(power);
    }
    */
    
    public void setPoseD(double x, double y, double headingD){
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
    
    public void updatePose(){
        odo.update();
        Pose2D goBildaPos = odo.getPosition();
        double x=goBildaPos.getX(DistanceUnit.INCH);
        double y=goBildaPos.getY(DistanceUnit.INCH);
        double heading=goBildaPos.getHeading(AngleUnit.RADIANS);
        RoPose relPoseEst = new RoPose(x, y, heading);
        poseEst = poseBase.then(relPoseEst);
    }
    
    /* turnToHeading
    Inputs:
     heading   - target heading (in radians)
     vDiffMax  - largest vDiff to allow
     threshold - stop once robot heading is this close
     slowAngle - start slowing down once robot heading is this close
     timeoutS - timeout in seconds
    Notes:
    - after slowAngle is met, robot attempts to make constant deceleration slowing.
      The goal is to make a smooth stop without over-turning.
    */
    public void turnToHeading(double heading, double vDiffMax, double slowAngle, double threshold, double timeoutS){
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutS){
            opMode.telemetry.addData("time", timer.seconds());
            opMode.telemetry.update();
            
            updatePose();
            double dHeading = RoPose.normalizeAngle(heading - poseEst.heading);
            Log.d("stepTowardPose", "target heading: " + heading);
            Log.d("stepTowardPose", "pose heading: " + poseEst.heading);
            Log.d("stepTowardPose", "dHeading: " + dHeading);
            if (Math.abs(dHeading) < threshold){
                    Log.d("stepTowardPose", "turn complete");
                    break;
            }
            double ratioSqrt = Math.sqrt(Math.abs(dHeading/slowAngle));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05/vDiffMax, 1);
            double vDiffMin = 0.4; //value of 0.3 was too small and the robot got stuck one turn
            double vDiff = vDiffMax * clippedSqrt * Math.signum(dHeading);
            if (Math.abs(vDiff)<vDiffMin){
                vDiff = vDiffMin*Math.signum(dHeading);
            }
            String message = "Turning! ";
            message+=" poseEst: "+poseEst;
            message+=String.format(", (vDiff)=(%.2f)", vDiff);
            message+=String.format(", (dHeading, ratioSqrt, clippedSqrt)=(%.2f,%.2f,%.2f)", dHeading, ratioSqrt, clippedSqrt);
            Log.d("stepTowardPose", message);
            setMotorPowers(0, vDiff);
            
            reportPositionData();
        }
    }
    
    public void turnToHeading(double heading, double vDiffMax, double slowAngle, double threshold){
        double timeoutS = 90; //this is an excessively long timeout (in seconds)
        turnToHeading(heading, vDiffMax, slowAngle, threshold, timeoutS);
    }
    
    /* turnToHeadingD
    This is essentially the same as turnToHeading, but all angles are specified in degrees.
    */
    public void turnToHeadingD(double headingD, double vDiffMax, double slowAngleD, double thresholdD){
        double heading = headingD * Math.PI/180;
        double slowAngle = slowAngleD * Math.PI/180;
        double threshold = thresholdD * Math.PI/180;
        turnToHeading(heading, vDiffMax, slowAngle, threshold);
    }
    public void turnToHeadingD(double headingD, DriveParameters params){
        turnToHeadingD(headingD, params.vDiffMax, params.slowAngleD, params.thresholdD);
    }
    
    /* turnToHeadingDAndStop
    Turn to a heading, then stop the robot.
    Angles are given in degrees.
    */
    public void turnToHeadingDAndStop(double headingD, double vDiffMax, double slowAngleD, double thresholdD){
        turnToHeadingD(headingD, vDiffMax, slowAngleD, thresholdD);
        setRobotDrivePower(0,0);
    }
    public void turnToHeadingDAndStop(double headingD, DriveParameters params){
        turnToHeadingD(headingD, params);
        setRobotDrivePower(0,0);
    }

    /* turnToHeadingAndStop
    Turn to a heading, then stop the robot.
    Angles are given in radians.
    */
    public void turnToHeadingAndStop(double heading, double vDiffMax, double slowAngle, double threshold){
        turnToHeading(heading, vDiffMax, slowAngle, threshold);
        setRobotDrivePower(0,0);
    }
    
    /* waitUntilTimeWithUpdates
    Input:
     timer   – ElapsedTime object from the opmode
     endtime - time to end the wait
    Notes:
    - Safely waits until the timer reaches endtime.
    - Performs odometry updates while we wait so that odometry accuracy doesn't suffer.
    */
    // public void waitUntilTimeWithUpdates(ElapsedTime timer, double endtime){
    //     while (timer.seconds()<endtime&&opMode.opModeIsActive()){
    //         telemetry.addData("current time: ",timer.seconds());
    //         telemetry.update();
    //         updatePose();
    //     }
    // }
    
    /* waitWithUpdates
    Input:
     timer     – ElapsedTime object from the opmode
     secToWait - number of seconds to wait
    Notes:
    - Safely waits until the number of seconds has passed
    - Performs odometry updates while we wait so that odometry accuracy doesn't suffer.
    */
    // public void waitWithUpdates(ElapsedTime timer, double secToWait){
    //     double endTime = timer.seconds()+secToWait;
    //     waitUntilTimeWithUpdates(timer, endTime);
    // }
    
    
    /* stepTowardPose
    Inputs:
     target     - target pose
     vCMax      - max allowed center-of-robot velocity
     vDiffMax   - max allowed left/right velocity difference
     alignmentThreshold  - if heading is too far off target, just turn robot
     direction  - try to match pose by driving forward or in reverse? (They are different!)
    Notes:
    - Given current robot pose, determines how current motors should be set 
      in order to eventually get to target, following a modified pure pursuit
      algorithm.
    - However, near the end of the path, achieving something close to the right heading is
      often more important than achieving the right (x,y) position.  Once robot is within
      alignmentThreshold of the target, extra weight is given to the heading and less to 
      the (x,y) position.
    - In either case, this function just tells the robot what to do at this moment in time.
      In order to actually follow the pure-pursuit path, you will need to call this function
      repeatedly in a loop and decide when to stop.
    */
    public RoPose stepTowardPose(
                RoPose target,
                double vCMax,
                double vDiffMax,
                double alignmentThreshold,
                DRIVE_DIRECTION direction ) {
        updatePose();
        RoPose drivingPose = this.poseEst;
        if (direction == DRIVE_DIRECTION.REVERSE){
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
        double heading = relPose.heading; // heading in radians
        double targetDist = Math.sqrt(x*x + y*y);
        double trackWidth = TRACK_WIDTH;
        double pointDist = Math.min(targetDist/2, targetDist-trackWidth);
        RoPose pointPose = relPose.thenForward(-pointDist);
        double pointX = pointPose.position.x;
        double pointY = pointPose.position.y;
        double vC = 0;
        double vDiff = 0;
        if (2*pointX<Math.abs(pointY)){
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
        if (direction == DRIVE_DIRECTION.FORWARD){
            setMotorPowers(vC, vDiff);
        }
        else{
            setMotorPowers(-vC, vDiff);
        }
        
        reportPositionData();
        
        return robotPoseInTargetPOV;
    }
    
    public RoPose stepStrafeTowardPose(RoPose target,
                double vCMax,
                double vDiffMax,
                double alignmentThreshold){
        updatePose();
        RoPose drivingPose = this.poseEst;
        // getting the target's pose in robot coordinates.
        RoPose relPose = drivingPose.getRelativeRoPoseFor(target);
        RoPose robotPoseInTargetPOV = relPose.inverse();
        // setting the coordinates, make typing easier
        double x = relPose.position.x;
        double y = relPose.position.y;
        double heading = relPose.heading; // heading in radians
        double trackWidth = TRACK_WIDTH;
        
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
        Log.d("stepTowardPose", message);
        
        strafeDrive(-vX, -vY, vDiff);
        
        reportPositionData();
        
        return robotPoseInTargetPOV;
    }
    public void stepStrafeTowardPose(RoPose target, DriveParameters params){
        stepStrafeTowardPose(target, params.vCMax, params.vDiffMax, params.threshold);
    }
    
    public void strafeToPoseAndStop(
                RoPose target,
                double vCMax /* drive speed */,
                double vDiffMax,
                double slowDistance,  // distance to start slowing before stop
                double threshold){
        double vC = vCMax;
        while (opMode.opModeIsActive()){
            RoPose robotPoseInTargetPOV = stepStrafeTowardPose(target, vC, vDiffMax, slowDistance/2);
            double x = robotPoseInTargetPOV.position.x;
            double y = robotPoseInTargetPOV.position.y;
            double tenDegrees = 10*Math.PI/180;
            double weight = 10/tenDegrees; //this is the weight that makes 10 deg = 1 inch
            double relLength = robotPoseInTargetPOV.length(weight);
            double ratioSqrt = Math.sqrt(Math.abs(relLength/slowDistance));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05/vCMax, 1);
            vC = vCMax * clippedSqrt;
            if (relLength<threshold){
                break;
            }
        }
        setRobotDrivePower(0,0);
    }
    public void strafeToPoseAndStop(RoPose target, DriveParameters driveParams){
        strafeToPoseAndStop(target, 
            driveParams.vCMax, 
            driveParams.vDiffMax, 
            driveParams.slowDistance,
            driveParams.threshold);
    }
    
    public void strafeToPoseAndContinue(
                RoPose target,
                double vCMax, /* drive speed */
                double vDiffMax, /* turn speed */
                double threshold) {
        while (opMode.opModeIsActive()){ // to do!!! check if op mode is active.
            RoPose robotPoseInTargetPOV = stepStrafeTowardPose(target, vCMax, vDiffMax, threshold);
            double tenDegrees = 10*Math.PI/180;
            double weight = 10/tenDegrees; //this is the weight that makes 10 deg = 1 inch
            double relLength = robotPoseInTargetPOV.length(weight);
            if (relLength<threshold){
                break;
            }
        }
    }
    public void strafeToPoseAndContinue(RoPose target, DriveParameters driveParams){
        strafeToPoseAndContinue(target, 
            driveParams.vCMax, 
            driveParams.vDiffMax, 
            driveParams.threshold);
    }
    
    public void strafeToPoseAndContinueD(
                double target_x, double target_y, double targetHeadingD,
                double vCMax /* drive speed */,
                double vDiffMax /* turn speed */,
                double threshold,
                DRIVE_DIRECTION direction){
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndContinue(target, vCMax, vDiffMax, threshold);
    }
    public void strafeToPoseAndContinueD(double target_x, double target_y, double targetHeadingD, DriveParameters driveParams){
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        strafeToPoseAndContinue(target, driveParams);
    }
    
    /* goToPoseAndStop
    Repeatedly call stepTowardPose inside of a loop that will slow down and stop
    when the robot gets near the target
    */
    public void goToPoseAndStop(
                RoPose target,
                double vCMax /* drive speed */,
                double vDiffMax /* turn speed */,
                double slowDistance,  // distance to start slowing before stop
                double threshold,
                DRIVE_DIRECTION direction){
        double vC = vCMax;
        while (opMode.opModeIsActive()){
            RoPose robotPoseInTargetPOV = stepTowardPose(target, vC, vDiffMax, slowDistance/2, direction);
            double x = robotPoseInTargetPOV.position.x;
            double y = robotPoseInTargetPOV.position.y;
            double targetDist = Math.sqrt(x*x+y*y);
            double ratioSqrt = Math.sqrt(Math.abs(targetDist/slowDistance));
            double clippedSqrt = Range.clip(ratioSqrt, 0.05/vCMax, 1);
            vC = vCMax * clippedSqrt;
            if (x>-threshold){
                break;
            }
        }
        setRobotDrivePower(0,0);
    }
    public void goToPoseAndStop(RoPose target, DriveParameters driveParams){
        goToPoseAndStop(target, 
            driveParams.vCMax, 
            driveParams.vDiffMax, 
            driveParams.slowDistance,
            driveParams.threshold, 
            driveParams.direction);
    }
    
    /*
    goToPoseAndStopD is goToPoseAndStop but angles are specified in degrees
    */
    public void goToPoseAndStopD(
                double target_x, double target_y, double targetHeadingD,
                double vCMax /* drive speed */,
                double vDiffMax /* turn speed */,
                double slowDistance,  // distance to start slowing before stop
                double threshold,
                DRIVE_DIRECTION direction){
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndStop(target, vCMax, vDiffMax, slowDistance, threshold, direction);
    }
    public void goToPoseAndStopD(double target_x, double target_y, double targetHeadingD, DriveParameters driveParams){
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndStop(target, driveParams);
    }
    
    /* goToPoseAndContinue
    Repeatedly call stepTowardPose inside of a loop that will *not* slow down and stop.
    Instead, when robot gets near target this function simply returns with the robot 
    still moving.  This assumes the next function in the auto sets a new target to 
    drive to, and that eventually one of them will stop.
    */
    public void goToPoseAndContinue(
                RoPose target,
                double vCMax, /* drive speed */
                double vDiffMax, /* turn speed */
                double threshold,
                DRIVE_DIRECTION direction ) {
        while (opMode.opModeIsActive()){ // to do!!! check if op mode is active.
            RoPose robotPoseInTargetPOV = stepTowardPose(target, vCMax, vDiffMax, threshold, direction);
            double x = robotPoseInTargetPOV.position.x;
            if (x>-threshold){
                break;
            }
        }
    }
    public void goToPoseAndContinue(RoPose target, DriveParameters driveParams){
        goToPoseAndContinue(target, 
            driveParams.vCMax, 
            driveParams.vDiffMax, 
            driveParams.threshold, 
            driveParams.direction);
    }
    
    public void goToPoseAndContinueD(
                double target_x, double target_y, double targetHeadingD,
                double vCMax /* drive speed */,
                double vDiffMax /* turn speed */,
                double threshold,
                DRIVE_DIRECTION direction){
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndContinue(target, vCMax, vDiffMax, threshold, direction);
    }
    public void goToPoseAndContinueD(double target_x, double target_y, double targetHeadingD, DriveParameters driveParams){
        RoPose target =  RoPose.RoPoseD(target_x, target_y, targetHeadingD);
        goToPoseAndContinue(target, driveParams);
    }
    
    
    public void reportPositionData(){
        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
         */
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", poseEst.position.x, poseEst.position.y, poseEst.heading*180/Math.PI);
        opMode.telemetry.addData("Position", data);
        /*
        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
         */
        /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        */
        opMode.telemetry.addData("Status", odo.getDeviceStatus());

        opMode.telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        opMode.telemetry.update();
    }
    
}
