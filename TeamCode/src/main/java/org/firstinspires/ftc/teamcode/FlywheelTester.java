package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "Test: Flywheel", group = "Testing")

public class FlywheelTester extends OpMode {
    // This declares the four motors needed
    DcMotorEx flywheel;
    DcMotor intakeLift;
    private Servo gate = null;
    private Servo kicker = null;


    public static final double GATE_OPEN_POSITION = 0.5785;
    public static final double GATE_CLOSED_POSITION = 0.4783;
    public static final double GATE_CLOSE_DELAY = 0.25; //seconds
    double time_when_pressed = 0;
    public enum GateState {
        BASE,
        WAIT_FOR_GATE,
        WAIT_FOR_KICKER,
    }
    GateState gateState = GateState.BASE;
    public static boolean isBallInIntake = false;
    
    
    public static final int BALL_INTO_INTAKE_DELTA = -300; 
    public static final int BALL_OUT_OF_INTAKE_DELTA = 550; 
    public static final double KICKER_TOP_POSITION = 0.2484;
    public static final double KICKER_BOTTOM_POSITION = 0.68;
    public static final double KICKER_CLOSE_DELAY = 0.75; //seconds

    
    void kickerUp() {
        kicker.setPosition(KICKER_TOP_POSITION);
   }
    void kickerDown() {
        kicker.setPosition(KICKER_BOTTOM_POSITION);
    }
    void gateOpen() {
        gate.setPosition(GATE_OPEN_POSITION);
    }
    void gateClose() {
        gate.setPosition(GATE_CLOSED_POSITION);
    }
    void moveIntake(int delta) {
        int liftPosition = intakeLift.getCurrentPosition();
        int liftTarget = liftPosition + delta;
        intakeLift.setTargetPosition(liftTarget);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLift.setPower(0.5);
    }
     
    void putBallInIntake() {
        moveIntake(BALL_INTO_INTAKE_DELTA);
    }
    void putBallInIntake() {
        moveIntake(BALL_INTO_INTAKE_DELTA);
    }



    // This is state for the flywheel
    double flywheel_speed = 0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");
        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");

        gateClose();
        kickerDown();
        
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);
    
        telemetry.addLine("Successful Initialization");

    }

    @Override
    public void loop() {
        telemetry.addLine("This is the flywheel tester");
        telemetry.addLine("Use right joystick to speed up or slow down the flywheel");
        telemetry.addLine("Press A to set the speed to 0");

        // If you press the A button, then you reset the speed to be zero
        if (gamepad1.a) {
            flywheel_speed = 0;
        }
        flywheel_speed += -gamepad1.right_stick_y/1000;
        flywheel_speed = Range.clip(flywheel_speed, -1,1);
        flywheel.setPower(flywheel_speed);
        
        double flywheel_velocity = flywheel.getVelocity();
        
        if (gamepad1.leftBumperWasPressed()) {
            isBallInIntake = true;
            int liftPosition = intakeLift.getCurrentPosition();
            int liftTarget = liftPosition + BALL_INTO_INTAKE_DELTA;
            intakeLift.setTargetPosition(liftTarget);
            intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeLift.setPower(0.5);
        }
        
        if (gateState == GateState.BASE) {
            if (gamepad1.rightBumperWasPressed()) {
                //Now we will transition to the next state
                time_when_pressed = getRuntime();
                gateOpen();
                gateState = GateState.WAIT_FOR_GATE;
            } 
        } else if (gateState == GateState.WAIT_FOR_GATE) {
            if (getRuntime() > time_when_pressed + GATE_CLOSE_DELAY) {
                gateClose();
                kickerUp();
                gateState = GateState.WAIT_FOR_KICKER;
            }
        } else if (gateState == GateState.WAIT_FOR_KICKER) {
            if (getRuntime() > time_when_pressed + KICKER_CLOSE_DELAY) {
                kickerDown();
                gateState = GateState.BASE;
            }
        }
        
        telemetry.addData("Flywheel Velocity:", flywheel_velocity);
        telemetry.addData("Flywheel Power:", flywheel_speed);
     
    }
}
