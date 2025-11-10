package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "Test: Flywheel", group = "Testing")

public class FlywheelTester extends OpMode {

    ////////////////////////////////////
    // Hardware interface
    ////////////////////////////////////

    public DcMotorEx flywheel = null;
    public DcMotor intakeLift = null;
    public Servo gate = null;
    public Servo kicker = null;

    ////////////////////////////////////
    // Important Constants
    ////////////////////////////////////

    public static final double GATE_OPEN_POSITION = 0.5785;
    public static final double GATE_CLOSED_POSITION = 0.4783;
    public static final double GATE_CLOSE_DELAY = 0.25; //seconds

    public static final double KICKER_TOP_POSITION = 0.2484;
    public static final double KICKER_BOTTOM_POSITION = 0.68;
    public static final double KICKER_CLOSE_DELAY = 0.75; //seconds

    public static final int BALL_INTO_INTAKE_DELTA = -300;
    public static final int BALL_OUT_OF_INTAKE_DELTA = 550;

    ////////////////////////////////////
    // State info
    ////////////////////////////////////

    public enum GateState {
        BASE,
        WAIT_FOR_GATE,
        WAIT_FOR_KICKER,
    }
    GateState gateState = GateState.BASE;

    public double time_when_pressed = 0;
    public boolean isBallInIntake = false;
    public double flywheel_speed = 0;

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
    // UTILITY: Ball in and out of intake
    ////////////////////////////////////

    public void moveIntake(int delta) {
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int liftPosition = intakeLift.getCurrentPosition();
        int liftTarget = liftPosition + delta;
        intakeLift.setTargetPosition(liftTarget);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLift.setPower(0.5);
    }
    public void putBallIntoIntake() {
        moveIntake(BALL_INTO_INTAKE_DELTA);
        isBallInIntake = true;
    }
    public void pullBallOutOfIntake() {
        moveIntake(BALL_OUT_OF_INTAKE_DELTA);
        isBallInIntake = false;
    }


    ////////////////////////////////////
    // OPMODE: init
    ////////////////////////////////////

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

    ////////////////////////////////////
    // OPMODE: loop
    ////////////////////////////////////

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

        ////////////////////////////////////
        // Shooting Process

        if (gamepad1.leftBumperWasPressed()) {
            putBallIntoIntake();
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
                if (isBallInIntake == true) {
                    pullBallOutOfIntake();
                    gateState = GateState.BASE;
                } else {
                    kickerUp();
                    gateState = GateState.WAIT_FOR_KICKER;
                }
            }
        } else if (gateState == GateState.WAIT_FOR_KICKER) {
            if (getRuntime() > time_when_pressed + KICKER_CLOSE_DELAY) {
                kickerDown();
                gateState = GateState.BASE;
            }
        }

        ////////////////////////////////////
        // Telemetry Update

        telemetry.addData("Flywheel Velocity:", flywheel_velocity);
        telemetry.addData("Flywheel Power:", flywheel_speed);

    }
}