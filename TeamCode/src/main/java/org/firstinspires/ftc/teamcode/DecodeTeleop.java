package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@TeleOp(name = "Decode Teleop", group = "AAFirst Group")

public class DecodeTeleop extends OpMode {

    ////////////////////////////////////
    // Hardware interface
    ////////////////////////////////////

    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor intakeFront = null;
    public DcMotor intakeLift = null;
    public Servo gate = null;
    public Servo kicker = null;
    public DcMotorEx flywheel = null;

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

    public static final double SHOOTING_POWER = 0.498;

    ////////////////////////////////////
    // State info
    ////////////////////////////////////

    public double flapperPower = 0;
    public boolean intakeAutoIn = false;

    public enum GateState {
        BASE,
        WAIT_FOR_GATE,
        WAIT_FOR_KICKER,
    }
    GateState gateState = GateState.BASE;

    public double time_when_pressed = 0;
    public boolean isBallInIntake = false;

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
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");

        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");

        gateClose();
        kickerDown();

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeFront.setDirection(DcMotor.Direction.FORWARD);
        intakeLift.setDirection(DcMotor.Direction.FORWARD);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    ////////////////////////////////////
    // OPMODE: Start
    ////////////////////////////////////

    @Override
    public void start() {
        flywheel.setPower(SHOOTING_POWER);
    }

    ////////////////////////////////////
    // OPMODE: loop
    ////////////////////////////////////

    @Override
    public void loop() {
        telemetry.addLine("Right trigger for intake");
        telemetry.addLine("Left trigger for reverse intake");
        telemetry.addLine("Right bumper to stop intake");

        ////////////////////////////////////
        // Intake

        if (gamepad1.right_trigger > 0.5) {
            // Intake is locked in the forward position
            flapperPower = 1;
            intakeAutoIn = true;
            intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.left_trigger > 0.3){
            // Intake is now controlled by the left trigger
            intakeAutoIn = false;
            intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (intakeAutoIn == false) {
            // Intake power equals how much left trigger is pressed down
            flapperPower = -gamepad1.left_trigger;
        }
        intakeFront.setPower(flapperPower);
        intakeLift.setPower(flapperPower * 0.5);

        ////////////////////////////////////
        // Flywheel On/Off

        if (gamepad1.dpad_up) {
            flywheel.setPower(SHOOTING_POWER);
        }
        if (gamepad1.dpad_down) {
            flywheel.setPower(0);
        }

        ////////////////////////////////////
        // Shooting Process

        /*
        nathan says we dont need the chamber
        if (gamepad1.leftBumperWasPressed()) {
            putBallIntoIntake();
        }
        */


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

        // Driving
        drive(-gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                gamepad1.left_stick_button);

    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate, boolean turbo) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower  = - forward + right - rotate;
        double frontRightPower = + forward - right - rotate;
        double backRightPower  = + forward + right - rotate;
        double backLeftPower   = - forward - right - rotate;

        double maxPower = 1.0;
        double maxSpeed = (turbo) ? 1.0 : 0.65;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}