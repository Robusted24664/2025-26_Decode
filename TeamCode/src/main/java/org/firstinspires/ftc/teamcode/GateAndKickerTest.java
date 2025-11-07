package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Gate and Kicker Test", group="Testing")
public class GateAndKickerTest extends LinearOpMode {

    // Declare OpMode members.
    private Servo gate = null;
    private Servo kicker = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        gate = hardwareMap.get(Servo.class, "gate");
        kicker = hardwareMap.get(Servo.class, "kicker");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double gatePos = 0.5;
        double kickerPos = 0.5;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            gatePos += gamepad1.left_stick_y / 2000;
            kickerPos += gamepad1.right_stick_y / 2000;

            
            gate.setPosition(gatePos);
            kicker.setPosition(kickerPos);

            
            
            
            // Send telemetry data to the driver station
            telemetry.addData("Gate Position", gatePos);
            telemetry.addData("Kicker Position", kickerPos);

            telemetry.update();
        }
    }
}
