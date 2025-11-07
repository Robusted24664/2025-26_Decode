package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "New Intake Tester", group = "Test")
public class NewIntakeTester extends LinearOpMode{
    public void runOpMode() {
        //seconds you want the servo to go from 0 to 1
        double SECONDSFORONESERVOROTATION = 1.5;
        
        
        // Wait for the game to start (driver presses PLAY)
        CRServo intakeRotation = hardwareMap.get(CRServo.class, "intakeRotation");
        Servo intakePosition = hardwareMap.get(Servo.class, "intakePosition");
        double intakePos = 0.5;
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();
        // Run until the end of the match (driver presses STOP)
        double timeNow = runtime.seconds();
        while (opModeIsActive()) {
            double timeLast = timeNow;
            timeNow = runtime.seconds();
            double lastLoopDuration = timeLast-timeNow;
            
            intakePos += gamepad1.left_stick_y/SECONDSFORONESERVOROTATION*lastLoopDuration;
            intakePos = Range.clip(intakePos, 0, 1);
            
            double intakePower = gamepad1.right_stick_y;
            
            intakeRotation.setPower(intakePower);
            intakePosition.setPosition(intakePos);
            telemetry.addData("IntakePos", intakePos);
            telemetry.addData("IntakePower", intakePower);
            telemetry.update();
        }    

    }
}