package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name = "Motor Test", group = "Test")

public class MotorTest extends LinearOpMode {
    
    DcMotorEx motorExFlywheel;
    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0.5;

    public void runOpMode() {
        motorExFlywheel = (DcMotorEx)hardwareMap.get(DcMotor.class, "flywheel");
        
        waitForStart();
        
        PIDFCoefficients pidfOrig = motorExFlywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P,NEW_I, NEW_D, NEW_F);
        motorExFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        
        PIDFCoefficients pidfModified = motorExFlywheel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while(opModeIsActive()) {
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
            pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
            pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.update();
       

        }
    }
}
