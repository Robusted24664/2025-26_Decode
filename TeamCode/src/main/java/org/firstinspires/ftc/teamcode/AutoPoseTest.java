package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.RoRobot.DRIVE_DIRECTION;
import org.firstinspires.ftc.teamcode.RoRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.RoRobot.DRIVE_DIRECTION;
import org.firstinspires.ftc.teamcode.RoRobot.DriveParameters;

//@Disabled

@Autonomous(name="AutoPoseTest", group="test")
public class AutoPoseTest extends LinearOpMode {
    public RoRobot robot = new RoRobot();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeAuto(hardwareMap, (LinearOpMode)this);
        RoPose target = null;
        
        DriveParameters slowForward = new DriveParameters();
        slowForward.vCMax = 0.5;
        slowForward.vDiffMax = 0.4;
        slowForward.slowDistance = 5;
        slowForward.threshold = 2;
        slowForward.direction = DRIVE_DIRECTION.FORWARD;
        
        DriveParameters slowReverse = new DriveParameters();
        slowReverse.vCMax = 0.5;
        slowReverse.vDiffMax = 0.4;
        slowReverse.slowDistance = 5;
        slowReverse.threshold = 2;
        slowReverse.direction = DRIVE_DIRECTION.REVERSE;
        
        DriveParameters slowTurnParameters = new DriveParameters();
        slowTurnParameters.vDiffMax = 0.5;
        slowTurnParameters.slowAngleD = 10;
        slowTurnParameters.thresholdD = 0.5;
        
        DriveParameters turnParameters = new DriveParameters();
        turnParameters.vDiffMax = 1;
        turnParameters.slowAngleD = 10;
        turnParameters.thresholdD = 0.5;
        
        DriveParameters fastTurnParameters = new DriveParameters();
        fastTurnParameters.vDiffMax = 1.5;
        fastTurnParameters.slowAngleD = 45;
        fastTurnParameters.thresholdD = 2;
        
        waitForStart();
        
        double vCMax = 0.5;
        double vDiffMax = 0.4;
        double slowDistance=15;
        double threshold=2;
        
        target = RoPose.RoPoseD(10, 20, 45);
        // vCMax
        // double vDiffMax,
        // double slowDistance, 
        // double threshold
        robot.strafeToPoseAndContinue(target, slowForward);
        target = RoPose.RoPoseD(30, 20, 45);
        robot.strafeToPoseAndContinue(target, slowForward);
        target = RoPose.RoPoseD(40, 20, 0);
        robot.strafeToPoseAndStop(target, slowForward);
        
        // robot.goToPoseAndStop(target, slowForward);
        // telemetry.addData("Status", "done going forward");
        // telemetry.update();
        // sleep(2000);
        
        // target = RoPose.RoPoseD(10, 0, 180);
        // robot.goToPoseAndStop(target, slowReverse);
        

        
        // robot.turnToHeadingDAndStop(90, slowTurnParameters);
        // robot.turnToHeadingDAndStop(170, slowTurnParameters);
        // robot.turnToHeadingDAndStop(0, slowTurnParameters);
        
        // telemetry.addData("Status", "Done");
        // telemetry.update();
        
        while (opModeIsActive()){
            robot.reportPositionData();
        }
        
    }
}
