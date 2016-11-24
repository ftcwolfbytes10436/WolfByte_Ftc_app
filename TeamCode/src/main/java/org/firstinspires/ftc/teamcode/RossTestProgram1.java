package org.firstinspires.ftc.teamcode;

/**
 * Created by RossL on 11/23/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name="Ross test program", group="BetaLykos")
public class RossTestProgram1 extends LinearOpMode {

    BetaLykosHardware robot = new BetaLykosHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        double sum = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double seconds = 15;
        while (opModeIsActive()) {
            timer.reset();
            //robot.moveRobotForSeconds(0,(float)0.5,0,this,.571);
            //robot.moveRobotToPositionUsingTime(0,2,.5,false,this);
            robot.moveRobotToPositionUsingTime(-7,0,.5,false,this);
            //robot.moveRobotToPositionUsingTime(2,0,.5,false,this);
            //robot.moveRobotToPositionUsingTime(0,0,.5,false,this);
            telemetry.addData("time",timer.seconds());
            telemetry.update();
            while (!gamepad1.a && opModeIsActive()) {
                idle();
            }
            robot.moveRobotToPositionUsingTime(0,0,.5,false,this);
        }
    }
}
