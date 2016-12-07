package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ProgramId: CalebTestProgram
 * Creater: caleb
 * CreationDate: 11.12.16
 *
 * Modifications:
 *  Date:
 *  Comment code:
 *  Name:
 *  Details:
 */

@Autonomous(name="Caleb test program", group="BetaLykos")
@Disabled
public class CalebTestProgram1 extends LinearOpMode {

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

        double seconds = .25;
        while (opModeIsActive()) {
            timer.reset();
            robot.moveRobotForSeconds((float)0,1f,0,this,seconds);
//            robot.moveRobotForSeconds(0,(float)0.5,0,this,.571);
//            robot.moveRobotToPositionUsingTime(1,1,.5,false,this);
            telemetry.addData("time",timer.seconds());
            telemetry.update();
            while (!gamepad1.a && opModeIsActive()) {idle();}
            seconds += .25;
        }
    }
}
