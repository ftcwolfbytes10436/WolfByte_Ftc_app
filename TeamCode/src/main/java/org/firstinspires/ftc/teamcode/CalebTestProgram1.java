package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class CalebTestProgram1 extends LinearOpMode {

    BetaLykosHardware robot = new BetaLykosHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        int myTime = 1;
//        while (opModeIsActive()) {
//            robot.moveRobotForSeconds(0, 0.4f, 0, this, (float)myTime);
//            while (!gamepad1.a && opModeIsActive()) {idle();}
//            myTime++;
//        }
        for (int i = 0; i < 2; i++) {
            robot.moveRobotForSeconds(0, 0.5f, 0, this, 1.2f);
            robot.moveRobotForSeconds(0,0,0,this,0.5f);
            robot.moveRobotForSeconds(0.5f, 0, 0, this, 1.2f);
            robot.moveRobotForSeconds(0,0,0,this,0.5f);
            robot.moveRobotForSeconds(-0.5f, 0, 0, this, 1.2f);
            robot.moveRobotForSeconds(0,0,0,this,0.5f);
            robot.moveRobotForSeconds(0, -0.5f, 0, this, 1.2f);
            robot.moveRobotForSeconds(0,0,0,this,0.5f);
        }
    }
}
