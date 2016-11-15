package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;    //original
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;    //original
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;  //original

/**
 * Created by Andrew on 11/12/16.
 *
 * ProgramID: AndrewTestProgram1
 * Date Modified: 11/12/16 tag = "//orig" (original due to it being the first day created
 *  Details: Making a simple test program.
 *
 * Date Modified:
 *  Details:
 *
 */
@Autonomous(name="Andrew Test Program", group="BetaLykos") //orig // @TeleOp(...) is the other common choice

public class AndrewTestProgram1 extends LinearOpMode {  //orig


    BetaLykosHardware robot = new BetaLykosHardware();  //orig

    @Override   //orig
    public void runOpMode() throws InterruptedException    //orig
    {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            robot.moveRobot(.2,0, 0, telemetry);

//            robot.moveRobotForSeconds(0, 0.4f, 0, this, (float)myTime);
            while (!gamepad1.a && opModeIsActive() && robot.getODSLightLevel() == 0)
            {
                idle();
            }

            robot.moveRobot(0,0, 0, telemetry);

//            myTime++;
        }



        robot.moveRobotForSeconds(0, 1, 0, this, 3);      //orig
    }
}
