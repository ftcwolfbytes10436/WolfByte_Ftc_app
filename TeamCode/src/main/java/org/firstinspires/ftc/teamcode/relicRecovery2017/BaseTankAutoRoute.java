package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by RPS on 10/6/17.
 */

@Autonomous(name="tank autonomous routes")
public class BaseTankAutoRoute extends LinearOpMode {

    BaseTankAuto robot = new BaseTankAuto();
    Telemetry.Item pathTelemetry = null;
    Telemetry.Item targetTelemetry = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry, this);

        waitForStart();
        try
        {
            robot.setGriperPos(1);
            /*
            robot.jewelHit.setPosition(.3);
            sleep(250);
            for(int i = 0; i < .25; i += .002)
            {
                robot.jewelRaise.setPosition(i);
                sleep(1);
            }
            robot.jewelRaise.setPosition(0.255);
            //robot.jewelRaise.setPosition(.25);
            //find color

            sleep(5000);
            robot.displayColorTelementy();

            //robot.hitLeft();
            //sleep(500);
            //robot.hitRight();
            //hit it
            sleep(500);
            robot.jewelRaise.setPosition(1);
            sleep(500);
            robot.jewelHit.setPosition(1);
            sleep(10000);
            */
            sleep(500);
            robot.LifterMotor.setPower(-0.3);
            sleep(1000);
            robot.LifterMotor.setPower(-0.1);
            robot.moveForInches(35, .25);
            while (opModeIsActive())sleep(1);
        }
        catch (Exception e)
        {
            telemetry.addData("error", e.toString());
            telemetry.update();
            while (opModeIsActive())sleep(1);
        }

    }

}
