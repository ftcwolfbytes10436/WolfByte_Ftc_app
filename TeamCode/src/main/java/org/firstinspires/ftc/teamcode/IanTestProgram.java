package org.firstinspires.ftc.teamcode;

/**
 * Created by IanL on 11/26/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@Autonomous(name="Ian test program", group="BetaLykos")
public class IanTestProgram extends LinearOpMode {

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
            robot.moveTwoRobotToPositionUsingTime(-7,0,.5,false,this);
        }
    }
}
