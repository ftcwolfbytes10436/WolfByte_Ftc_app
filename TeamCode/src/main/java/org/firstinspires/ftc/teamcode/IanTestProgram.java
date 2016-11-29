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
        double time = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        double seconds = 15;
        while (opModeIsActive()) {
            robot.moveRobotToPositionUsingTime(4,3.5,.5,false,this);
            pressBeaconButton();
            while(opModeIsActive() && time < 50) {
                idle();
                time  = time + .01;
            }
            robot.currentPosition = new Position(DistanceUnit.METER,1,5,0,System.currentTimeMillis());
            robot.moveRobotToPositionUsingTime(1,4,.5,false,this);
            robot.moveRobotToPositionUsingTime(1,4.5,.5,false,this);
            robot.turnRobotToHeading(0,.2,this);
            robot.currentPosition = new Position(DistanceUnit.METER,1,5,0,System.currentTimeMillis());
            robot.moveRobotToPositionUsingTime(1.5,5,.5,false,this);
//            pressBeaconButton();
//            robot.currentPosition = new Position(DistanceUnit.METER,8,4.5,0,System.currentTimeMillis());
//            robot.moveRobotToPositionUsingTime(8,4,.5,false,this);
//            robot.turnRobotToHeading(0,.25,this);
//            robot.moveRobotToPositionUsingTime(3,4,.5,false,this);
//            robot.moveRobotToPositionUsingTime(3,2,.5,false,this);
//            robot.moveRobotToPositionUsingTime(0,0,.5,false,this);
            robot.moveRobot(0,0,0,telemetry);
            while(opModeIsActive() && !gamepad1.a) {
                idle();
            }
        }
    }

    double alliance = 1;
    static final double MINREDBLUEDIFF = 25;

    void pressBeaconButton() {
        robot.moveRobot(.1, 0, 0, telemetry); // start moving the robot to the right
        while (robot.getODSLightLevel() < .01 && opModeIsActive()) {
            idle();
        } // wait until the ods sensor sees white
        robot.moveRobot(0, 0, 0, telemetry);
        robot.turnRobotToHeading(0,.2,this);
        robot.moveRobot(.05, 0, 0, telemetry);
        int i = 0;
        while (robot.getODSLightLevel() >= .01 && opModeIsActive()) {
            i++;
            idle();
        }
        i = i / 2;
        robot.moveRobot(0, 0, 0, telemetry);
        robot.moveRobot(-.05, .1, 0, telemetry);
        while (robot.getODSLightLevel() < .01 && opModeIsActive()) {
            idle();
        } // wait until the ods sensor sees white
        robot.moveRobot(0, 0, 0, telemetry);
        robot.moveRobot(0, .1, 0, telemetry);
        while (!robot.touchSensor.isPressed()) {
            idle();
        }
        robot.moveRobot(0, 0, 0, telemetry);
        double diff = robot.sensorRGB.red() - robot.sensorRGB.blue();
        if (alliance == 1) {
            if (diff > MINREDBLUEDIFF) {
                //robot.pressLeftServo();
                telemetry.addData("Left servo", "pressed");
                telemetry.addData("Right servo", "not pressed");
                telemetry.update();
            } else {
                //robot.pressRightServo();
                telemetry.addData("Left servo", "not pressed");
                telemetry.addData("Right servo", "pressed");
                telemetry.update();
            }
        } else if (alliance == 2) {
            if (diff > MINREDBLUEDIFF) {
                //robot.pressRightServo();
                telemetry.addData("Left servo", "not pressed");
                telemetry.addData("Right servo", "pressed");
                telemetry.update();
            } else {
                //robot.pressLeftServo();
                telemetry.addData("Left servo", "pressed");
                telemetry.addData("Right servo", "not pressed");
                telemetry.update();
            }
        }
    }
}
