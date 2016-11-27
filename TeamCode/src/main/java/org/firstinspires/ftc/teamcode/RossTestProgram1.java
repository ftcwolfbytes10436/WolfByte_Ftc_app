package org.firstinspires.ftc.teamcode;

/**
 * Created by RossL on 11/23/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


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
            robot.moveRobotToPositionUsingTime(4,3.5,.5,false,this);
            robot.moveRobot(.1,0,0,telemetry); // start moving the robot to the right
            while (robot.getODSLightLevel() < .01 && opModeIsActive())  {
                idle();} // wait until the ods sensor sees white
            robot.moveRobot(0,0,0,telemetry);
            robot.moveRobot(.05,0,0,telemetry);
            int i = 0;
            while (robot.getODSLightLevel() >= .01 && opModeIsActive()) {
                i++;
                idle();
            }
            i=i/2;
            robot.moveRobot(0,0,0,telemetry);
            robot.moveRobot(-.05,.1,0,telemetry);
            while (robot.getODSLightLevel() < .01 && opModeIsActive())  {
                idle();} // wait until the ods sensor sees white
            robot.moveRobot(0,0,0,telemetry);
            robot.moveRobot(0,.1,0,telemetry);
            while (!robot.touchSensor.isPressed()) {
                idle();
            }
            robot.moveRobot(0,0,0,telemetry);
            robot.currentPosition = new Position(DistanceUnit.METER,5.3,5.1,0,System.currentTimeMillis());
            robot.moveRobotToPositionUsingTime(3,3,.5,false,this);
            robot.turnRobotToHeading(-45,.25,this);
            /*while (i >= 0) {
                i--;
                idle();
            }

            robot.moveRobot(0,0,0,telemetry);
            */
            //robot.moveRobotToPositionUsingTime(2,4,.5,false,this);
            //robot.moveRobotForSeconds((float).2,0,0,this,1);
            //Position position = getPositionfromRangeSensor();

            //robot.moveRobotToPositionUsingTime(5,7,.5,false,this);
            //robot.moveRobotToPositionUsingTime(2,0,.5,false,this);
            //robot.moveRobotToPositionUsingTime(0,0,.5,false,this);
            telemetry.addData("time",timer.seconds());
//            telemetry.update();
            while (!gamepad1.a && opModeIsActive()) {
                idle();
            }
        }
    }
}
