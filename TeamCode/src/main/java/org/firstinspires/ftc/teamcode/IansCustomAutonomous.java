package org.firstinspires.ftc.teamcode;

/**
 * Created by IanL on 11/26/2016.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@Autonomous(name="Ian's Custom Autonomous Mode", group="BetaLykos")
public class IansCustomAutonomous extends LinearOpMode {

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
            robot.moveRobotToPositionUsingTime(4, 3.5, .5, false, this);
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(4,3.5)");
//            telemetry.update();
//            while (opModeIsActive() && !gamepad1.a) {
//                idle();
//            }

            pressBeaconButton();
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(beacon button pressed)");
//            telemetry.update();
  /*          while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            while(opModeIsActive() && time < 50) {
                idle();
                time  = time + .01;
            }
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(wait)");
//            telemetry.update();
  /*          while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            robot.currentPosition = new Position(DistanceUnit.METER,1,5,0,System.currentTimeMillis());
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(1,5)");
//            telemetry.update();
  /*          while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            robot.moveRobotToPositionUsingTime(1,4,.5,false,this);//back up
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(1,4.5)");
//            telemetry.update();
  /*          while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            robot.moveRobotToPositionUsingTime(1,4.4,.5,false,this);//go forward
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(1,4.3)");
//            telemetry.update();
  /*          while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            robot.turnRobotToHeading(0,.2,this);//recheck headding
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(check heading)");
//            telemetry.update();
            /*while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            robot.currentPosition = new Position(DistanceUnit.METER,0,5,0,System.currentTimeMillis());//say you are at beacon 1
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(0,5)");
//            telemetry.update();
            /*while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            robot.moveRobotToPositionUsingTime(3.5,5,.5,false,this);
            //this is the real code -  robot.moveRobotToPositionUsingTime(3,5,.5,false,this);//move to beacon 2 left of the tape
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(3,5)");
//            telemetry.update();
            /*while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
//            robot.moveRobot(0,0,0,telemetry);
//            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(stop)");
//            telemetry.update();
    /*        while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
//            while(opModeIsActive() && time < 100) {
//                idle();
//                time  = time + .01;
//            }

//            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(wait)");
//            telemetry.update();
  /*          while (opModeIsActive() && !gamepad1.a) {
                idle();
            }
*/
            pressBeaconButton();
            robot.moveRobot(0,0,0,telemetry);
//            telemetry.addData("Position","(press beacon 2)");
//            telemetry.update();
//            while (opModeIsActive() && !gamepad1.a) {
//                idle();
//            }


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
        while (robot.getODSLightLevel() < .005 && opModeIsActive()) {
            idle();
        } // wait until the ods sensor sees white
        robot.moveRobot(0, 0, 0, telemetry);
        robot.turnRobotToHeading(0,.2,this);
        robot.moveRobot(.05, 0, 0, telemetry);
        int i = 0;
        while (robot.getODSLightLevel() >= .005 && opModeIsActive()) {
            i++;
            idle();
        }
        i = i / 2;
        robot.moveRobot(0, 0, 0, telemetry);
        robot.moveRobot(-.05, .1, 0, telemetry);
        while (robot.getODSLightLevel() < .005 && opModeIsActive()) {
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
