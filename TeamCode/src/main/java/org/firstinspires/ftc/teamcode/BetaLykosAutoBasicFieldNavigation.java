/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * This Opmode is a basic autonomous mode that can navigate the field using the accelerometer and
 * distance sensors
 *
 * This particular OpMode just executes a basic Teleop
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic Field Navigation", group="BetaLykos")
public class BetaLykosAutoBasicFieldNavigation extends LinearOpMode {

    /* Declare OpMode members. */
    BetaLykosHardware robot           = new BetaLykosHardware();   // Use betaLykos' hardware

    static final double MINREDBLUEDIFF = 25;
    static final double distanceBetweenButtons = 1;
    static final Position[] redStartPositions = {new Position(DistanceUnit.METER,5,1,0,0), new Position(DistanceUnit.METER,7,1,0,0)};
    static final Position[] redBeacon1 = {new Position(DistanceUnit.METER,2,4.5,0,0), new Position(DistanceUnit.METER,1,5.5,0,0), new Position(DistanceUnit.METER,2,5.5,0,0)};
    static final Position[] redBeacon2 = {new Position(DistanceUnit.METER,2,8,0,0), new Position(DistanceUnit.METER,1,9,0,0), new Position(DistanceUnit.METER,2,9,0,0)};
    static final Position[] redShootingPositions = {new Position(DistanceUnit.METER,3,7,0,0)};
    static final Position[] blueStartPositions = {new Position(DistanceUnit.METER,11,7,0,0), new Position(DistanceUnit.METER,11,5,0,0)};
    static final Position[] blueBeacon1 = {new Position(DistanceUnit.METER,6,10,0,0), new Position(DistanceUnit.METER,7,11,0,0)};
    static final Position[] blueBeacon2 = {new Position(DistanceUnit.METER,2,10,0,0), new Position(DistanceUnit.METER,3,11,0,0)};
    static final Position[] blueShootingPositions = {new Position(DistanceUnit.METER,9,5,0,0)};
    static final int[] startHeading = {90,0};



    int alliance = 0;
    int startPosition = 0;
    int[] questions = new int[5];
    int question8 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.scoopServo.setPower(-.05);

        robot.onRedAlliance = gamepad1.a;
        robot.useDistanceSensorForInitialPosition = true;

        String[] options = {"beacon 1", "beacon 2", "push ball", "shoot", "cornerV", "centerV", "stay"};


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        while (question8 != 1) {
            telemetry.addData("Alliance", "b = red   x = blue");
            telemetry.update();
            alliance = 0;
            while (alliance == 0) {
                if (gamepad1.b) {
                    alliance = 1;
                    telemetry.addData("Alliance", "Red");
                } else if (gamepad1.x) {
                    alliance = 2;
                    telemetry.addData("Alliance", "Blue");
                }
                idle();
            }
            waitForNoButton();

            telemetry.addData("Start Position", "a = start position 1 b = start position 2");
            telemetry.update();
            startPosition = getInput(2);

            telemetry.addData("Move 1", "g1.a = beacon 1   g1.b = beacon 2   g1.x = push ball g1.y = shoot g2.a = cornerV g2.b = centerV g2.x = stay");
            telemetry.update();
            questions[0] = getInput(7);

            telemetry.addData("Move 2", "g1.a = beacon 1   g1.b = beacon 2   g1.x = push ball g1.y = shoot g2.a = cornerV g2.b = centerV g2.x = stay");
            telemetry.update();
            questions[1] = getInput(7);

            telemetry.addData("Move 3", "g1.a = beacon 1   g1.b = beacon 2   g1.x = push ball g1.y = shoot g2.a = cornerV g2.b = centerV g2.x = stay");
            telemetry.update();
            questions[2] = getInput(7);

            telemetry.addData("Move 4", "g1.a = beacon 1   g1.b = beacon 2   g1.x = push ball g1.y = shoot g2.a = cornerV g2.b = centerV g2.x = stay");
            telemetry.update();
            questions[3] = getInput(7);

            telemetry.addData("Move 5", "g1.a = beacon 1   g1.b = beacon 2   g1.x = push ball g1.y = shoot g2.a = cornerV g2.b = centerV g2.x = stay");
            telemetry.update();
            questions[4] = getInput(7);

            switch (alliance) {
                case 1:
                    telemetry.addData("Alliance", "Red");
                    break;
                case 2:
                    telemetry.addData("Alliance", "Blue");
                    break;
            }
            telemetry.addData("Start Position", startPosition);
            telemetry.addData("Move 1", options[questions[0] - 1]);
            telemetry.addData("Move 2", options[questions[1] - 1]);
            telemetry.addData("Move 3", options[questions[2] - 1]);
            telemetry.addData("Move 4", options[questions[3] - 1]);
            telemetry.addData("Move 5", options[questions[4] - 1]);
            telemetry.addData("Is this correct?", "a = yes b = no");

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            question8 = getInput(2);
        }

        if (alliance == 1) {
            robot.headingOffset = 90;
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//
//        while (opModeIsActive()) {
//            while (!gamepad1.a) {
//                idle();
//            }
//            while (gamepad1.a){
//                idle();
//            }
//            pressBeaconButton();
//        }
        if (alliance == 1) {
            if (startPosition == 1){
                robot.currentPosition = redStartPositions[0];
            } else if (startPosition == 2){
                robot.currentPosition = redStartPositions[1];
            }
        } else if (alliance == 2) {
            if (startPosition == 1) {
                robot.currentPosition = blueStartPositions[0];
            } else if (startPosition == 2) {
                robot.currentPosition = blueStartPositions[1];
            }
        }
        for (int i = 0; i < questions.length; i++) {
            switch (questions[i]) {

                case 1:
                    beacon1();
                    break;
                case 2:
                    beacon2();
                    break;
                case 3:
                    //push ball
                    break;
                case 4:
                    shoot();
                    break;
                case 5:
                    //cornerV
                    break;
                case 6:
                    //centerV
                    break;
                case 7:
                    //stay
                    break;
            }
        }
    }

    void beacon1() throws InterruptedException {
        if (alliance == 1) {
            robot.moveRobotToPositionUsingTime(redBeacon1[0].x,redBeacon1[0].y,0.5,false,this);
            lineUpToBeacon();
            robot.currentPosition = redBeacon1[1];
            pressBeaconButton();
            robot.moveRobotToPositionUsingTime(redBeacon1[2].x,redBeacon1[2].y,0.5,false,this);
        } else {
            robot.moveRobotToPositionUsingTime(blueBeacon1[0].x,redBeacon1[0].y,0.5,false,this);
            lineUpToBeacon();
            robot.currentPosition = blueBeacon1[1];
            pressBeaconButton();
            robot.moveRobotToPositionUsingTime(blueBeacon1[2].x,blueBeacon1[2].y,0.5,false,this);
        }
    }

    void beacon2() throws InterruptedException {
        if (alliance == 1) {
            robot.moveRobotToPositionUsingTime(redBeacon2[0].x,redBeacon2[0].y,0.5,false,this);
            lineUpToBeacon();
            robot.currentPosition = redBeacon2[1];
            pressBeaconButton();
            robot.moveRobotToPositionUsingTime(redBeacon2[2].x,redBeacon2[2].y,0.5,false,this);
        } else {
            robot.moveRobotToPositionUsingTime(blueBeacon2[0].x,redBeacon2[0].y,0.5,false,this);
            lineUpToBeacon();
            robot.currentPosition = blueBeacon2[1];
            pressBeaconButton();
            robot.moveRobotToPositionUsingTime(blueBeacon2[0].x,blueBeacon2[0].y,0.5,false,this);
        }
    }

    void lineUpToBeacon() throws InterruptedException {
        robot.moveRobot(.1,0,0,telemetry); // start moving the robot to the right
        while (robot.getODSLightLevel() < .005 && opModeIsActive())  {
            idle();} // wait until the ods sensor sees white
        robot.moveRobot(0,0,0,telemetry);
        robot.turnRobotToHeading(startHeading[alliance - 1],.2,this);
        robot.moveRobot(.05,0,0,telemetry);
        while (robot.getODSLightLevel() >= .005 && opModeIsActive()) {
            idle();
        }
        robot.moveRobot(0,0,0,telemetry);
        robot.moveRobot(-.05,.1,0,telemetry);
        while (robot.getODSLightLevel() < .005 && opModeIsActive())  {
            idle();} // wait until the ods sensor sees white
        robot.moveRobot(0,0,0,telemetry);
        robot.moveRobot(0,.1,0,telemetry);
        while (!robot.touchSensor.isPressed()) {
            idle();
        }
        robot.moveRobot(0,0,0,telemetry);
    }

    void shoot() throws InterruptedException {
        if(alliance == 1) {
            robot.moveRobotToPositionUsingTime(redShootingPositions[0].x, redShootingPositions[0].y,0.5,false,this);
            robot.turnRobotTowardsPoint(redShootingPositions[0].x, redShootingPositions[0].y,.25,this);
        } else {
            robot.moveRobotToPositionUsingTime(blueShootingPositions[0].x, blueShootingPositions[0].y,0.5,false,this);
            robot.turnRobotTowardsPoint(blueShootingPositions[0].x, blueShootingPositions[0].y,.25,this);
        }


        //Start the launcher motor (shoots the 1st ball)
        robot.particleLauncher.setPower(1);

        //keep the motor running until the switch turns on
        while( !robot.launcherLimitSwitch.getState()) {
            robot.waitForTick(10);
            idle();
        }

        //now run the motor .6s to get it ready to fire again
        robot.waitForTick(600);

        //stop the motor
        robot.particleLauncher.setPower(0);



        robot.scoopServo.setPower(1);
        if (!(robot.scoopTouchSensor.getState())) {
            robot.scoopServo.setPower(1);
        } else {
            robot.scoopServo.setPower(-0.05);
        }

        robot.waitForTick(500);
        robot.particleLauncher.setPower(1);

        //keep the motor running until the switch turns on
        while( !robot.launcherLimitSwitch.getState()) {
            robot.waitForTick(10);
            idle();
        }

        //now run the motor .6s to get it ready to fire again
        robot.waitForTick(600);

        //stop the motor
        robot.particleLauncher.setPower(0);


//       if (!robot.launcherLimitSwitch.getState()) {
//          while (!robot.launcherLimitSwitch.getState()) {
//          robot.particleLauncher.setPower(1);
//          }
//            robot.particleLauncher.setPower(0);
//        } else {
//            while (robot.launcherLimitSwitch.getState()) {
//                robot.particleLauncher.setPower(1);
//            }
//            while (!robot.launcherLimitSwitch.getState()) {
//                robot.particleLauncher.setPower(1);
//            }
//            robot.particleLauncher.setPower(0);
//       }

    }

    int getInput(int numAnswers) {
        int input = 0;
        while (input == 0) {
            if (gamepad1.a) {
                input = 1;
            } else if (gamepad1.b) {
                input = 2;
            } else if (gamepad1.x && numAnswers >= 3) {
                input = 3;
            } else if (gamepad1.y && numAnswers >= 4) {
                input = 4;
            } else if (gamepad2.a && numAnswers >= 5) {
                input = 5;
            } else if (gamepad2.b && numAnswers >= 6) {
                input = 6;
            } else if (gamepad2.x && numAnswers >= 7) {
                input = 7;
            }
            idle();
        }
        waitForNoButton();
        return input;
    }

    void waitForNoButton() {
        while (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y||gamepad2.a||gamepad2.b||gamepad2.x) {
            idle();
        }
    }

    void pressBeaconButton() {
        double diff = robot.sensorRGB.red() - robot.sensorRGB.blue();
        if (alliance == 1) {
            if (diff > MINREDBLUEDIFF) {
                robot.pressLeftServo();
                telemetry.addData("Left servo","pressed");
                telemetry.addData("Right servo","not pressed");
                telemetry.update();
            } else {
                robot.pressRightServo();
                telemetry.addData("Left servo","not pressed");
                telemetry.addData("Right servo","pressed");
                telemetry.update();
            }
        } else if (alliance == 2) {
            if (diff > MINREDBLUEDIFF) {
                robot.pressRightServo();
                telemetry.addData("Left servo","not pressed");
                telemetry.addData("Right servo","pressed");
                telemetry.update();
            } else {
                robot.pressLeftServo();
                telemetry.addData("Left servo","pressed");
                telemetry.addData("Right servo","not pressed");
                telemetry.update();
            }
        }
    }
}


