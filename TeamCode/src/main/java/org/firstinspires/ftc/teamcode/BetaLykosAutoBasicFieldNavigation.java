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
    static final Position[] redStartPositions = {new Position(DistanceUnit.METER,0,0,0,0),new Position(DistanceUnit.METER,0,0,0,0)};
    static final Position[] blueStartPositions = {new Position(DistanceUnit.METER,0,0,0,0),new Position(DistanceUnit.METER,0,0,0,0)};


    int alliance = 0;
    int startPosition = 0;
    int[] questions = new int[5];
    int question8 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

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
            if (startPosition == 1){
                robot.currentPosition = blueStartPositions[0];
            } else if (startPosition == 2){
                robot.currentPosition = blueStartPositions[1];
            }
            for (int i = 0; i < questions.length; i++) {
                switch (questions[i]) {

                    case 1:
                        //becon 1
                        break;
                    case 2:
                        //becon 2
                        break;
                    case 3:
                        //push ball
                        break;
                    case 4:
                        //shoot
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

    }
    void pushBeaconButton() throws InterruptedException {
        robot.moveRobotFeetRelitive(1,0,1,this);
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
               // robot.pressLeftServo();
                telemetry.addData("Left servo","pressed");
                telemetry.addData("Right servo","not pressed");
                telemetry.update();
            } else {
               // robot.pressRightServo();
                telemetry.addData("Left servo","not pressed");
                telemetry.addData("Right servo","pressed");
                telemetry.update();
            }
        } else if (alliance == 2) {
            if (diff > MINREDBLUEDIFF) {
              //  robot.pressRightServo();
                telemetry.addData("Left servo","not pressed");
                telemetry.addData("Right servo","pressed");
                telemetry.update();
            } else {
               // robot.pressLeftServo();
                telemetry.addData("Left servo","pressed");
                telemetry.addData("Right servo","not pressed");
                telemetry.update();
            }
        }
    }
}


