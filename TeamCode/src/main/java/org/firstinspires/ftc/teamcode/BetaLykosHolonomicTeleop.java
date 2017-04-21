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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common BetaLykos hardware class to define the devices on the robot.
 * All device access is managed through the BetaLykosHardware class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a holonomic Game style Teleop for a holonomic drive
 * In this mode the left stick moves the robot's position, the Right stick turns left and right.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Holonomic TeleOp", group="BetaLykos")
//@Disabled
public class BetaLykosHolonomicTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    BetaLykosHardware robot           = new BetaLykosHardware();   // Use betaLykos' hardware

    @Override
    public void runOpMode() throws InterruptedException {


        boolean lowGear = false;
        boolean runParticleLauncher = false;
        boolean particleLauncherButtonHeld = false;
        boolean railsUnlocked = false;
        boolean unlockRailButtonHeld = false;
        boolean launching = false;
        boolean resettingLauncher = false;
        double resetLoopCounter = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Send telemetry message to signify robot running;
            telemetry.addData("Status", "Running");

            if (gamepad1.a) {
                lowGear = true;
            } else if (gamepad1.b && lowGear) {
                lowGear = false;
            }

            float gamepad1RightY = -gamepad1.right_stick_y;
            // modified the lower 2 lines by removing the - from ...X = -gamepad1...;
            float gamepad1RightX = gamepad1.right_stick_x;
            float gamepad1LeftX  = gamepad1.left_stick_x;
            float extendRobot = gamepad1.right_trigger - gamepad1.left_trigger;

            if (lowGear) {
                gamepad1RightX /= 2;
                gamepad1RightY /= 2;
                gamepad1LeftX /= 4;
            }

            double ca = Math.cos(Math.toRadians(-robot.getHeading()));
            double sa = Math.sin(Math.toRadians(-robot.getHeading()));
            double finalX = ca * gamepad1RightX - sa * gamepad1RightY;
            double finalY = sa * gamepad1RightX + ca * gamepad1RightY;

            finalX *= finalX * finalX;
            finalY *= finalY * finalY;

//            if (robot.getFrontRangeDistance() <= 6 && gamepad1RightY > 0) {
//                gamepad1RightY = 0;
//            }
            robot.moveRobot(finalX,finalY,gamepad1LeftX,extendRobot,telemetry);
            telemetry.addData("White Sensor" , robot.getODSLightLevel());
            // unlock or lock rails
            if (gamepad1.left_bumper && !unlockRailButtonHeld) {
                railsUnlocked = !railsUnlocked;
                unlockRailButtonHeld = true;
                robot.lockUnlockRails(railsUnlocked);

            } else if (!gamepad1.left_bumper && unlockRailButtonHeld) {
                unlockRailButtonHeld = false;
            }
            telemetry.addData("Rails unlocked",railsUnlocked);

            if (gamepad2.left_bumper && !launching && !resettingLauncher) {
                telemetry.addData("Launching","Launching");
                robot.particleLauncher.setPower(1);
                launching = true;
            }


            if (launching && gamepad2.y) {
                telemetry.addData("Waiting for switch", "Reloading");
                resettingLauncher = true;
                launching = false;

            }

            if (resettingLauncher) {
                telemetry.addData("ResetLooper Inc", "Inc resetLoopCounter");
                resetLoopCounter ++;
            }

            if (resetLoopCounter >= 12) {
                telemetry.addData("Reseting", "Stopping Moter");
                robot.particleLauncher.setPower(0);
                resettingLauncher=false;
                resetLoopCounter = 0;
            }

            if (gamepad2.right_bumper)
            {
                robot.particleLauncher.setPower(-1);
                resetLoopCounter = 0;
                resettingLauncher = false;
                launching = false;
            }

            if (gamepad2.x)
            {
                robot.pressLeftServo();
            }

            if (gamepad2.b)
            {
                robot.pressRightServo();
            }

            if (gamepad2.y) {
                robot.pushServosDown();
            }



            telemetry.addData("Launcher Switch", robot.launcherLimitSwitch.getState());
            telemetry.addData("Launching", launching);
            telemetry.addData("ResettingLauncher", resettingLauncher);
            telemetry.addData("Reseting Loop Counter", resetLoopCounter);


            //telemetry.update();
//            if (gamepad2.left_bumper) {
//                robot.particleLauncher.setPower(1);
//            } else {
//                robot.particleLauncher.setPower(0);
//            }
//
//            if (robot.launcherLimitSwitch.getState()) {
//                robot.particleLauncher.setPower(0);
//            } else {
//                robot.particleLauncher.setPower(1);
//            }

            double scoopServo = gamepad2.right_stick_y;

            telemetry.addData("ScoopStick", scoopServo);
            telemetry.addData("ScoopSwitch", robot.scoopTouchSensor.getState());
            robot.scoopServo.setPower(Range.clip(scoopServo+robot.scoopServoCalibration,-1,1));
//            if (((scoopServo > 0.05) && !robot.scoopTouchSensor.getState()) || scoopServo < -0.05)
//            {
//                robot.scoopServo.setPower(scoopServo);
//            }
//            else
//            {
//                robot.waitForTick(11);
//                robot.scoopServo.setPower(robot.scoopServoCalibration);
//            }

            telemetry.addData("Run Particle Launcher", runParticleLauncher);

            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle();
        }
    }
}
