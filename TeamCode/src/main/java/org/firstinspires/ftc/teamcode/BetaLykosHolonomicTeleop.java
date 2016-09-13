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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

@TeleOp(name="BetaLykos: Holonomic Tele", group="BetaLykos")
//@Disabled
public class BetaLykosHolonomicTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    BetaLykosHardware robot           = new BetaLykosHardware();   // Use betaLykos' hardware

    @Override
    public void runOpMode() throws InterruptedException {
        double fLeft;
        double fRight;
        double bLeft;
        double bRight;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            float gamepad1RightY = -gamepad1.right_stick_y;
            float gamepad1RightX = -gamepad1.right_stick_x;
            float gamepad1LeftX  = -gamepad1.left_stick_x;

            fLeft  = Range.clip(gamepad1RightY + gamepad1RightX + gamepad1LeftX,-1,1);
            fRight = Range.clip(gamepad1RightY - gamepad1RightX - gamepad1LeftX,-1,1);
            bLeft  = Range.clip(gamepad1RightY - gamepad1RightX + gamepad1LeftX,-1,1);
            bRight = Range.clip(gamepad1RightY + gamepad1RightX - gamepad1LeftX,-1,1);

            robot.frontLeftMotor.setPower(fLeft);
            robot.frontRightMotor.setPower(fRight);
            robot.backLeftMotor.setPower(bLeft);
            robot.backRightMotor.setPower(bRight);

            // Send telemetry message to signify robot running;
            telemetry.addData("front left",  "%.2f", fLeft);
            telemetry.addData("front right", "%.2f", fRight);
            telemetry.addData("back left",  "%.2f", bLeft);
            telemetry.addData("back right", "%.2f", bRight);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle();
        }
    }
}
