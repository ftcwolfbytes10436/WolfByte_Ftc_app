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

package org.firstinspires.ftc.teamcode; //original

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;    //original
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;    //original
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;  //original



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Teleop
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 *program id:AndrewDriveProgram
 *original code = "0"
 *date Modified: 11/12/16 = "1"
 *detail: creating a simple program
 *
*/

@Autonomous(name="Andrew Drive Program", group="BetaLykosTest") //original // @TeleOp(...) is the other common choice
@Disabled   //original
public class AndrewDriveProgram extends LinearOpMode {  //original

    /* Declare OpMode members. */
    BetaLykosHardware robot           = new BetaLykosHardware();   // Use betaLykos' hardware   //original

    @Override   //original
    public void runOpMode() throws InterruptedException {   //original

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);    //original

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized"); //original
        telemetry.update(); //original

        // Wait for the game to start (driver presses PLAY)
        waitForStart(); //original

        while (opModeIsActive()) {  //original

            // Send telemetry message to signify robot running;
            telemetry.addData("Status", "Running"); //original

            float gamepad1RightY = -gamepad1.right_stick_y; //original
            float gamepad1RightX = -gamepad1.right_stick_x; //original
            float gamepad1LeftX  = -gamepad1.left_stick_x;  //original
            float gamepad1LeftY  = -gamepad1.left_stick_y;  //original

            robot.moveRobot(gamepad1RightX,gamepad1RightY,gamepad1LeftX,telemetry); //original

            telemetry.update(); //original

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);  //original
            idle(); //original
        }
    }
}
