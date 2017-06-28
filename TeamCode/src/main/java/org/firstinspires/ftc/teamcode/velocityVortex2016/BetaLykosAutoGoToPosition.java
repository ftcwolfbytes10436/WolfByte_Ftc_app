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
package org.firstinspires.ftc.teamcode.velocityVortex2016;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Teleop
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 */

@Autonomous(name="Go to position", group="BetaLykos")
@Disabled
public class BetaLykosAutoGoToPosition extends LinearOpMode {

    /* Declare OpMode members. */
    BetaLykosHardware robot           = new BetaLykosHardware();   // Use betaLykos' hardware

    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Position targets[] = new Position[10];
        int selectedElement = 0;
        float xPosition = 0 ;
        float yPosition = 0;
        float power = 0;
        boolean buttonPressed = false;

        for (int i = 0; i < targets.length; i++) {
            targets[i] = new Position(DistanceUnit.METER, 0, 0, 0, 0);
        }

        while (opModeIsActive()) {

            Position position = new Position(DistanceUnit.METER,xPosition,yPosition,power,0);
            targets[selectedElement] = position;

            // Send telemetry message to signify robot running;
            telemetry.addData("Status", "Running"+selectedElement);
            for (int i = 0; i < targets.length; i++) {
                if (targets[i] != null) {

                    if (selectedElement == i) {
                        telemetry.addData("Target " + selectedElement + 1, "Position: ( %.2f, %.2f)     Power: %.2f  <----Selected"
                                , targets[i].x, targets[i].y ,targets[i].z);

                    } else {
                        telemetry.addData("Target " + selectedElement + 1, "Position: ( %.2f, %.2f)     Power: %.2f "
                                , targets[i].x, targets[i].y ,targets[i].z);
                    }
                }
            }
            telemetry.update();

            if (gamepad1.a) {
                for (Position target : targets) {
                    if (target != null) {
                        Boolean succeeded = robot.moveRobotToPosition(target.x, target.y, target.z, false, this);
                        if (!succeeded) {
                            break;
                        }
                    }
                }
                for (int i = 0; i < targets.length; i++) {
                    targets[i] = new Position(DistanceUnit.METER, 0, 0, 0, 0);
                }
                xPosition = 0;
                yPosition = 0;
                power = 0;

            } else if (-gamepad1.right_stick_y < 0 && !buttonPressed && selectedElement < 10) {
                xPosition = 0;
                yPosition = 0;
                power = 0;
                selectedElement ++;
                buttonPressed = true;
                
            } else if (-gamepad1.right_stick_y > 0 && !buttonPressed && selectedElement > 0) {
                xPosition = 0;
                yPosition = 0;
                power = 0;
                selectedElement --;
                buttonPressed = true;
                
            } else if (-gamepad1.left_stick_y > 0 && !buttonPressed) {
                if (power <= 1) {
                    power += .1;
                }
                buttonPressed = true;

            } else if (-gamepad1.left_stick_y < 0 && !buttonPressed) {
                if (power > 0) {
                    power -= .1;
                }
                buttonPressed = true;

            } else if (gamepad1.dpad_up && !buttonPressed) {
                yPosition += 0.5;
                buttonPressed = true;
                
            } else if (gamepad1.dpad_down && !buttonPressed) {
                yPosition -= 0.5;
                buttonPressed = true;
                
            } else if (gamepad1.dpad_right && !buttonPressed) {
                xPosition += 0.5;
                buttonPressed = true;
                
            } else if (gamepad1.dpad_left && !buttonPressed) {
                xPosition -= 0.5;
                buttonPressed = true;
                
            } else {
                buttonPressed = false;
            }

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle();
        }
    }
}
