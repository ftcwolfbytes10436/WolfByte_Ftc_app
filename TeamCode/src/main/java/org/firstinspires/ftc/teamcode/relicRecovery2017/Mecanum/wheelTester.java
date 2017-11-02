package org.firstinspires.ftc.teamcode.relicRecovery2017.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name ="wheelTester")
public class wheelTester extends LinearOpMode {

    BaseMecanumHardware robot = new BaseMecanumHardware();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        DcMotor currentMotor = robot.frontLeftMotor;
        int currentMotorNum = 0;
        boolean buttonPressed = false;
        while (opModeIsActive()) {

            if (currentMotorNum > 0 && gamepad1.dpad_down && !buttonPressed) {
                currentMotorNum--;
            } else if (currentMotorNum < 3 && gamepad1.dpad_up && !buttonPressed) {
                currentMotorNum++;
            }
            buttonPressed = gamepad1.dpad_up || gamepad1.dpad_down;

            switch (currentMotorNum) {
                case 0: currentMotor = robot.frontLeftMotor; break;
                case 1: currentMotor = robot.frontRightMotor; break;
                case 2: currentMotor = robot.backLeftMotor; break;
                case 3: currentMotor = robot.backRightMotor; break;
            }

            currentMotor.setPower(gamepad1.right_stick_y);
        }
    }
}
