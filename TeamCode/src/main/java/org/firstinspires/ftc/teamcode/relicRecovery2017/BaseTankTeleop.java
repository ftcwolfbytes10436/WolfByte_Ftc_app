package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name ="tank_teleop")

public class BaseTankTeleop extends OpMode{

    /* Declare OpMode members. */
    BaseTankHardware robot = new BaseTankHardware();
    boolean triggerHit = false;
    double liftMotorPower = 0;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();

        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        robot.LeftMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        robot.RightMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));

        /*
        if (gamepad2.right_trigger > 0.5 && !triggerHit) {
            robot.backFeedPower += 0.05;
        }
        if (gamepad2.left_trigger > 0.5 && !triggerHit) {
            robot.backFeedPower -= 0.05;
        }
        triggerHit = gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        */

        //we have to invert the y joystick then clip it to make sure it is always between -1 and 1
        liftMotorPower = Range.clip(gamepad2.right_stick_y * -1, -1, 1 );

        if (liftMotorPower > 0) //lifting
        {
            liftMotorPower = liftMotorPower *.4; //we want 40% of the joystick value for the power
        }
        else //lowering
        {
            liftMotorPower = liftMotorPower * .1; // we want 10% of the joytick value for power (slow down the lowering)
        }

        robot.LifterMotor.setPower(robot.backFeedPower + liftMotorPower);

        telemetry.addData("backdrive", robot.backFeedPower);
        telemetry.addData("Left Servo: ", robot.leftGripper.getPosition());
        telemetry.addData("Right Servo: ", robot.rightGripper.getPosition());
        telemetry.update();

        if (gamepad2.left_stick_y > .8)
        {
            robot.setGriperPos(.75);
        }
        else if (gamepad2.left_stick_y < -.8)
        {
            robot.setGriperPos(.5);
        }
        else
        {
            robot.setGriperPos(1);
        }
    }
}

