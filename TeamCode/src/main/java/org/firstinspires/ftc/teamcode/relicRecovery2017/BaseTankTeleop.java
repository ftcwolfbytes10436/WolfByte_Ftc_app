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
            backFeedPower += 0.1;
        }
        if (gamepad2.left_trigger > 0.5 && !triggerHit) {
            backFeedPower -= 0.1;
        }
        triggerHit = gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        */

        robot.LifterMotor.setPower(robot.backFeedPower+0.4 * (Range.clip(-gamepad2.right_stick_y, -1, 1)));

        //telemetry.addData("backdrive", robot.backFeedPower);
        telemetry.addData("Left Servo: ", robot.leftGripper.getPosition());
        telemetry.addData("Right Servo: ", robot.rightGripper.getPosition());
        telemetry.update();

        if (gamepad2.left_stick_y > .8)
        {
            robot.setGriperPos(.9);
        }
        else if (gamepad2.left_stick_y < -.8)
        {
            robot.setGriperPos(.75);
        }
        else
        {
            robot.setGriperPos(1);
        }
    }
}

