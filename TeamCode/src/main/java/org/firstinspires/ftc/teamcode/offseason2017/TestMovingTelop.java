package org.firstinspires.ftc.teamcode.offseason2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.Timer;
import java.util.TimerTask;

import java.io.FileWriter;
import java.io.IOException;


/**
 * Created by Andrew Brown on 4/16/18.
 */

@TeleOp(name ="Moving Teleop")

public class TestMovingTelop extends OpMode
{

    /* Declare OpMode members. */
    TestHardwareClass robot = new TestHardwareClass();

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();
        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop()
    {
        robot.LeftMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        robot.RightMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
    }

}
