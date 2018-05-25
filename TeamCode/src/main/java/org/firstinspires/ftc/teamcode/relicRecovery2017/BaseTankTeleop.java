package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

// For Saving Joystick values
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.lang.Thread;

@TeleOp(name ="tank_teleop")



public class BaseTankTeleop extends OpMode{

    /* Declare OpMode members. */
    BaseTankHardware robot = new BaseTankHardware();
    boolean triggerHit = false;
    double liftMotorPower = 0;


    // For Saving Joystick values
    ArrayList<Float> LeftValues = new ArrayList<>();
    ArrayList<Float> RightValues = new ArrayList<>();
    ArrayList<Long> TimeValues = new ArrayList<>();

    File f;
    BufferedWriter bw;
    FileWriter fw;

    long StartMillis;

    boolean Recording;


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();

        // For Saving Joystick values
        StartMillis = System.currentTimeMillis();
        Recording = false;

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

        // For Saving Joystick values

        if (gamepad1.a == true) {
            Recording = true;
            LeftValues.clear();
            RightValues.clear();
            TimeValues.clear();
            StartMillis = System.currentTimeMillis();
        }

        if (Recording) { //Only record if a has been pressed to start recording, b stops recording
            try {
                LeftValues.add(gamepad1.left_stick_y);
                RightValues.add(gamepad1.right_stick_y);
                TimeValues.add(System.currentTimeMillis() - StartMillis);

                if (gamepad1.b == true) {
                    try {
                        f = new File("/home/lvuser/Output.txt");
                        if (!f.exists()) {
                            f.createNewFile();
                        }
                        fw = new FileWriter(f);
                        bw = new BufferedWriter(fw);
                    } catch (IOException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }

                    for (int x = 0; x < LeftValues.size(); x++) {
                        bw.write(TimeValues.get(x) + "," + LeftValues.get(x) + "," + RightValues.get(x) + "\n");
                    }

                    bw.close();
                    fw.close();
                    Thread.sleep(1000); //give them time to release the button.
                }

            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            Recording = false;
        }
    }
}


