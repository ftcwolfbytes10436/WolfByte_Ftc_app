package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name ="teleop")
public class basicTeleOp extends OpMode {

    /* Declare OpMode members. */
    BaseMecanumHardware robot = new BaseMecanumHardware();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Run the wheels in POV mode (note: The joystick goes negative when pushed forwards so negate it)
        // In this mode the Left stick moves the robot and the Right stick rotates it left and right
        double xAxis = gamepad1.left_stick_x;
        double yAxis = -gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x /* * ((Math.abs(yAxis) > Math.abs(xAxis))? Math.abs(yAxis): Math.abs(xAxis))*/;

        robot.moveRobot(xAxis, yAxis, rotation);
    }
}
