package org.firstinspires.ftc.teamcode.relicRecovery2017.Test_Scripts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.relicRecovery2017.Test_Scripts.BaseTankAuto;

/**
 * Created by RPS on 10/6/17.
 */

public class BaseTankAutoRoute extends LinearOpMode {

    BaseTankAuto robot = new BaseTankAuto();
    Telemetry.Item pathTelemetry = null;
    Telemetry.Item targetTelemetry = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.setOpMode(this);

        robot.moveForInches(10, 1);

    }

}
