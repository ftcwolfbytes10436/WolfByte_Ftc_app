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

@TeleOp(name ="Recording Teleop")

public class TestTeleop extends OpMode
{
    boolean timing = false;

    int counter = 0;

    FileWriter fileWriter = null;

    private static final String COMMA_DELIMITER = ",";
    private static final String NEW_LINE_SEPARATOR = "\n";

    private static final String FILE_HEADER = "leftEncoder,rightEncoder";

    /* Declare OpMode members. */
    TestHardwareClass robot = new TestHardwareClass();

    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry);

        telemetry.addData("Status", "waiting to start");
        telemetry.update();

        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try
        {
            fileWriter = new FileWriter("testRecording.csv");

            //Write the CSV file header
            fileWriter.append(FILE_HEADER);

            //Add a new line separator after the header
            fileWriter.append(NEW_LINE_SEPARATOR);
        }
        catch (Exception e)
        {
            System.out.println("Error in CsvFileWriter !!!");
            e.printStackTrace();
        }

        timing = false;
        counter = 1;

        Record();
    }

    Timer timer;

    public void Record() {
        timer = new Timer();
        timer.schedule(new Recording(),
                0,        //initial delay in ms
                250);  //subsequent rate in ms
    }

    class Recording extends TimerTask {

        public void run() {
            try {
                fileWriter.append(String.valueOf(robot.LeftMotor.getCurrentPosition()));
                fileWriter.append(COMMA_DELIMITER);
                fileWriter.append(String.valueOf(robot.RightMotor.getCurrentPosition()));
                fileWriter.append(NEW_LINE_SEPARATOR);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void loop()
    {
        robot.LeftMotor.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        robot.RightMotor.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
    }

}
