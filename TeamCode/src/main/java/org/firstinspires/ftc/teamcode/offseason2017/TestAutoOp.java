package org.firstinspires.ftc.teamcode.offseason2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;


@Autonomous (name = "TestRun")
public class TestAutoOp extends LinearOpMode {

    TestAuto robot = new TestAuto();

    int counter = 0;
    int leftCount = 0;
    int rightCount = 0;
    String prefix = "";

    @Override
    public void runOpMode() throws InterruptedException
    {
        try (Scanner scanner = new Scanner(new File("MOCK_DATA.csv"));)
        {
            scanner.useDelimiter(",|\\n");

            while (scanner.hasNext())
            {
                String token = scanner.next();
                if (counter > 2)
                {
                    if (counter % 2 == 0)
                    {
                        leftCount = Integer.parseInt(token);
                    }
                    else if (counter % 2 != 0)
                    {
                        rightCount = Integer.parseInt(token);
                    }
                }
                counter++;
            }
        }
        catch(FileNotFoundException fnfe)
        {
            System.out.println(fnfe);
        }

        try {
            robot.moveToTick(leftCount, rightCount, 0.4);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
}
