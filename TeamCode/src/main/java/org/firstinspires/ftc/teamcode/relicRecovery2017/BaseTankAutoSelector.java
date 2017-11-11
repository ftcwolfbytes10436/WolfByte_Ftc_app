package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous(name="tank autonomous route selector")
public class BaseTankAutoSelector extends LinearOpMode {

    final String fileName = "AutoSelection";

    @Override
    public void runOpMode() throws InterruptedException {

        boolean buttonPushed = false;

        Integer[] optionsList = VariableMenu.loadSelectedOptions(fileName, hardwareMap.appContext.getFilesDir());
        boolean isFull = optionsList.length > 3;
        VariableMenu menu = new VariableMenu();
        ArrayList<VariableMenu.MenuOption> menuOptions = new ArrayList<>();
        menuOptions.add(menu.new MenuOption("Team color", new String[]{"Red","blue"}, isFull? optionsList[0]: 0));
        menuOptions.add(menu.new MenuOption("Start Position", new String[]{"Straight", "Turn"}, isFull? optionsList[1]: 0));
        menuOptions.add(menu.new MenuOption("Placement", new String[]{"Left", "Center", "Right"}, isFull? optionsList[2]: 0));

        waitForStart();

        menu.initMenu(telemetry, menuOptions);

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !buttonPushed) {
                menu.previousOption();

            } else if (gamepad1.dpad_down && !buttonPushed) {
                menu.nextOption();

            } else if (gamepad1.dpad_left && !buttonPushed) {
                menu.decrementOption();

            } else if (gamepad1.dpad_right && !buttonPushed){
                menu.incrementOption();

            }
            buttonPushed = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
        }
        menu.saveSelectedOptions(fileName, hardwareMap.appContext.getFilesDir());
    }
}