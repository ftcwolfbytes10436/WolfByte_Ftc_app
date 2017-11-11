package org.firstinspires.ftc.teamcode.relicRecovery2017;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class VariableMenu {
    public int selectedOption = 0;
    Telemetry telemetry;
    ArrayList<MenuOption> menuOptions;

    public void initMenu(Telemetry telemetry, ArrayList<MenuOption> menuOptions) {
        this.menuOptions = menuOptions;
        this.telemetry = telemetry;
        telemetry.setAutoClear(false);
        for (MenuOption variable: menuOptions) {
            variable.initMenuOption();
        }
        telemetry.update();
        menuOptions.get(selectedOption).selectVariable();
    }

    public void nextOption() {
        if (selectedOption < menuOptions.size()-1) {
            menuOptions.get(selectedOption).deselectVariable();
            selectedOption++;
            menuOptions.get(selectedOption).selectVariable();
        }
    }

    public void previousOption() {
        if (selectedOption > 0) {
            menuOptions.get(selectedOption).deselectVariable();
            selectedOption--;
            menuOptions.get(selectedOption).selectVariable();
        }
    }

    public void incrementOption() {
        MenuOption menuItem = menuOptions.get(selectedOption);
        if (menuItem.getSelectedValue() < menuItem.itemNames.length-1) {
            menuItem.changeSelected(menuItem.getSelectedValue() + 1);
        }
    }

    public void decrementOption() {
        MenuOption menuItem = menuOptions.get(selectedOption);
        if (menuItem.getSelectedValue() > 0) {
            menuItem.changeSelected(menuItem.getSelectedValue() - 1);
        }
    }

    public void saveSelectedOptions(String fileName, File filesDir) {
        String options = "";
        for (MenuOption option: menuOptions) {
            options += (char)(option.getSelectedValue()+(int)'a');
        }
        System.out.println(options);
        try {
            File file = new File(filesDir,fileName);
            FileWriter writer = new FileWriter(file);
            writer.write(options);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Integer[] loadSelectedOptions(String fileName, File filesDir) {
        ArrayList<Integer> options = new ArrayList<>();
        try {
            File file = new File(filesDir, fileName);
            FileReader reader = new FileReader(file);
            int option = 0;
            while (option != -1) {
                option = reader.read();
                options.add(option-'a');
            }
            System.out.println(options);
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        Integer[] array = {};
        return options.toArray(array);
    }

    class MenuOption {
        public String displayName;
        public String[] itemNames;
        private int selectedValue;
        private Telemetry.Item tItem;

        public MenuOption(String displayName, String[] itemNames) {
            this(displayName, itemNames, 0);
        }

        public MenuOption(String displayName, String[] itemNames, int startValue) {
            this.displayName = displayName;
            this.itemNames = itemNames;
            this.selectedValue = startValue;
        }

        public void initMenuOption() {
            this.tItem = telemetry.addData(displayName, itemNames[selectedValue]);
        }

        public int getSelectedValue() {
            return selectedValue;
        }

        public void changeSelected(int newSelected) {
            selectedValue = newSelected;
            tItem.setValue(itemNames[selectedValue]);
            telemetry.update();
        }

        public void selectVariable() {
            tItem.setCaption("->" + displayName);
            telemetry.update();
        }

        public void deselectVariable() {
            tItem.setCaption(displayName);
            telemetry.update();
        }
    }
}
