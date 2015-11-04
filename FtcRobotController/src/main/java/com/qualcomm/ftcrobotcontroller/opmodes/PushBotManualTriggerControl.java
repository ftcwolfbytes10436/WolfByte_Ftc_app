package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Provide a Alternate manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from two
 * gamepads for the Push Bot.
 *
 * @author Caleb parks
 * Created by rps on 10/22/15.
 */
public class PushBotManualTriggerControl extends PushBotTelemetry {

    int controller1ControlScheme = 2;
    int controller2ControlScheme = 1;
    boolean switchControlSchemesC1 = false;
    boolean switchControlSchemesC2 = false;
    float l_left_arm_powerC1 = 0;
    float l_left_arm_powerC2 = 0;

    //--------------------------------------------------------------------------
    //
    // PushBotManual
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public PushBotManualTriggerControl ()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotManual

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until this method ends.
        //

        //
        // Determine if the Controller 1 control Scheme needs to be changed
        //

        if (gamepad1.guide){
           switchControlSchemesC1 = !switchControlSchemesC1;
        }

        if (switchControlSchemesC1) {
            if (gamepad1.a) {
                controller1ControlScheme = 1;
            } else if (gamepad1.b) {
                controller1ControlScheme = 2;
            }
            switchControlSchemesC1 = false;
        }

        //
        // Determine Which controller 1 control Scheme to use
        //

        if (!switchControlSchemesC1) {
            if (controller1ControlScheme == 1) {
                useController1ControlScheme1();
            } else if (controller1ControlScheme == 2) {
                useController1ControlScheme2();
            }
        }

        //
        // Determine if the Controller 2 control Scheme needs to be changed
        //

        if (gamepad1.guide) {
            switchControlSchemesC2 = !switchControlSchemesC2;
        }

        if (switchControlSchemesC1) {
            if (gamepad1.a) {
                controller1ControlScheme = 1;
            } else if (gamepad1.b) {
                controller1ControlScheme = 1;
            }
            switchControlSchemesC1 = false;
        }

        //
        // Determine Which controller 1 control Scheme to use
        //

        if (!switchControlSchemesC2) {
            if (controller2ControlScheme == 1) {
                useController2ControlScheme1();
            } else if (controller2ControlScheme == 2) {
                useController1ControlScheme1();
            }
        }

        //
        // set the arm motor
        //

        if (Math.abs(l_left_arm_powerC1) > Math.abs(l_left_arm_powerC2)) {
            m_left_arm_power(l_left_arm_powerC1);
        } else {
            m_left_arm_power(l_left_arm_powerC2);
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        update_gamepad_telemetry ();

    } // loop

    public  void useController1ControlScheme1() {

        // Manage the drive wheel motors.

        float l_left_drive_power = scale_motor_power (-gamepad1.left_stick_y);
        float l_right_drive_power = scale_motor_power (-gamepad1.right_stick_y);

        // set the drive power

        set_drive_power (l_left_drive_power, l_right_drive_power);
    }

    public  void useController1ControlScheme2() {

        //
        // Manage the drive wheel motors.

        // get the power from the right trigger

        float l_left_drive_power = scale_motor_power(gamepad1.right_trigger);
        float l_right_drive_power = scale_motor_power(gamepad1.right_trigger);

        // modify the current power with the left trigger

        l_left_drive_power = l_left_drive_power - scale_motor_power(gamepad1.left_trigger);
        l_right_drive_power = l_right_drive_power - scale_motor_power(gamepad1.left_trigger);

        // determine which way to turn and slow down that wheel

        if (scale_motor_power(gamepad1.left_stick_x) < 0){
            l_left_drive_power = l_left_drive_power * -(gamepad1.left_stick_x - 1);
        }
        else if (scale_motor_power(gamepad1.left_stick_x) > 0){
            l_right_drive_power = l_right_drive_power * -(-gamepad1.left_stick_x - 1);
        }

        // set the drive power

        set_drive_power (scale_motor_power(l_left_drive_power), scale_motor_power(l_right_drive_power));

        // set the arm

        l_left_arm_powerC1 = scale_motor_power (-gamepad2.left_stick_y);

        // set the hands

        if (gamepad1.dpad_right)
        {
            m_hand_position (a_hand_position () + 0.05);
        }
        else if (gamepad1.dpad_left)
        {
            m_hand_position (a_hand_position () - 0.05);
        }

    }

    public void useController2ControlScheme1() {
        //
        // Manage the arm motor.
        //
        l_left_arm_powerC2 = scale_motor_power (-gamepad2.left_stick_y);

        //----------------------------------------------------------------------
        //
        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        //
        // Note that x and b buttons have boolean values of true and false.
        //
        // The clip method guarantees the value never exceeds the allowable range of
        // [0,1].
        //
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.
        //
        if (gamepad2.x)
        {
            m_hand_position (a_hand_position () + 0.05);
        }
        else if (gamepad2.b)
        {
            m_hand_position (a_hand_position () - 0.05);
        }

    }

} // PushBotManual