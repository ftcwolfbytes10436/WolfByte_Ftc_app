package com.qualcomm.ftcrobotcontroller.opmodes;


/**
 * Provide a Alternate manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from two
 * gamepads for the Push Bot.
 *
 * @author Caleb parks
 * Created by rps on 10/22/15.
 */
public class AlphaLykosManualTestRun1 extends AlphaLykosTelemetry {

    final float a_left_arm_speed = .01f;
    final float a_x_hand_speed = .05f;
    final float a_y_hand_speed = .03f;
    int controller1ControlScheme = 2;
    int controller2ControlScheme = 1;
    boolean switchControlSchemesC1 = false;
    boolean switchControlSchemesC2 = false;
    float l_left_arm_power = 0;
    float l_extendable_arm_power = 0;
    float l_vacuum_power = 0;
    int ticker = 1;

    //--------------------------------------------------------------------------
    //
    // PushBotManual
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public AlphaLykosManualTestRun1()

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

        float l_left_drive_power = scale_motor_power(-gamepad1.left_stick_y);
        float l_right_drive_power = scale_motor_power(-gamepad1.right_stick_y);
        set_drive_power(l_left_drive_power,l_right_drive_power);

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();
        telemetry.addData("12",controller1ControlScheme);
        telemetry.addData("13",controller2ControlScheme);


    } // loop


} // AlphaLykosManual