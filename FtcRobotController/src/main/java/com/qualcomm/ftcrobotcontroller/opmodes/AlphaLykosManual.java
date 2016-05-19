package com.qualcomm.ftcrobotcontroller.opmodes;


/**
 * Provide a Alternate manual operational mode that uses the left and right
 * drive motors, left arm motor, servo motors and gamepad input from two
 * gamepads for the Push Bot.
 *
 * @author Caleb parks
 * Created by rps on 10/22/15.
 */
public class AlphaLykosManual extends AlphaLykosTelemetry {

    final float a_left_arm_speed = .01f;
    final float a_x_hand_speed = .05f;
    final float a_y_hand_speed = .03f;
    int controller1ControlScheme = 2;
    int controller2ControlScheme = 1;
    float l_left_arm_power = 0;
    float l_extendable_arm_power = 0;
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
    public AlphaLykosManual ()

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

        if (gamepad1.back) {
            if (gamepad1.a) {
                controller1ControlScheme = 1;
            } else if (gamepad1.b) {
                controller1ControlScheme = 2;
            }
        }

        //
        // Determine Which controller 1 control Scheme to use
        //

            if (controller1ControlScheme == 1) {
                useController1ControlScheme1();
            } else if (controller1ControlScheme == 2) {
                useController1ControlScheme2();
            }

        //
        // Determine if the Controller 2 control Scheme needs to be changed
        //

        if (gamepad2.back) {
            if (gamepad1.a) {
                controller1ControlScheme = 1;
            } else if (gamepad1.b) {
                controller1ControlScheme = 1;
            }
        }

        //
        // Determine Which controller 1 control Scheme to use
        //

            if (controller2ControlScheme == 1) {
                useController2ControlScheme1();
            } else if (controller2ControlScheme == 2) {
                useController1ControlScheme1();
        }
        //
        // Send telemetry data to the driver station.
        //
        update_telemetry(); // Update common telemetry
        update_gamepad_telemetry();
        telemetry.addData("12",controller1ControlScheme);
        telemetry.addData("13",controller2ControlScheme);


    } // loop

    void useController1ControlScheme1() {

        // Manage the drive wheel motors.

        float l_left_drive_powerC1 = scale_motor_power (-gamepad1.left_stick_y);
        float l_right_drive_powerC1 = scale_motor_power (-gamepad1.right_stick_y);

        // set the drive power

        set_drive_power (l_left_drive_powerC1, l_right_drive_powerC1);
    }

    void useController1ControlScheme2() {

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
            l_left_drive_power = l_left_drive_power * (gamepad1.left_stick_x - 1);
        }
        else if (scale_motor_power(gamepad1.left_stick_x) > 0){
            l_right_drive_power = l_right_drive_power * (-gamepad1.left_stick_x - 1);
        }

        // set the drive power

        set_drive_power(scale_motor_power(l_left_drive_power), scale_motor_power(l_right_drive_power));

    }

    void useController2ControlScheme1() {

        //
        // Manage the extendable arm
        //
        l_extendable_arm_power = scale_motor_power (gamepad2.left_trigger);
        l_extendable_arm_power = scale_motor_power( l_extendable_arm_power - gamepad2.right_trigger);

        m_extendable_arm_power (l_extendable_arm_power);

        l_left_arm_power = scale_motor_power(-gamepad2.left_stick_y);
        m_left_arm_power(l_left_arm_power);



        // Servo Motors
        //
        // Obtain the current values of the gamepad 'x' and 'b' buttons.
        // Note that x and b buttons have boolean values of true and false.
        // The clip method guarantees the value never exceeds the allowable range of
        // [0,1].
        // The setPosition methods write the motor power values to the Servo
        // class, but the positions aren't applied until this method ends.


        if (ticker == 2) {
            double upper_left_hand = a_upper_left_hand_position() + -gamepad2.left_stick_x * a_x_hand_speed;

            if (gamepad2.left_stick_x != 0 && upper_left_hand >= 0.2) {
                m_upper_left_hand_position(upper_left_hand);
            }

            double lower_left_hand = a_lower_left_hand_position() + gamepad2.left_stick_y * a_y_hand_speed;

            if (gamepad2.left_stick_y != 0) {
                m_lower_left_hand_position(lower_left_hand);
            }

            double upper_right_hand = a_upper_right_hand_position() + -gamepad2.right_stick_x * a_x_hand_speed;

            if (gamepad2.right_stick_x != 0) {
                m_upper_right_hand_position(upper_right_hand);
            }

            double lower_right_hand = a_lower_right_hand_position() + -gamepad2.right_stick_y * a_y_hand_speed;

            if (gamepad2.right_stick_y != 0) {
                m_lower_right_hand_position(lower_right_hand);
            }
            ticker = 1;
        } else {
            ticker++;
        }
    }

} // AlphaLykosManual