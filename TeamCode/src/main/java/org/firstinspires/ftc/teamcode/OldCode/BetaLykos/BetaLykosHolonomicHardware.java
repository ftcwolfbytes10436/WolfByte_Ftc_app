package org.firstinspires.ftc.teamcode.OldCode.BetaLykos;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// BetaLykosHolonomicHardware
//
/**
 * Provides a single hardware access point between custom op-modes and the
 * OpMode class for the Push Bot.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 */
public class BetaLykosHolonomicHardware extends OpMode

{
    //--------------------------------------------------------------------------
    //
    // BetaLykosHolonomicHardware
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public BetaLykosHolonomicHardware()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotHardware

    //--------------------------------------------------------------------------
    //
    // init
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void init ()

    {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //
        // The variable below is used to provide telemetry data to a class user.
        //
        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        try
        {
            v_motor_front_left_drive = hardwareMap.dcMotor.get ("front_left_drive");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_front_left_drive = null;
        }

        try
        {
            v_motor_front_right_drive = hardwareMap.dcMotor.get ("front_right_drive");
            v_motor_front_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_front_right_drive = null;
        }

        try
        {
            v_motor_back_left_drive = hardwareMap.dcMotor.get ("back_left_drive");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("back_left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_back_left_drive = null;
        }

        try
        {
            v_motor_back_right_drive = hardwareMap.dcMotor.get ("back_right_drive");
            v_motor_back_right_drive.setDirection (DcMotor.Direction.REVERSE);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("back_right_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_back_right_drive = null;
        }

        //
        // Connect the arm motor.
        //
        try
        {
            v_motor_left_arm = hardwareMap.dcMotor.get ("left_arm");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_arm");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_motor_left_arm = null;
        }

        //
        // Connect the servo motors.
        //
        // Indicate the initial position of both the left and right servos.  The
        // hand should be halfway opened/closed.
        //
        double l_hand_position = 0.5;

        try
        {
            v_servo_left_hand = hardwareMap.servo.get ("left_hand");
            v_servo_left_hand.setPosition (l_hand_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_left_hand = null;
        }

        try
        {
            v_servo_right_hand = hardwareMap.servo.get ("right_hand");
            v_servo_right_hand.setPosition (l_hand_position);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("right_hand");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            v_servo_right_hand = null;
        }

    } // init

    //--------------------------------------------------------------------------
    //
    // a_warning_generated
    //
    /**
     * Access whether a warning has been generated.
     */
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    /**
     * Access the warning message.
     */
    String a_warning_message ()

    {
        return v_warning_message;

    } // a_warning_message

    //--------------------------------------------------------------------------
    //
    // m_warning_message
    //
    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     *
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void m_warning_message (String p_exception_message)

    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()

    {
        //
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Perform any actions that are necessary while the OpMode is running.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // loop

    //--------------------------------------------------------------------------
    //
    // stop
    //
    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     *
     * The system calls this member once when the OpMode is disabled.
     */
    @Override public void stop ()
    {
        //
        // Nothing needs to be done for this method.
        //

    } // stop

    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    float scale_motor_power (float p_power)
    {
        //
        // Assume no scaling.
        //
        float l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        float l_power = Range.clip (p_power, -1, 1);

        float[] l_array =
                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int)(l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_front_left_drive_power
    //
    /**
     * Access the front left drive motor's power level.
     */
    double a_front_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_front_left_drive != null)
        {
            l_return = v_motor_front_left_drive.getPower ();
        }

        return l_return;

    } // a_front_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_front_right_drive_power
    //
    /**
     * Access the front right drive motor's power level.
     */
    double a_front_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_front_right_drive != null)
        {
            l_return = v_motor_front_right_drive.getPower ();
        }

        return l_return;

    } // a_front_right_drive_power

    //--------------------------------------------------------------------------
    //
    // a_back_left_drive_power
    //
    /**
     * Access the back left drive motor's power level.
     */
    double a_back_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_back_left_drive != null)
        {
            l_return = v_motor_back_left_drive.getPower ();
        }

        return l_return;

    } // a_back_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_back_right_drive_power
    //
    /**
     * Access the back right drive motor's power level.
     */
    double a_back_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_back_right_drive != null)
        {
            l_return = v_motor_back_right_drive.getPower ();
        }

        return l_return;

    } // a_back_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */

    void  set_drive_power (double p_front_left_power, double p_front_right_power, double p_back_left_power, double p_back_right_power)

    {
        if (v_motor_front_left_drive != null)
        {
            v_motor_front_left_drive.setPower (p_front_left_power);
        }
        if (v_motor_front_right_drive != null)
        {
            v_motor_front_right_drive.setPower (p_front_right_power);
        }
        if (v_motor_back_left_drive != null)
        {
            v_motor_back_left_drive.setPower (p_back_left_power);
        }
        if (v_motor_back_right_drive != null)
        {
            v_motor_back_right_drive.setPower (p_back_right_power);
        }
    }

    //--------------------------------------------------------------------------
    //
    // run_using_front_left_drive_encoder
    //
    /**
     * Set the front left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_front_left_drive_encoder()

    {
        if (v_motor_front_left_drive != null)
        {
            v_motor_front_left_drive.setMode
                    ( DcMotor.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_front_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_front_right_drive_encoder
    //
    /**
     * Set the front right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_front_right_drive_encoder()

    {
        if (v_motor_front_right_drive != null)
        {
            v_motor_front_right_drive.setMode
                    ( DcMotor.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_front_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_back_left_drive_encoder
    //
    /**
     * Set the back left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_back_left_drive_encoder()

    {
        if (v_motor_back_left_drive != null)
        {
            v_motor_back_left_drive.setMode
                    ( DcMotor.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_back_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_front_right_drive_encoder
    //
    /**
     * Set the back right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_back_right_drive_encoder()

    {
        if (v_motor_back_right_drive != null)
        {
            v_motor_back_right_drive.setMode
                    ( DcMotor.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_front_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Set all drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_using_front_left_drive_encoder();
        run_using_front_right_drive_encoder();
        run_using_back_left_drive_encoder();
        run_using_back_right_drive_encoder();

    } // run_using_encoders

    //--------------------------------------------------------------------------
    //
    // run_without_front_left_drive_encoder
    //
    /**
     * Set the front left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_front_left_drive_encoder()

    {
        if (v_motor_front_left_drive != null)
        {
            if (v_motor_front_left_drive.getMode () ==
                    DcMotor.RunMode.RESET_ENCODERS)
            {
                v_motor_front_left_drive.setMode
                        ( DcMotor.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_front_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_front_right_drive_encoder
    //
    /**
     * Set the front right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_front_right_drive_encoder()

    {
        if (v_motor_front_right_drive != null)
        {
            if (v_motor_front_right_drive.getMode () ==
                    DcMotor.RunMode.RESET_ENCODERS)
            {
                v_motor_front_right_drive.setMode
                        ( DcMotor.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_front_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_back_left_drive_encoder
    //
    /**
     * Set the back left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_back_left_drive_encoder()

    {
        if (v_motor_back_left_drive != null)
        {
            if (v_motor_back_left_drive.getMode () ==
                    DcMotor.RunMode.RESET_ENCODERS)
            {
                v_motor_back_left_drive.setMode
                        ( DcMotor.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_front_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_back_right_drive_encoder
    //
    /**
     * Set the back right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_back_right_drive_encoder()

    {
        if (v_motor_back_right_drive != null)
        {
            if (v_motor_back_right_drive.getMode () ==
                    DcMotor.RunMode.RESET_ENCODERS)
            {
                v_motor_back_right_drive.setMode
                        ( DcMotor.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_back_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_drive_encoders
    //
    /**
     * Set all drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_without_drive_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_without_front_left_drive_encoder();
        run_without_front_right_drive_encoder();
        run_without_back_left_drive_encoder();
        run_without_back_right_drive_encoder();

    } // run_without_drive_encoders

    //--------------------------------------------------------------------------
    //
    // reset_front_left_drive_encoder
    //
    /**
     * Reset the front left drive wheel encoder.
     */
    public void reset_front_left_drive_encoder()

    {
        if (v_motor_front_left_drive != null)
        {
            v_motor_front_left_drive.setMode
                    ( DcMotor.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_front_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_front_right_drive_encoder
    //
    /**
     * Reset the front right drive wheel encoder.
     */
    public void reset_front_right_drive_encoder()

    {
        if (v_motor_front_right_drive != null)
        {
            v_motor_front_right_drive.setMode
                    ( DcMotor.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_front_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_back_left_drive_encoder
    //
    /**
     * Reset the back left drive wheel encoder.
     */
    public void reset_back_left_drive_encoder()

    {
        if (v_motor_back_left_drive != null)
        {
            v_motor_back_left_drive.setMode
                    ( DcMotor.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_back_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_back_right_drive_encoder
    //
    /**
     * Reset the back right drive wheel encoder.
     */
    public void reset_back_right_drive_encoder()

    {
        if (v_motor_back_right_drive != null)
        {
            v_motor_back_right_drive.setMode
                    ( DcMotor.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_back_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Reset all drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_front_left_drive_encoder();
        reset_front_right_drive_encoder();
        reset_back_left_drive_encoder();
        reset_back_right_drive_encoder();

    } // reset_drive_encoders

    //--------------------------------------------------------------------------
    //
    // a_front_left_encoder_count
    //
    /**
     * Access the front left encoder's count.
     */
    int a_front_left_encoder_count()
    {
        int l_return = 0;

        if (v_motor_front_left_drive != null)
        {
            l_return = v_motor_front_left_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_front_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_front_right_encoder_count
    //
    /**
     * Access the front right encoder's count.
     */
    int a_front_right_encoder_count()

    {
        int l_return = 0;

        if (v_motor_front_right_drive != null)
        {
            l_return = v_motor_front_right_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_front_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_back_left_encoder_count
    //
    /**
     * Access the bakc left encoder's count.
     */
    int a_back_left_encoder_count()
    {
        int l_return = 0;

        if (v_motor_back_left_drive != null)
        {
            l_return = v_motor_back_left_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_back_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_back_right_encoder_count
    //
    /**
     * Access the back right encoder's count.
     */
    int a_back_right_encoder_count()

    {
        int l_return = 0;

        if (v_motor_back_right_drive != null)
        {
            l_return = v_motor_back_right_drive.getCurrentPosition ();
        }

        return l_return;

    } // a_back_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // has_front_left_drive_encoder_reached
    //
    /**
     * Indicate whether the front left drive motor's encoder has reached a value.
     */
    boolean has_front_left_drive_encoder_reached(double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_front_left_drive != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_front_left_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_front_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_front_right_drive_encoder_reached
    //
    /**
     * Indicate whether the front right drive motor's encoder has reached a value.
     */
    boolean has_front_right_drive_encoder_reached(double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_front_right_drive != null)
        {
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_front_right_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_front_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_back_left_drive_encoder_reached
    //
    /**
     * Indicate whether the back left drive motor's encoder has reached a value.
     */
    boolean has_back_left_drive_encoder_reached(double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_back_left_drive != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_back_left_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_back_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_back_right_drive_encoder_reached
    //
    /**
     * Indicate whether the back right drive motor's encoder has reached a value.
     */
    boolean has_back_right_drive_encoder_reached(double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_back_right_drive != null)
        {
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (v_motor_back_right_drive.getCurrentPosition ()) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_back_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean have_drive_encoders_reached
    ( double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_front_left_drive_encoder_reached(p_left_count) &&
                has_front_right_drive_encoder_reached(p_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    boolean have_drive_encoders_reached
            ( double p_front_left_count
                    , double p_front_right_count
                    , double p_back_left_count
                    , double p_back_right_count
            )

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_front_left_drive_encoder_reached(p_front_left_count) &&
                has_front_right_drive_encoder_reached(p_front_right_count) &&
                has_back_left_drive_encoder_reached(p_back_left_count) &&
                has_back_right_drive_encoder_reached(p_back_right_count))
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    //--------------------------------------------------------------------------
    //
    // drive_using_encoders
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean drive_using_encoders
    ( double p_front_left_power
            , double p_front_right_power
            , double p_back_left_power
            , double p_back_right_power
            , double p_front_left_count
            , double p_front_right_count
            , double p_back_left_count
            , double p_back_right_count
    )

    {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        run_using_encoders ();

        //
        // Start the drive wheel motors at full power.
        //
        set_drive_power (p_front_left_power, p_front_right_power, p_back_left_power,p_back_right_power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached (p_front_left_count, p_front_right_count,p_back_left_count,p_back_right_count))
        {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders ();

            //
            // Stop the motors.
            //
            set_drive_power (0.0f, 0.0f,0.0f,0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_using_encoders

    //--------------------------------------------------------------------------
    //
    // has_front_left_drive_encoder_reset
    //
    /**
     * Indicate whether the front left drive encoder has been completely reset.
     */
    boolean has_front_left_drive_encoder_reset()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the front left encoder reached zero?
        //
        if (a_front_left_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_front_left_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_front_right_drive_encoder_reset
    //
    /**
     * Indicate whether the front right drive encoder has been completely reset.
     */
    boolean has_front_right_drive_encoder_reset()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the front right encoder reached zero?
        //
        if (a_front_right_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_front_right_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_back_left_drive_encoder_reset
    //
    /**
     * Indicate whether the back left drive encoder has been completely reset.
     */
    boolean has_back_left_drive_encoder_reset()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the back left encoder reached zero?
        //
        if (a_back_left_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_back_left_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_back_right_drive_encoder_reset
    //
    /**
     * Indicate whether the back right drive encoder has been completely reset.
     */
    boolean has_back_right_drive_encoder_reset()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the back right encoder reached zero?
        //
        if (a_back_right_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_back_right_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    /**
     * Indicate whether the encoders have been completely reset.
     */
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached zero?
        //
        if (has_front_left_drive_encoder_reset() && has_front_right_drive_encoder_reset()
                && has_back_left_drive_encoder_reset() && has_back_right_drive_encoder_reset())
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_drive_encoders_reset

    //--------------------------------------------------------------------------
    //
    // a_left_arm_power
    //
    /**
     * Access the left arm motor's power level.
     */
    double a_left_arm_power ()
    {
        double l_return = 0.0;

        if (v_motor_left_arm != null)
        {
            l_return = v_motor_left_arm.getPower ();
        }

        return l_return;

    } // a_left_arm_power

    //--------------------------------------------------------------------------
    //
    // m_left_arm_power
    //
    /**
     * Access the left arm motor's power level.
     */
    void m_left_arm_power (double p_level)
    {
        if (v_motor_left_arm != null)
        {
            v_motor_left_arm.setPower (p_level);
        }

    } // m_left_arm_power

    //--------------------------------------------------------------------------
    //
    // a_hand_position
    //
    /**
     * Access the hand position.
     */
    double a_hand_position ()
    {
        double l_return = 0.0;

        if (v_servo_left_hand != null)
        {
            l_return = v_servo_left_hand.getPosition ();
        }

        return l_return;

    } // a_hand_position

    //--------------------------------------------------------------------------
    //
    // m_hand_position
    //
    /**
     * Mutate the hand position.
     */
    void m_hand_position (double p_position)
    {
        //
        // Ensure the specific value is legal.
        //
        double l_position = Range.clip
                ( p_position
                        , Servo.MIN_POSITION
                        , Servo.MAX_POSITION
                );

        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        if (v_servo_left_hand != null)
        {
            v_servo_left_hand.setPosition (l_position);
        }
        if (v_servo_right_hand != null)
        {
            v_servo_right_hand.setPosition (1.0 - l_position);
        }

    } // m_hand_position

    //--------------------------------------------------------------------------
    //
    // open_hand
    //
    /**
     * Open the hand to its fullest.
     */
    void open_hand ()

    {
        //
        // Set the value.  The right hand value must be opposite of the left
        // value.
        //
        if (v_servo_left_hand != null)
        {
            v_servo_left_hand.setPosition (Servo.MAX_POSITION);
        }
        if (v_servo_right_hand != null)
        {
            v_servo_right_hand.setPosition (Servo.MIN_POSITION);
        }

    } // open_hand

    //--------------------------------------------------------------------------
    //
    // v_warning_generated
    //
    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean v_warning_generated = false;

    //--------------------------------------------------------------------------
    //
    // v_warning_message
    //
    /**
     * Store a message to the user if one has been generated.
     */
    private String v_warning_message;

    //--------------------------------------------------------------------------
    //
    // v_motor_front_left_drive
    //
    /**
     * Manage the aspects of the front left drive motor.
     */
    private DcMotor v_motor_front_left_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_front_right_drive
    //
    /**
     * Manage the aspects of the front right drive motor.
     */
    private DcMotor v_motor_front_right_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_back_left_drive
    //
    /**
     * Manage the aspects of the back left drive motor.
     */
    private DcMotor v_motor_back_left_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_back_right_drive
    //
    /**
     * Manage the aspects of the back right drive motor.
     */
    private DcMotor v_motor_back_right_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_left_arm
    //
    /**
     * Manage the aspects of the left arm motor.
     */
    private DcMotor v_motor_left_arm;

    //--------------------------------------------------------------------------
    //
    // v_servo_left_hand
    //
    /**
     * Manage the aspects of the left hand servo.
     */
    private Servo v_servo_left_hand;

    //--------------------------------------------------------------------------
    //
    // v_servo_right_hand
    //
    /**
     * Manage the aspects of the right hand servo.
     */
    private Servo v_servo_right_hand;

} // PushBotHardware
