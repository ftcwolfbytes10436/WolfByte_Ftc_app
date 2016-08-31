package com.qualcomm.ftcrobotcontroller.opmodes.alphaLykos;

import java.util.Timer;
import java.util.TimerTask;

//------------------------------------------------------------------------------
//
// PushBotAuto
//
/**
 * Provide a basic autonomous operational mode that uses the left and right
 * drive motors and associated encoders implemented using a state machine for
 * the Push Bot.
 *
 * @author SSI Robotics
 * @version 2015-08-01-06-01
 */
public class AlphaLykosAutoTest extends AlphaLykosTelemetry

{
    //--------------------------------------------------------------------------
    //
    // PushBotAuto
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    public AlphaLykosAutoTest ()

    {

        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } // PushBotAuto


    @Override public void init()

    {
        Timer timer = new Timer();
    }

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {

        if (!timerScheduled) {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    driveForward = false;
                    driveMoterSet = false;
                }
            },7 * 1000);
            driveForward = true;
            timerScheduled = true;
        }

        if (!driveMoterSet) {
            if (driveForward) {
                set_drive_power(1,1);
                driveMoterSet = true;
            } else {
                set_drive_power(0,0);
                driveMoterSet = true;
            }
        }

        //
        // Send telemetry data to the driver station.
        //
        update_telemetry (); // Update common telemetry
        telemetry.addData ("18", "State: " + v_state);

    } // loop

    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     */
    private int v_state = 0;
    private boolean timerScheduled = false;
    private boolean driveForward = false;
    private boolean driveMoterSet = false;
    private Timer timer = new Timer();


} // PushBotAuto
