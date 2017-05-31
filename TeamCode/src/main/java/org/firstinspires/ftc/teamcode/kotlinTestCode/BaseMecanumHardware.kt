package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Created by caleb on 5/30/2017.
 *
 * This is a hardware class to start testing if kotlin can be used to program as an alternative to java
 *
 * This is a base hardware class for the mecanum drive. Do not put robot specific code in this class.
 * Instead create a robot specific hardware class that inherits this class and put robot specific code in it.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front Left  drive motor:        "f_left_drive"
 * Motor channel:  Front Right drive motor:        "f_right_drive"
 * Motor channel:  Back  Left  drive motor:        "b_left_drive"
 * Motor channel:  Back  Right drive motor:        "b_right_drive"
 *
 */

open class BaseMecanumHardware {

    /* Public OpMode members. */
    lateinit var frontLeftMotor:  DcMotor private set
    lateinit var frontRightMotor: DcMotor private set
    lateinit var backLeftMotor:   DcMotor private set
    lateinit var backRightMotor:  DcMotor private set

    /* local OpMode members. */
    internal lateinit var hwMap: HardwareMap private set
    private var period = ElapsedTime()

    /* Initialize standard Hardware interfaces */
    public fun init(ahwMap: HardwareMap) {
        // Save reference to Hardware map
        hwMap = ahwMap

        // Define and Initialize Motors
        frontLeftMotor  = hwMap.dcMotor.get("f_left_drive")
        frontRightMotor = hwMap.dcMotor.get("f_right_drive")
        backLeftMotor   = hwMap.dcMotor.get("b_left_drive")
        backRightMotor  = hwMap.dcMotor.get("b_right_drive")

        frontLeftMotor.direction  = DcMotorSimple.Direction.FORWARD
        frontRightMotor.direction = DcMotorSimple.Direction.REVERSE
        backLeftMotor.direction   = DcMotorSimple.Direction.FORWARD
        backRightMotor.direction  = DcMotorSimple.Direction.REVERSE

        // set all motors to zero power
        frontLeftMotor.power  = 0.0
        frontRightMotor.power = 0.0
        backLeftMotor.power   = 0.0
        backRightMotor.power  = 0.0

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed
        frontLeftMotor.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeftMotor.mode   = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRightMotor.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    public fun moveRobot(xAxis: Double = 0.0, yAxis: Double = 0.0, rotation: Double = 0.0, telemetry: Telemetry) {
        frontLeftMotor.power  = Range.clip(yAxis + xAxis + rotation, -1.0, 1.0)
        frontRightMotor.power = Range.clip(yAxis - xAxis - rotation, -1.0, 1.0)
        backLeftMotor.power   = Range.clip(yAxis - xAxis + rotation, -1.0, 1.0)
        backRightMotor.power  = Range.clip(yAxis + xAxis - rotation, -1.0, 1.0)
    }

    public fun moveRobot(direction: Direction, rotation: Double, telemetry: Telemetry) {
        moveRobot(direction.xAxis, direction.yAxis, rotation, telemetry)
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick. This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remainight time interval.
     *
     * @param periodMs Lenght of wait cycle in mSec.
     */
    public fun waitForTick (periodMs: Long) {

        var remaining: Long = periodMs - period.milliseconds() as Long

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            Thread.sleep(remaining)
        }

        // Reset the cycle clock for the next pass.
        period.reset()
    }
}

data class Direction (var xAxis: Double, var yAxis: Double)