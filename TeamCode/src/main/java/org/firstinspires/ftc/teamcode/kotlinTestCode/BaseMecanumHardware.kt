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
 * This hardware class assumes that the robot only has 4 drive motors without encoders.
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
    var frontLeftMotor:  DcMotor? = null; private set
    var frontRightMotor: DcMotor? = null; private set
    var backLeftMotor:   DcMotor? = null; private set
    var backRightMotor:  DcMotor? = null; private set

    /* local OpMode members. */
    internal lateinit var hwMap: HardwareMap private set
    internal lateinit var telemetry: Telemetry private set
    private var period = ElapsedTime()

    /* Initialize standard Hardware interfaces */
    open fun init(ahwMap: HardwareMap, telemetry: Telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap

        // Save reference to Telemetry instance
        this.telemetry = telemetry

        // assign motors to temp variables
        var frontLeftMotorTemp: DcMotor?  = hwMap.dcMotor.get("f_left_drive")
        var frontRightMotorTemp: DcMotor? = hwMap.dcMotor.get("f_right_drive")
        var backLeftMotorTemp: DcMotor?   = hwMap.dcMotor.get("b_left_drive")
        var backRightMotorTemp: DcMotor?  = hwMap.dcMotor.get("b_right_drive")

        // check to make sure the motor variables are not null and assign the motors to class variables
        // else if they are null send a message to telemetry
        if (frontLeftMotorTemp  != null) frontLeftMotor  = frontLeftMotorTemp  else telemetry.addData("error", "f_left_drive not found")
        if (frontRightMotorTemp != null) frontRightMotor = frontRightMotorTemp else telemetry.addData("error", "f_right_drive not found")
        if (backLeftMotorTemp   != null) backLeftMotor   = backLeftMotorTemp   else telemetry.addData("error", "b_left_drive not found")
        if (backRightMotorTemp  != null) backRightMotor  = backRightMotorTemp  else telemetry.addData("error", "b_right_drive not found")

        // set the motor direction
        frontLeftMotor?.direction = DcMotorSimple.Direction.FORWARD
        frontRightMotor?.direction = DcMotorSimple.Direction.REVERSE
        backLeftMotor?.direction   = DcMotorSimple.Direction.FORWARD
        backRightMotor?.direction  = DcMotorSimple.Direction.REVERSE

        // set all motors to zero power
        frontLeftMotor?.power  = 0.0
        frontRightMotor?.power = 0.0
        backLeftMotor?.power   = 0.0
        backRightMotor?.power  = 0.0

        // Set all motors to run without encoders.
        frontLeftMotor?.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRightMotor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeftMotor?.mode   = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRightMotor?.mode  = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun moveRobot(xAxis: Double = 0.0, yAxis: Double = 0.0, rotation: Double = 0.0) {
        frontLeftMotor?.power  = Range.clip(yAxis + xAxis + rotation, -1.0, 1.0)
        frontRightMotor?.power = Range.clip(yAxis - xAxis - rotation, -1.0, 1.0)
        backLeftMotor?.power   = Range.clip(yAxis - xAxis + rotation, -1.0, 1.0)
        backRightMotor?.power  = Range.clip(yAxis + xAxis - rotation, -1.0, 1.0)
    }

    fun moveRobot(direction: Direction, rotation: Double = 0.0) {
        moveRobot(direction.xAxis, direction.yAxis, rotation)
    }
}

data class Direction (var xAxis: Double, var yAxis: Double)