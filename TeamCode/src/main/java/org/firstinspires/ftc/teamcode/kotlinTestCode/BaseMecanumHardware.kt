package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.cout970.vector.impl.Vector2d
import com.qualcomm.robotcore.hardware.*
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

    /* Initialize standard Hardware interfaces */
    open fun init(ahwMap: HardwareMap, telemetry: Telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap

        // Save reference to Telemetry instance
        this.telemetry = telemetry

        // Get and initialize the motors
        frontLeftMotor = initMotor("f_left_drive", DcMotorSimple.Direction.FORWARD)
        frontRightMotor = initMotor("f_right_drive", DcMotorSimple.Direction.REVERSE)
        backLeftMotor = initMotor("b_left_drive", DcMotorSimple.Direction.FORWARD)
        backRightMotor = initMotor("b_right_drive", DcMotorSimple.Direction.REVERSE)
    }

    fun initMotor(hardwareName: String, direction: DcMotorSimple.Direction) : DcMotor? {
        val motor: DcMotor? = hwMap.dcMotor.get(hardwareName)
        if (motor == null) telemetry.addData("error", "$hardwareName not found")
        motor?.direction = direction
        motor?.power = 0.0
        motor?.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        return motor
    }

    fun moveRobot(xAxis: Double = 0.0, yAxis: Double = 0.0, rotation: Double = 0.0) {
        frontLeftMotor?.power  = Range.clip(yAxis + xAxis + rotation, -1.0, 1.0)
        frontRightMotor?.power = Range.clip(yAxis - xAxis - rotation, -1.0, 1.0)
        backLeftMotor?.power   = Range.clip(yAxis - xAxis + rotation, -1.0, 1.0)
        backRightMotor?.power  = Range.clip(yAxis + xAxis - rotation, -1.0, 1.0)
    }

    fun moveRobot(direction: Vector2d, rotation: Double = 0.0) {
        moveRobot(direction.x, direction.y, rotation)
    }
}