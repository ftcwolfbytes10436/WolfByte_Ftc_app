package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.cout970.vector.extensions.length
import com.cout970.vector.extensions.normalize
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

open class pidtestHardware {

    /* Public OpMode members. */
    var frontLeftMotor:  DcMotor? = null; private set
    var frontRightMotor: DcMotor? = null; private set
    var backLeftMotor:   DcMotor? = null; private set
    var backRightMotor:  DcMotor? = null; private set

    // last Encoder count
    var flEC: Int = 0
    var frEC: Int = 0
    var blEC: Int = 0
    var brEC: Int = 0

    var lastTime: Long = 0

    val frontLeftPID: PIDController = PIDController()
    val frontRightPID: PIDController = PIDController()
    val backLeftPID: PIDController = PIDController()
    val backRightPID: PIDController = PIDController()

    var driveWheel: PIDController = PIDController();private set(value) {field.SetMode(true); field = value; value.SetMode(false)}

    var lastDirection: Vector2d = Vector2d(0.0,1.0)

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

        frontLeftPID.SetTunings(0.0f,0.0f, 0.0f)
        frontRightPID.SetTunings(0.0f,0.0f, 0.0f)
        backLeftPID.SetTunings(0.0f,0.0f, 0.0f)
        backRightPID.SetTunings(0.0f,0.0f, 0.0f)

        frontLeftPID.SetMode(true)
        frontRightPID.SetMode(true)
        backLeftPID.SetMode(true)
        backRightPID.SetMode(true)
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
        val timeDiff = ((System.currentTimeMillis() - lastTime)/1000).toFloat()
        lastTime = System.currentTimeMillis()

        val flS: Float = ((frontLeftMotor?.currentPosition ?: 0) - flEC).toFloat()
        val frS: Float = ((frontRightMotor?.currentPosition ?: 0) - frEC).toFloat()
        val blS: Float = ((backLeftMotor?.currentPosition ?: 0) - blEC).toFloat()
        val brS: Float = ((backRightMotor?.currentPosition ?: 0) - brEC).toFloat()

        flEC = frontLeftMotor?.currentPosition ?: 0
        frEC = frontRightMotor?.currentPosition ?: 0
        blEC = backLeftMotor?.currentPosition ?: 0
        brEC = backRightMotor?.currentPosition ?: 0

        frontLeftPID.Input = Math.abs(flS / if (lastDirection.length() != 0.0) calculateWheelPower(1,lastDirection) else 1.0f)
        frontRightPID.Input = Math.abs(frS / if (lastDirection.length() != 0.0) calculateWheelPower(2,lastDirection) else 1.0f)
        backLeftPID.Input = Math.abs(blS / if (lastDirection.length() != 0.0) calculateWheelPower(3,lastDirection) else 1.0f)
        backRightPID.Input = Math.abs(brS / if (lastDirection.length() != 0.0) calculateWheelPower(4,lastDirection) else 1.0f)

        frontLeftPID.Setpoint = driveWheel.Input
        frontRightPID.Setpoint = driveWheel.Input
        backLeftPID.Setpoint = driveWheel.Input
        backRightPID.Setpoint = driveWheel.Input

        frontLeftPID.Compute()
        frontRightPID.Compute()
        backLeftPID.Compute()
        backRightPID.Compute()

        driveWheel.Output = calculateWheelPower(getMotorNumberFromReference(driveWheel), Vector2d(xAxis, yAxis))

        val normalizedInput: Vector2d = Vector2d(xAxis, yAxis).normalize() as Vector2d

        val flOutput = (frontLeftMotor?.power ?: 0.0) + Math.copySign(frontLeftPID.Output, calculateWheelPower(1, normalizedInput))
        val frOutput = (frontRightMotor?.power ?: 0.0) + Math.copySign(frontRightPID.Output, calculateWheelPower(2, normalizedInput))
        val blOutput = (backLeftMotor?.power ?: 0.0) + Math.copySign(backLeftPID.Output, calculateWheelPower(3, normalizedInput))
        val brOutput = (backRightMotor?.power ?: 0.0) + Math.copySign(backRightPID.Output, calculateWheelPower(4, normalizedInput))

        frontLeftMotor?.power  = Range.clip(flOutput + rotation, -1.0, 1.0)
        frontRightMotor?.power = Range.clip(frOutput - rotation, -1.0, 1.0)
        backLeftMotor?.power   = Range.clip(blOutput + rotation, -1.0, 1.0)
        backRightMotor?.power  = Range.clip(brOutput - rotation, -1.0, 1.0)

        lastDirection = normalizedInput
    }

    fun moveRobot(direction: Vector2d, rotation: Double = 0.0) {
        moveRobot(direction.x, direction.y, rotation)
    }

    fun calculateWheelPower(wheel: Int, direction: Vector2d): Float = when(wheel) {
        1,4 -> (direction.y + direction.x).toFloat()
        else -> (direction.y - direction.x).toFloat()
    }

    fun  getMotorNumberFromReference(ref: PIDController): Int {
        if (ref === frontLeftPID) return 1
        else if (ref === frontRightPID) return 2
        else if (ref === backLeftPID) return 3
        else if (ref === backRightPID) return 4
        else return 1
    }

    fun changeDriveWheel(wheel: Int) = when(wheel) {
        1 -> driveWheel = frontLeftPID
        2 -> driveWheel = frontRightPID
        3 -> driveWheel = backLeftPID
        4 -> driveWheel = backRightPID
        else -> driveWheel = frontLeftPID
    }
}