package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.qualcomm.hardware.adafruit.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Position

/**
 * Created by caleb on 6/4/2017.
 * A Class that
 * This class impliments a system that tracks the position of the robot.
 * It also has functions that use the position to tell the robot where to go.
 * This class assumes that the robot has a gyro connected along with an encoder connected to each wheel.
 */
class FPS {

    /* Public OpMode members. */
    var imu: BNO055IMU? = null; private set
    var headingOffset = 0
    val currentHeading: Double get() {
        val currentHeading = imu?.getAngularOrientation()?.toAxesReference(AxesReference.INTRINSIC)?.toAxesOrder(AxesOrder.ZYX)
        var angle = AngleUnit.DEGREES.fromUnit(currentHeading?.angleUnit, currentHeading?.firstAngle ?: 0.0f) + headingOffset
        if (angle > 180) angle = -180 + angle % 180
        return angle as Double
    }

    val fLEncoder: Int get() = hardware?.frontLeftMotor?.currentPosition ?: 0
    val fREncoder: Int get() = hardware?.frontRightMotor?.currentPosition ?: 0
    val bLEncoder: Int get() = hardware?.backLeftMotor?.currentPosition ?: 0
    val bREncoder: Int get() = hardware?.backRightMotor?.currentPosition ?: 0

    var position: Position = Position(); private set

    /* local OpMode members. */
    private var hardware: BaseMecanumHardware? = null
    private lateinit var telemetry: Telemetry
    private lateinit var hwMap : HardwareMap

    /* Initialize standard Hardware interfaces */
    fun init(ahwMap: HardwareMap, telemetry: Telemetry, hardware: BaseMecanumHardware) {

        // assign private variables
        hwMap = ahwMap
        this.telemetry = telemetry
        this.hardware = hardware

        // set the motors to all use encoders
        hardware.frontLeftMotor?.mode  = DcMotor.RunMode.RUN_USING_ENCODER
        hardware.frontRightMotor?.mode = DcMotor.RunMode.RUN_USING_ENCODER
        hardware.backLeftMotor?.mode   = DcMotor.RunMode.RUN_USING_ENCODER
        hardware.backRightMotor?.mode  = DcMotor.RunMode.RUN_USING_ENCODER

        // Set up the parameters with which we will use our IMU.
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"

        // Retrieve the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        var imuTemp: BNO055IMU? = hwMap.get(BNO055IMU::class.java, "imu")

        // Check to make sure the IMU variable is not null and assign the IMU to a class variable
        // else if it is null send a message to telemetry
        if (imuTemp  != null) imu  = imuTemp  else telemetry.addData("error", "imu not found")

        // Initialize the IMU
        imu?.initialize(parameters)
    }

    /* method to update the tracking for the FPS */
    fun updateFPS() {

    }
}