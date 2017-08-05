package org.firstinspires.ftc.teamcode.kotlinTestCode

import com.cout970.vector.impl.Vector2d
import com.qualcomm.hardware.adafruit.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation

class AutonomousHardware: BaseMecanumHardware()  {

    var imu: BNO055IMU? = null; private set

    private var currentOrientation: Orientation? = null
    val currentHeading get() = getHeading()

    private var timer: ElapsedTime = ElapsedTime()

    var loopFunction: () -> Unit = {}

    override fun init(ahwMap: HardwareMap, telemetry: Telemetry) {
        super.init(ahwMap, telemetry)

        // Set up the parameters with which we will use our IMU.
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU::class.java, "imu")
        imu?.initialize(parameters)
    }


    private fun getHeading(): Float {
        if (currentOrientation == null || currentOrientation?.acquisitionTime != System.nanoTime()) {
            currentOrientation = imu?.getAngularOrientation()?.toAxesReference(AxesReference.INTRINSIC)?.toAxesOrder(AxesOrder.ZYX)
        }
        var angle = AngleUnit.DEGREES.fromUnit(currentOrientation?.angleUnit, currentOrientation?.firstAngle ?: 0.0f)
//        if (angle > 180) {
//            angle = -180 + angle % 180
//        }
        return angle
    }

    fun driveForSecs(direction: Vector2d, time: Double, rotation: Double = 0.0) {
        val startTime: Double = timer.seconds()

        moveRobot(direction, rotation)
        var currentTime: Double = timer.seconds()
        while (currentTime - startTime < time) {
            loopFunction()
            currentTime = timer.seconds()
        }
    }

    fun turnToHeading(target: Double, power: Double = 0.5, direction: Vector2d = Vector2d(0.0, 0.0)) {
        var distance = target - currentHeading
        var hDirection = if (distance > 0) 1 else -1
        if (Math.abs(distance) > 180) hDirection *= -1
        var numTargetPassed = 1

        while (Math.abs(distance) > 0.5) {
            loopFunction()
            distance = target - currentHeading
            hDirection = if (distance > 0) 1 else -1
            if (Math.abs(distance) > 180) {hDirection *= -1; if(power/numTargetPassed > 0.1) numTargetPassed++}
            moveRobot(direction, power * hDirection /numTargetPassed)
        }
    }
}