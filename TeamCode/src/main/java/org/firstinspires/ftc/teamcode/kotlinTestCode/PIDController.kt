package org.firstinspires.ftc.teamcode.kotlinTestCode


class PIDController {

    var lastTime: Long = 0
    var Input: Float = 0.0f
    var Output: Float = 0.0f
    var Setpoint: Float = 0.0f
    var ITerm: Float = 0.0f
    var lastInput: Float = 0.0f
    var kp: Float = 0.0f
    var ki: Float = 0.0f
    var kd: Float = 0.0f
    var SampleTime: Int = 100 // 0.1 sec
    var outMin: Float = -1.0f
    var outMax: Float = 1.0f
    var inAuto: Boolean = false

    fun Compute() {

        if (!inAuto) return
        val now: Long = System.currentTimeMillis()
        val timeChange: Int = (now - lastTime) as Int
        if (timeChange >= SampleTime) {

            val error: Float = Setpoint - Input
            ITerm += (ki * error)
            if (ITerm > outMax) ITerm = outMax
            else if (ITerm < outMin) ITerm = outMin
            var dInput: Float = Input - lastInput

            Output = kp * error + ITerm - kd * dInput
            if (Output > outMax) Output = outMax
            else if (Output < outMin) Output = outMin

            lastInput = Input;
            lastTime = now
        }
    }

    fun SetTunings(Kp: Float, Ki: Float, Kd: Float) {
        val SampleTimeInSec: Float = (SampleTime as Float)/1000
        kp = Kp
        ki = Ki * SampleTimeInSec
        kd = Kd / SampleTimeInSec
    }

    fun SetSampleTime(NewSampleTime: Int) {
        if (NewSampleTime > 0) {
            val ratio: Float = NewSampleTime as Float / SampleTime as Float
            ki *= ratio
            kd /= ratio
            SampleTime = NewSampleTime
        }
    }

    fun SetOutputLimits(Min: Float, Max: Float) {
        if (Min > Max) return
        outMin = Min
        outMax = Max

        if (Output > outMax) Output = outMax
        else if (Output < outMin) Output = outMin

        if (ITerm > outMax) ITerm = outMax
        else if (ITerm < outMin) ITerm = outMin
    }

    fun SetMode(putInAuto: Boolean){
        if( putInAuto && !inAuto) {
            Initialize()
        }
        inAuto = putInAuto
    }

    fun Initialize() {
        lastInput = Input
        ITerm = Output
        if(ITerm > outMax) ITerm = outMax
        else if(ITerm < outMin) ITerm = outMin
    }
}