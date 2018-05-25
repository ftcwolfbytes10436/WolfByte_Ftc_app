package org.firstinspires.ftc.teamcode.relicRecovery2017;

/**
 * Created by Andrew Brown on 8/18/17.
 */


public class PID_Test_Andrew {

    double mError;
    double mSumError;
    double mLastError;
    double pCorrection;
    double iCorrection;
    double dCorrection;
    double slope;

    public void init(){
        mError = 0;
        mSumError = 0;
        mLastError = 0;
    }

    public double PIDloop(double pVal, double iVal, double dVal, double CurrentValue, double TargetValue, double MaxCorr, double MinCorr, double feedForward){
        mError = TargetValue - CurrentValue;
        pCorrection = pVal * mError;

        mSumError = mSumError + mError;
        iCorrection = iVal * mSumError;

        slope = mError - mLastError;
        dCorrection = slope * dVal;
        mLastError = mError;

        double correction = pCorrection + iCorrection + dCorrection;

        if (correction > MaxCorr) correction = MaxCorr;
        if (correction < MinCorr) correction = MinCorr;

        return correction + feedForward;
    }
}