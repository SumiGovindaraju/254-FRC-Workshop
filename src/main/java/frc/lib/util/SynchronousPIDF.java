package frc.lib.util;

/**
 * Simplified version of https://github.com/Team254/FRC-2019-Public/blob/master/src/main/java/com/team254/lib/util/SynchronousPIDF.java
 */
public class SynchronousPIDF {
    private double mP, mI, mD, mF;
    private double mSetpoint;
    private double mError, mLastError;
    private double mIntegral;
    private double mOutput;

    public SynchronousPIDF() {
        this(0.0, 0.0, 0.0);
    }

    public SynchronousPIDF(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0);
    }

    public SynchronousPIDF(double kP, double kI, double kD, double kF) {
        mP = kP;
        mI = kI;
        mD = kD;
        mF = kF;
    }

    public double update(double current_pose, double dt) {
        double mError = mSetpoint - current_pose;

        mIntegral += mError * dt;
        double derivative = (mError - mLastError) / dt;
        mOutput = mP * mError + mI * mIntegral + mD * derivative + mF * mSetpoint;

        mLastError = mError;

        return mOutput;
    }

    public double getP() {
        return mP;
    }

    public double getI() {
        return mI;
    }

    public double getD() {
        return mD;
    }

    public double getF() {
        return mF;
    }

    public double getError() {
        return mError;
    }

    public double getSetpoint() {
        return mSetpoint;
    }

    public void setP(double kP) {
        mP = kP;
    }

    public void setI(double kI) {
        mI = kI;
    }

    public void setD(double kD) {
        mD = kD;
    }

    public void setF(double kF) {
        mF = kF;
    }

    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }
}