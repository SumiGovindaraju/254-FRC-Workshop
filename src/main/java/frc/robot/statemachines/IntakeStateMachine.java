package frc.robot.statemachines;

import edu.wpi.first.wpilibj.Timer;

public class IntakeStateMachine {
    public static final double kIntakePower = -1.0;
    public static final double kHoldPower = -0.25;
    public static final double kShootPower = 0.85;
    public static final double kPlaceDelay = 0.5;
    public static final double kShootDelay = 1.0;

    public enum WantedState {
        IDLE,
        INTAKING,
        PLACE,
        SHOOT
    }

    public enum SystemState {
        IDLE,
        INTAKE,
        HOLD,
        PLACE,
        SHOOT
    }

    public static class IntakeState {
        public enum JawState {
            OPEN(false, false),
            CLOSED(true, false),
            CLAMPED(true, true);

            private final boolean close_solenoid_value, clamp_solenoid_value;
            
            private JawState(boolean close_val, boolean clamp_val) {
                close_solenoid_value = close_val;
                clamp_solenoid_value = clamp_val;
            }

            public boolean getCloseSolenoidValue() {
                return close_solenoid_value;
            }

            public boolean getClampSolenoidValue() {
                return clamp_solenoid_value;
            }
        }

        public JawState jawState = JawState.OPEN;
        public double motor = 0.0;
        public boolean hasCube = false;

        @Override
        public String toString() {
            return "IntakeState{" +
            "jawState=" + jawState +
            ", motor=" + motor +
            ", hasCube=" + hasCube +
            '}';
        }
    }

    private SystemState mSystemState = SystemState.IDLE;
    private IntakeState mDesiredState = new IntakeState();
    private double mCurrentStateStartTime = 0.0;

    public IntakeState update(double timestamp, WantedState wantedState, IntakeState currentState) {
        SystemState newState = mSystemState;
        double timeInState = timestamp - mCurrentStateStartTime;

        switch (mSystemState) {
            case IDLE:
                newState = handleIdleTransition(wantedState);
                break;
            case INTAKE:
                newState = handleIntakeTransition(wantedState, currentState);
                break;
            case HOLD:
                newState = handleHoldTransition(wantedState, currentState);
                break;
            case PLACE:
                newState = handlePlaceTransition(wantedState, timeInState);
                break;
            case SHOOT:
                newState = handleShootTransition(wantedState, timeInState);
                break;
            default:
                System.out.println("Unexpected intake system state: " + mSystemState);
                newState = mSystemState;
                break;
        }

        if (newState != mSystemState) {
            System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
            mSystemState = newState;
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            timeInState = 0.0;
        }

        switch (mSystemState) {
            case IDLE:
                mDesiredState = getIdleDesiredState();
                break;
            case INTAKE:
                mDesiredState = getIntakeDesiredState();
                break;
            case HOLD:
                mDesiredState = getHoldDesiredState();
                break;
            case PLACE:
                mDesiredState = getPlaceDesiredState();
                break;
            case SHOOT:
                mDesiredState = getShootDesiredState();
                break;
            default:
                System.out.println("Unexpected intake system state: " + mSystemState);
                break;
        }

        return mDesiredState;
    }

    public SystemState handleIdleTransition(WantedState wantedState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case INTAKING:
                return SystemState.INTAKE;
            case PLACE:
                return SystemState.PLACE;
            case SHOOT:
                return SystemState.SHOOT;
        }

        return SystemState.IDLE;
    }

    public SystemState handleIntakeTransition(WantedState wantedState, IntakeState currentState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case INTAKING:
            default:
                if (currentState.hasCube) {
                    return SystemState.HOLD;
                }

                return SystemState.INTAKE;
        }
    }

    public SystemState handleHoldTransition(WantedState wantedState, IntakeState currentState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case INTAKING:
                if (!currentState.hasCube) {
                    return SystemState.INTAKE;
                }

                return SystemState.HOLD;
            case PLACE:
                return SystemState.PLACE;
            case SHOOT:
                return SystemState.SHOOT;
        }

        return SystemState.HOLD;
    }

    public SystemState handlePlaceTransition(WantedState wantedState, double timeInState) {
        switch (wantedState) {
            case PLACE:
                if (timeInState > kPlaceDelay) {
                    return SystemState.IDLE;
                }
            default:
                return SystemState.PLACE;
        }
    }

    public SystemState handleShootTransition(WantedState wantedState, double timeInState) {
        switch (wantedState) {
            case SHOOT:
                if (timeInState > kShootDelay) {
                    return SystemState.IDLE;
                }
            default:
                return SystemState.SHOOT;
        }
    }

    public IntakeState getIdleDesiredState() {
        IntakeState ret_val = new IntakeState();
        ret_val.motor = 0.0;
        ret_val.jawState = IntakeState.JawState.OPEN;
        return ret_val;
    }

    public IntakeState getIntakeDesiredState() {
        IntakeState ret_val = new IntakeState();
        ret_val.motor = kIntakePower;
        ret_val.jawState = IntakeState.JawState.CLOSED;
        return ret_val;
    }

    public IntakeState getHoldDesiredState() {
        IntakeState ret_val = new IntakeState();
        ret_val.motor = kHoldPower;
        ret_val.jawState = IntakeState.JawState.CLAMPED;
        return ret_val;
    }

    public IntakeState getPlaceDesiredState() {
        IntakeState ret_val = new IntakeState();
        ret_val.motor = 0.0;
        ret_val.jawState = IntakeState.JawState.OPEN;
        return ret_val;
    }

    public IntakeState getShootDesiredState() {
        IntakeState ret_val = new IntakeState();
        ret_val.motor = kShootPower;
        ret_val.jawState = IntakeState.JawState.CLOSED;
        return ret_val;
    }
}