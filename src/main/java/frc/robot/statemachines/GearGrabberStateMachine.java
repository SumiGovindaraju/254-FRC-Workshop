package frc.robot.statemachines;

import edu.wpi.first.wpilibj.Timer;

public class GearGrabberStateMachine {
    private final static double kGearIntakePower = -1.0;
    private final static double kGearExhaustPower = 1.0;

    private final static double kStowDelayTime = 1.0; // sec to keep intaking while in stowed position
    private final static double kShootDelayTime = 0.1; // sec to keep gear while grabber moves down before shooting
    private final static double kStowingDelayTime = 0.5; // sec to stay in stowing before shifting stowed position

    private final static boolean kSolenoidUpValue = true;

    public enum WantedState {
        IDLE,
        ACQUIRE,
        SCORE
    }

    public enum SystemState {
        STOWED,
        STOWING,
        INTAKE,
        EXHAUST
    }

    public static class GearGrabberState {
        public enum ToggleState {
            UP(kSolenoidUpValue),
            DOWN(!kSolenoidUpValue);

            private final boolean solenoid_value;

            private ToggleState(boolean val) {
                solenoid_value = val;
            }

            public boolean getSolenoidValue() {
                return solenoid_value;
            }
        }

        public ToggleState toggleState = ToggleState.UP;

        public double motor = 0.0;

        public boolean hasGear = false;

        @Override
        public String toString() {
            return "GearGrabberState{" +
                    "toggleState=" + toggleState +
                    ", motor=" + motor +
                    ", hasGear=" + hasGear +
                    '}';
        }
    }

    private SystemState mSystemState = SystemState.STOWED;
    private double mCurrentStateStartTime = 0.0;
    private GearGrabberState mDesiredState = new GearGrabberState();

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized GearGrabberState update(double timestamp, WantedState wantedState, GearGrabberState currentState) {
        SystemState newState = mSystemState;
        double timeInState = timestamp - mCurrentStateStartTime;

        switch (mSystemState) {
            case STOWED:
                newState = handleStowedTransition(wantedState);
                break;
            case STOWING:
                newState = handleStowingTransition(timeInState);
                break;
            case INTAKE:
                newState = handleIntakeTransition(wantedState, currentState);
                break;
            case EXHAUST:
                newState = handleExhaustTransition(wantedState);
                break;
            default:
                System.out.println("Unexpected gear grabber system state: " + mSystemState);
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
            case STOWED:
                mDesiredState = getStowedDesiredState(timeInState);
                break;
            case STOWING:
                mDesiredState = getStowingDesiredState();
                break;
            case INTAKE:
                mDesiredState = getIntakeDesiredState();
                break;
            case EXHAUST:
                mDesiredState = getExhaustDesiredState(timeInState);
                break;
            default:
                System.out.println("Unexpected gear grabber system state: " + mSystemState);
                break;
        }

        return mDesiredState;
    }

    private SystemState handleStowedTransition(WantedState wantedState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.STOWED;
            case ACQUIRE:
                return SystemState.INTAKE;
            case SCORE:
                return SystemState.EXHAUST;
        }

        return SystemState.STOWED;
    }

    private SystemState handleStowingTransition(double timeInState) {
        if (timeInState > kStowingDelayTime) {
            return SystemState.STOWED;
        }

        return SystemState.STOWING;
    }

    private SystemState handleIntakeTransition(WantedState wantedState, GearGrabberState currentState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.STOWING;
            default:
                if (currentState.hasGear) {
                    return SystemState.STOWING;
                }

                return SystemState.INTAKE;
        }
    }

    private SystemState handleExhaustTransition(WantedState wantedState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.STOWING;
            case ACQUIRE:
                return SystemState.INTAKE;
            case SCORE:
                return SystemState.EXHAUST;
        }

        return SystemState.STOWED;
    }

    private GearGrabberState getStowedDesiredState(double timeInState) {
        GearGrabberState ret_val = new GearGrabberState();
        ret_val.toggleState = GearGrabberState.ToggleState.UP;

        if (timeInState < kStowDelayTime) {
            ret_val.motor = kGearIntakePower;
        }

        return ret_val;
    }

    private GearGrabberState getStowingDesiredState() {
        GearGrabberState ret_val = new GearGrabberState();
        ret_val.toggleState = GearGrabberState.ToggleState.UP;
        ret_val.motor = kGearIntakePower;

        return ret_val;
    }

    private GearGrabberState getIntakeDesiredState() {
        GearGrabberState ret_val = new GearGrabberState();
        ret_val.toggleState = GearGrabberState.ToggleState.DOWN;
        ret_val.motor = kGearIntakePower;

        return ret_val;
    }

    private GearGrabberState getExhaustDesiredState(double timeInState) {
        GearGrabberState ret_val = new GearGrabberState();
        ret_val.toggleState = GearGrabberState.ToggleState.DOWN;

        if (timeInState > kShootDelayTime) {
            ret_val.motor = kGearExhaustPower;
        }

        return ret_val;
    }
}