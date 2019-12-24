package frc.robot.statemachines;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Util;
import frc.robot.statemachines.IntakeStateMachine.IntakeState;

@RunWith(JUnit4.class)
public class TestIntakeStateMachine {
    IntakeStateMachine sm = new IntakeStateMachine();
    IntakeStateMachine.WantedState action = IntakeStateMachine.WantedState.IDLE;
    IntakeState simState = new IntakeState();

    @Test
    public void testBoot() {
        IntakeState commandedState = sm.update(0, action, simState);
        Assert.assertEquals("motor must default to 0", 0, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.OPEN);
    }

    @Test
    public void testGoesToIntaking() {
        action = IntakeStateMachine.WantedState.INTAKING;
        IntakeState commandedState = sm.update(0, action, simState);
        Assert.assertEquals(IntakeStateMachine.kIntakePower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);
    }

    @Test
    public void testGoesToIdle() {
        action = IntakeStateMachine.WantedState.IDLE;
        IntakeState commandedState = sm.update(0, action, simState);
        Assert.assertEquals(0, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.OPEN);
    }

    @Test
    public void testDoesClampClean() {
        double ts = Timer.getFPGATimestamp();
        action = IntakeStateMachine.WantedState.INTAKING;

        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kIntakePower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kIntakePower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 3
        ts += 0.005;
        simState.hasCube = true;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kHoldPower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
    }

    @Test
    public void testCanPlace() {
        double ts = Timer.getFPGATimestamp();
        action = IntakeStateMachine.WantedState.INTAKING;

        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kIntakePower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        ts += 0.005;
        simState.hasCube = true;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kHoldPower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 3 - try to place
        action = IntakeStateMachine.WantedState.PLACE;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(0.0, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.OPEN);

        // iter 4 - jump forward in time
        ts += IntakeStateMachine.kPlaceDelay;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(0.0, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.OPEN);
    }

    @Test
    public void testCanShoot() {
        double ts = Timer.getFPGATimestamp();
        action = IntakeStateMachine.WantedState.INTAKING;

        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kIntakePower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        simState.hasCube = true;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(IntakeStateMachine.kHoldPower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 3 - try to shoot
        action = IntakeStateMachine.WantedState.SHOOT;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals("motor must shoot", IntakeStateMachine.kShootPower, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 4 - jump forward in time
        ts += IntakeStateMachine.kShootDelay;
        commandedState = sm.update(ts, action, simState);
        Assert.assertEquals(0.0, commandedState.motor, Util.kEpsilon);
        Assert.assertEquals(commandedState.jawState, IntakeState.JawState.OPEN);
    }
}