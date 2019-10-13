/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {
	private Drive mDrive;
	
	private Joystick mThrottleStick;
	private Joystick mTurnStick;

	@Override
	public void robotInit() {
		mDrive = new Drive();

		mThrottleStick = new Joystick(Constants.kThrottleStickPort);
		mTurnStick = new Joystick(Constants.kTurnStickPort);
	}

	@Override
	public void autonomousInit() {
		mDrive.stop();
	}

	@Override
	public void disabledInit() {
		mDrive.stop();
	}

	@Override
	public void testInit() {
		mDrive.stop();
	}

	@Override
	public void teleopPeriodic() {
		mDrive.setOpenLoop(mThrottleStick.getRawAxis(1), mTurnStick.getRawAxis(0));
	}
}
