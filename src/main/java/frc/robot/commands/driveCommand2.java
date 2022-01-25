/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class driveCommand2 extends PIDCommand {

	private static boolean finished = false, start = false;
	private static int i = new Random().nextInt();

	public driveCommand2(Drivetrain m_Drivetrain, double end) {
		super(
				// The controller that the command will use
				new PIDController(Constants.Autonomous.kPDriveVel * 50, 0, 0.1),
				// This should return the measurement
				() -> {if(!start) {
					start = true;
					m_Drivetrain.resetEncoders();
				  }return m_Drivetrain.getAverageEncoderDistance();}, // m_Shooter.getGyroAngle(),
				// This should return the setpoint (can also be a constant)
				() -> end, 
				
				output -> {
					m_Drivetrain.arcadeDrive(-output/2, 0);
					SmartDashboard.putBoolean("key"+ i, finished);
					if (Math.abs(output) < 0.05) {
						finished = true;
					};
				});
		addRequirements(m_Drivetrain);
		m_Drivetrain.resetEncoders();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}
}
