// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class turnCommand extends PIDCommand {
  private static boolean finished = false, start = false;

  /** Creates a new turnCommand. */
  public turnCommand(Drivetrain m_Drivetrain, double angle) {
    super(
        // The controller that the command will use
        new PIDController(Constants.Autonomous.kPDriveVel, 0, 0.001),
        // This should return the measurement
        () -> m_Drivetrain.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          m_Drivetrain.arcadeDrive(0, output);
          if(Math.abs(output) < 0.005) {
            finished = true;
          }
        });
    addRequirements(m_Drivetrain);
    m_Drivetrain.resetNavx();
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
