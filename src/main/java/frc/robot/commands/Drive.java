// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  // variaveis de criação
  // subsistema requirido
  Drivetrain drivetrain;
  // temporizador para termino do commando
  Timer t;
  //Basiamente para ver se o robô está ligado
  boolean b;
  /** Creates a new Drive. */
  //
  public Drive(Drivetrain m_Drivetrain) {
    addRequirements(m_Drivetrain);
    drivetrain = m_Drivetrain;
    t = new Timer();
    b = false;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // resetando e iniciando o timer
    t.reset();
    t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // verificando quanto tempo se passou e caso tenha dado o tempo trocar para verdade a variavel de termino
    if(t.get() < 5.5) {
      drivetrain.arcadeDrive(-0.7, -0.0);
    } else if (t.get() < 6.6) {
      drivetrain.arcadeDrive(-0.8, -0.8);
    } else if (t.get() < 8.1) {
      drivetrain.arcadeDrive(-0.6, -0.0);
    } else if (t.get() < 9.2) {
      drivetrain.arcadeDrive(-0.8, -0.8);
    } else if (t.get() < 13) {
      drivetrain.arcadeDrive(-0.8, -0.1);
    }
    else {
      b = true;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // resetando tudo para valor padrão
    b = false;
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return b;
  }
}
