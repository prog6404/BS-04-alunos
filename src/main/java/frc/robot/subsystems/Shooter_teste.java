// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_teste extends SubsystemBase {
  WPI_VictorSPX leftMotor; 
  WPI_VictorSPX rightMotor;
  SpeedControllerGroup motores;

  /** Creates a new Shooter_teste. */
  public Shooter_teste() {
    leftMotor = new WPI_VictorSPX(12);
    rightMotor = new WPI_VictorSPX(14);
    motores = new SpeedControllerGroup(leftMotor, rightMotor);
  }

  public void shoot(double speed) {
    motores.set(speed);
  }

  public void stop() {
   
  }


}
