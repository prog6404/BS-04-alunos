/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Drivetrain extends PIDSubsystem {

  //#region Sensors, controllers & etc.
  private Spark _leftFront, _leftBack, _rightBack, _rightFront;
  private SpeedControllerGroup _left, _right;
  private DifferentialDrive m_drive;
  private AHRS _navX;

  private final DifferentialDriveOdometry m_odometry;
  //ENCODERS
  Encoder encoderLeft, encoderRight;

  //SHUFFLEBOARD
  private ShuffleboardTab drivetrainTab;
  private NetworkTableEntry encoderLeftEntry, encoderRightEntry, pose2dX, pose2dY,navXdata;
  //#endregion
  
  public Drivetrain(AHRS navX) {
    super(new PIDController(Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_kD));

    // Sensors
    _navX = navX;

    // Left encoder

    encoderLeft = new Encoder(9, 7, Constants.Sensors_Values.ENC_LEFT_REVERSE);
    encoderLeft.setDistancePerPulse(1/2048.0);
    encoderLeft.setMinRate(1);
    encoderLeft.setSamplesToAverage(5);

    // Right encoder
    encoderRight = new Encoder(Constants.Ports.Sensors.DRIVE_ENC_RIGHT_A, Constants.Ports.Sensors.DRIVE_ENC_RIGHT_B, Constants.Sensors_Values.ENC_RIGHT_REVERSE);
    encoderRight.setDistancePerPulse(Constants.Sensors_Values.ENC_DISTANCE_PER_PULSE);
    encoderRight.setMinRate(1.0);
    encoderRight.setSamplesToAverage(5);

    // Sparks
    _leftFront = new Spark(Constants.Ports.Motors.DRIVE_LEFT_FRONT);
    _leftBack = new Spark(Constants.Ports.Motors.DRIVE_LEFT_BACK);

    _rightFront = new Spark(Constants.Ports.Motors.DRIVE_RIGHT_FRONT);
    _rightBack = new Spark(Constants.Ports.Motors.DRIVE_RIGHT_BACK);

    // ControllerGroup
    _left = new SpeedControllerGroup(_leftFront, _leftBack);
    _right = new SpeedControllerGroup(_rightFront, _rightBack);

    // DifferentialDrive
    m_drive = new DifferentialDrive(_left, _right);

    //Odometria
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    //Shuffle
    drivetrainTab = Shuffleboard.getTab("Tração");
    encoderLeftEntry = drivetrainTab.add("Encoder Esquerdo", 0.0).getEntry();
    encoderRightEntry = drivetrainTab.add("Encoder Direito", 0.0).getEntry();
    pose2dX = drivetrainTab.add("Pose 2D X",0.0).getEntry();
    pose2dY = drivetrainTab.add("Pose 2D Y",0.0).getEntry();
    navXdata = drivetrainTab.add("NavX Data",0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Shuffle
    encoderLeftEntry.forceSetDouble(this.getEncoderLeft());
    encoderRightEntry.forceSetDouble(this.getEncoderRight());
    pose2dX.forceSetDouble(m_odometry.getPoseMeters().getX());
    pose2dY.forceSetDouble(encoderLeft.getRate());
    double rates = (encoderLeft.getRate()+
    encoderLeft.getRate()+
    encoderLeft.getRate()+
    encoderLeft.getRate()+encoderLeft.getRate())/5*.8;
    navXdata.forceSetDouble(rates);

    // Atualização de odometria para PathWeaver
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), encoderLeft.getDistance(), encoderRight.getDistance());
  }

  //#region Funções utilitárias para PathWeaver

  /**
   * Obtém a posição do robo em 2d
   * @return a posição do robo em 2d
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Define o máximo da saída
   * @param maxOutput máximo a ser definido
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Reset caso bugado
   */
  public void zeroHeading() {
    _navX.reset();
  }

  /**
   * Obtém a velocidade das rodas do robô
   * @return a velocidade das rodas do robô
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderLeft.getRate(), encoderRight.getRate());
  }

  /**
   * Define a voltagem da direção
   * @param leftVolts voltagem do grupo esquerdo
   * @param rightVolts voltagem do grupo direito
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _left.setVoltage(leftVolts);//rightVolts);
    _right.setVoltage(-rightVolts);//-leftVolts);
    m_drive.feed();
  }

  public void resetNavx() {
    _navX.reset();
  }

  /**
   * reseta a odometria
   * @param pose posição atual
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  //#endregion

  //#region SENSORS

  /**
   * Gets the value of the right encoder
   * @return the value of the right encoder
   */
  public double getEncoderRight() {
    return encoderRight.getDistance();
  }

  /**
   * Gets the value of the left encoder
   * @return the value of the left encoder
   */
  public double getEncoderLeft() {
    return encoderLeft.getDistance();
  }

  /**
   * Gets the value of the mean of both encoders encoder
   * @return the value of the mean of both encoders encoder
   */
  public double getAverageEncoderDistance() {
    return (encoderLeft.getDistance() + encoderRight.getDistance()) / 2.0;
  }

  /**
   * Gets the value of the heading of the robot
   * @return the value of the heading of the robot
   */
  public double getHeading() {
    return _navX.getAngle();//Math.IEEEremainder(_navX.getYaw(), 360); //* -1.0d;
  }

  /**
   * Gets the turn rate
   * @return the turn rate
   */
  public double getTurnRate() {
    return _navX.getRate();
  }

  /**
   * resets the encoders
   */
  public void resetEncoders() {
    encoderLeft.reset();
    encoderRight.reset();
  }
  //#endregion

  //#region ARCADE DRIVE

  /**
   * drives the robot based in fwd (forward) and
   * @param fwd value from 1 to 0 for the robot to move forward
   * @param rot value from 1 to 0 for the robot to rotate the robot
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(-fwd, rot);
  }

  /**
   * stops the robot
   */
  public void stop(){
    m_drive.arcadeDrive(0, 0);
  }
  //#endregion

  @Override
  protected void useOutput(double output, double setpoint) {
    Shuffleboard.getTab("DriveTrain").addNumber("PID Output", () -> output);
  }

  @Override
  protected double getMeasurement() {
    return _navX.getYaw();
  }

  
}
