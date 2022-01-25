/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter_teste;
public class RobotContainer {

  // #region VARIABLES, SUBSYSTEMS, CONTROLLERS, ...
  // controles do piloto e copiloto
  private XboxController pilot;// , COpilot;

  // Botoões utilizados do piloto
  private JoystickButton pilot_RB, pilot_ButtonA, pilot_ButtonX;

  // Botoões utilizados do copiloto
  // private JoystickButton CO_ButtonX, CO_ButtonB, CO_ButtonY;

  Timer t;

  // Sensores
  private AHRS m_navx;
  private Gyro gyro;
  // This must be refactored
  // private Encoder_AMT103 encoderT1;
  // private PowerDistributionPanel m_PDP;

  // Subsystemas
  // private final Climb m_Climb = new Climb();
  private Drivetrain m_DriveTrain;
  private Shooter_teste m_shooter;
  // private final Shooter m_Shooter = new Shooter();
  // private final Storage m_Storage = new Storage();

  // MOTORS
  WPI_VictorSPX intake;

  // SHUFFLEBOARD
  // #endregion

  // #region CONSTRUCTOR
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    init();
    setControllers();
    configureButtonBindings();
  }
  // #endregion

  // #region SET CONTROLLERS
  /**
   * Função para criar os controles e botões
   */
  public void setControllers() {
    pilot = new XboxController(Constants.OI_Map.PILOT);
    // pilot_RB = new JoystickButton(pilot, Constants.OI_Map.BUTTON_RIGHT);
    pilot_ButtonA = new JoystickButton(pilot, Constants.OI_Map.BUTTON_A);
    // pilot_ButtonX = new JoystickButton(pilot, Constants.OI_Map.BUTTON_X);
    // pilot_ButtonB = new JoystickButton(pilot, Constants.OI_Map.BUTTON_B);

    // COpilot = new XboxController(Constants.OI_Map.COPILOT);
    // CO_ButtonX = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_X);
    // CO_ButtonB = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_B);
    // CO_ButtonY = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_Y);
  }
  // #endregion

  // #region BUTTON BINDINGS
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * Serve para setar as bindings de cada botão
   */
  private void configureButtonBindings() {

    /*
     * CO_ButtonX.whenPressed(()-> intake.set(Constants.INTAKE_SPEED), m_Storage)
     * .whenReleased(() -> intake.set(0), m_Storage);
     */

    // pilot_ButtonA.whileHeld(() -> m_DriveTrain.arcadeDrive(0.0, 1.0),
    // m_DriveTrain);

    
     /*
     * CO_ButtonB.whileHeld(()-> { if(-COpilot.getY(GenericHID.Hand.kLeft) > 0.2){
     * m_Climb.climb(); } else if(COpilot.getY(GenericHID.Hand.kLeft) > 0.2){
     * m_Climb.inverseClimb(); } else m_Climb.stopClimb(); }, m_Climb)
     * .whenReleased(()-> m_Climb.stopClimb(), m_Climb);
     */

    // Botão INTAKE
    
      pilot_ButtonA.whenPressed(() ->
      intake.set(-Constants.INTAKE_SPEED)) .whenReleased(() ->
      intake.set(0));
    /*  
      // Botão STORAGE/CONVEYOR pilot_ButtonX.whileHeld(() ->
      m_Storage.MoveBelt(Constants.STORAGE_BELT_SPEED), m_Storage) .whenReleased(()
      -> m_Storage.MoveBelt(0), m_Storage);
      
      // Botão SHOOTER pilot_RB.whileHeld(new Shoot(m_Shooter, m_Storage, m_PDP))
      .whenReleased(() -> m_Shooter.Stop());
    //*/ 
  }
  // #endregion

  public void init() {
    // MOTORS
    t = new Timer();
    intake = new WPI_VictorSPX(Constants.Ports.Motors.INTAKE_COLLECTOR);
    m_shooter = new Shooter_teste();
    // SENSORS
    m_navx = new AHRS(SerialPort.Port.kUSB);
    m_DriveTrain = new Drivetrain(m_navx);
    gyro = new ADXRS450_Gyro();
    // m_PDP = new PowerDistributionPanel(Constants.Ports.Sensors.PDP_PORT);
    // COMMANDS

  }

  // #region AUTONOMOUS
  public Command getAutonomousCommand() {
    m_navx.reset();
    m_DriveTrain.resetEncoders();


    return new Drive(m_DriveTrain).andThen();
  }
  // #endregion

  // #region CALL BINDERS

  /**
   * Região para chamar as binders do robô após autonomo
   */
  public void callBinders() {

    // SET DRIVETRAIN COMMANDS
    m_DriveTrain.setDefaultCommand(new RunCommand(() -> {
      m_DriveTrain.arcadeDrive(pilot.getY(GenericHID.Hand.kLeft), pilot.getX(GenericHID.Hand.kRight));
      
    }, m_DriveTrain));
    

    
    m_shooter.setDefaultCommand(new RunCommand(() -> {
      if(pilot.getTriggerAxis(Hand.kRight)>0) {
        t.start();
        m_shooter.shoot(t.get()/2);
      } else {
        t.stop();
        t.reset();
        m_shooter.shoot(0);
      }
    },m_shooter));

  
  }
  // #endregion
}
