// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class SwerveModule {
   
  private static final double kModuleMaxAngularVelocity = 20*Constants.kTurnMotorMaxAngSpeed;
  private static final double kModuleMaxAngularAcceleration =  80*Constants.kTurnMotorMaxAngSpeed*2;

  public final SparkMax m_driveMotor;
  public final SparkMaxConfig m_driveConfig = new SparkMaxConfig();
  public final SparkMax m_turningMotor;
  public final SparkMaxConfig m_turningConfig = new SparkMaxConfig();

  public final RelativeEncoder m_driveEncoder;
  public final DutyCycleEncoder m_turningEncoder;
  private double m_turningEncoderOffset;

  public double driveMotorVoltage = 0;
  public double turnMotorVoltage = 0;

  public double driveActMotorVoltage = 0;
  public double turnActMotorVoltage = 0;

  public SwerveModuleState optState;

  // Gains are for example purposes only - must be determined for your own robot!
  public final PIDController m_drivePIDController = new PIDController(2, 0, 0);  //.05, 0, 0

  // Gains are for example purposes only - must be determined for your own robot!         //1, 0, 0
  public final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          4,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 0);   //1, 3
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);  //1, 0.5

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param turningEncoderChannel DIO input for the turning encoder channel
   * @param turningEncoderOffset Position offset for the turning encoder

   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double turningEncoderOffset) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
      m_driveConfig.inverted(true);
      m_driveConfig.encoder.positionConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio);
      m_driveConfig.encoder.velocityConversionFactor(2 * Math.PI * Constants.kWheelRadius / Constants.kGearRatio / 60);   
      m_driveMotor.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
      m_turningConfig.inverted(true);
      m_turningMotor.configure(m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_driveEncoder = m_driveMotor.getEncoder(); 
    m_turningEncoder = new DutyCycleEncoder(turningEncoderChannel);


    // Set the range of the turning encoder
    // Per REV docs, 1 - 1023 is the range of minimum & maximum (less 1us) pulses, 1025 is the output period
    // 1us is minimum (0 deg) & 1024us (360 deg) is the maximum pulse width
    m_turningEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //BREAKS in 2025// m_turningEncoder.setDistancePerRotation(2.0*Math.PI);   

    // Set the position offset for each Turning Encoder
    //BREAKS in 2025 // m_turningEncoder.setPositionOffset(turningEncoderOffset);
    m_turningEncoderOffset = turningEncoderOffset;

    
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(getAdjustedAngle()));  //getAbsolutePosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(getAdjustedAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getAdjustedAngle());

    // Optimize the reference state to avoid spinning further than 90 degrees
    optState = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    optState.speedMetersPerSecond *= optState.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), optState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(optState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getAdjustedAngle(), optState.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //Publish the motor control data for loggin
    putMotorVoltageData(driveOutput + driveFeedforward, turnOutput+turnFeedforward);
    
    /* This is the final output to the motors */
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public double getAdjustedAngle()
  {
    //I think the turning encoder is set up above with setting offset + dutycyclerange
    // THIS BREAKS in 2025 return ((m_turningEncoder.getAbsolutePosition()-m_turningEncoder.getPositionOffset())*2*Math.PI);
    return m_turningEncoder.get() - m_turningEncoderOffset;
  }

  public double getAbsAngle()
  {
    return m_turningEncoder.get();
  }

  public void putMotorVoltageData(double drive, double turn)
  {
    driveMotorVoltage = drive;
    turnMotorVoltage = turn;
  }

  public double[] getMotorVoltageData()
  {
    double[] temp = {driveMotorVoltage, turnMotorVoltage};
    return temp;
  }

  public void putActualMotorVoltageData(double drive, double turn)
  {
    driveActMotorVoltage = drive;
    turnActMotorVoltage = turn;
  }

  public double[] getActualMotorVoltageData()
  {
    double[] temp = {driveActMotorVoltage, turnActMotorVoltage};
    return temp;
  }

  public SwerveModuleState getOptState()
  {
    return optState;
  }
}
