/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.StringJoiner;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
// import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
 * Wrap up some limelight-related stuff; move to subsystem
 */
final class LimeLightSubsystem {
  private final NetworkTable limeLight = NetworkTableInstance.getDefault().getTable("limeLight");

  public LimeLightSubsystem(int pipeline) {
    limeLight.getEntry("pipeline").setNumber(pipeline);
  }

  public LimeLightSubsystem() {
    this(0);
  }

  public boolean hasTarget() {
    return limeLight.getEntry("tv").getDouble(0.0) > 0.0;
  }

  public double getX() {
    return limeLight.getEntry("tx").getDouble(0.0);
  }

  public double getY() {
    return limeLight.getEntry("ty").getDouble(0.0);
  }

  public double getArea() {
    return limeLight.getEntry("ta").getDouble(0.0);
  }

}

public class Robot extends TimedRobot {
  private XboxController driverXbox;

  private DifferentialDrive m_myRobot;
  private LimeLightSubsystem m_limeLight;

  private final CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(4, MotorType.kBrushless);

  @Override
  public void robotInit() {
    
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);
    // Should be able to pass in Master for each side rather than group explicitly
    // SpeedControllerGroup leftGroup = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
    // SpeedControllerGroup rightGroup = new SpeedControllerGroup(frontRightMotor, backRightMotor);

    m_myRobot = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    m_limeLight = new LimeLightSubsystem();
    driverXbox = new XboxController(1);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Limelight has target", m_limeLight.hasTarget());
    SmartDashboard.putNumber("Limelight tx", m_limeLight.getX());
    SmartDashboard.putNumber("Limelight ty", m_limeLight.getY());
    SmartDashboard.putNumber("Limelight Area", m_limeLight.getArea());
  }

  @Override
  public void teleopPeriodic() {
    final double K = 0.3;
    final double leftSpeed = driverXbox.getRawAxis(1);
    final double rightSpeed = driverXbox.getRawAxis(5);
    m_myRobot.tankDrive(leftSpeed * K, rightSpeed * K);
  }

  @Override
  public void autonomousPeriodic() {
    final double K_TURN = 0.02;
    final double MAX_SPEED = 0.25;

    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    if (this.isOperatorControl()) {
      System.out.println("INFO: autonomousPeriodic called during teleop");
      return;
    }

    if (m_limeLight.hasTarget()) {
      leftSpeed = 0.0;
      rightSpeed = 0.0;
    } else {
      double speed = m_limeLight.getX() * K_TURN;
      speed = MathUtil.clamp(speed, - MAX_SPEED, MAX_SPEED);
      leftSpeed = speed;
      rightSpeed = -speed;
    }
    m_myRobot.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void disabledInit() {
    // Ensure stopped on mode change
    m_myRobot.tankDrive(0.0, 0.0);
  }

  @Override
  public void testInit() {
    // Ensure stopped on mode change
    m_myRobot.tankDrive(0.0, 0.0);
  }

}