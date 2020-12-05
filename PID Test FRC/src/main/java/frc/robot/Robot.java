/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

/**
 * This is a sample program to demonstrate the use of a PIDController with an
 * ultrasonic sensor to reach and maintain a set distance from an object.
 */
public class Robot extends TimedRobot {
  // distance in inches the robot wants to stay from an object
  public static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  public static final double kValueToInches = 0.125;

  // proportional speed constant
  public static final double kP = 7.0;

  // integral speed constant
  public static final double kI = 0.018;

  // derivative speed constant
  public static final double kD = 1.5;

  //position and velocity tolerance range of pid controller
  public static final double posTolerance = 5;
  public static final double veloTolerance = 10;

  public static final int kLeftMotorPort = 0;
  public static final int kRightMotorPort = 1;
  public static final int kUltrasonicPort = 0;
  public static final int kDriverControllerPort = 2;

  // median filter to discard outliers; filters over 5 samples
  public static MedianFilter m_filter = new MedianFilter(5);

  public static AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  public static CANSparkMax leftSparkMax = new CANSparkMax(kLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static CANSparkMax rightSparkMax = new CANSparkMax(kRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static DifferentialDrive m_diffDrive = new DifferentialDrive (leftSparkMax, rightSparkMax); 
  public static PIDController m_pidController = new PIDController(kP, kI, kD);
  public static Joystick driverController = new Joystick(kDriverControllerPort);

  @Override
  public void teleopInit() {
    // Set setpoint of the pid controller
    m_pidController.setSetpoint(kHoldDistance * kValueToInches);

    //set tolerance range of pid controller
    m_pidController.setTolerance(posTolerance, veloTolerance);
  }

  @Override
  public void teleopPeriodic() {
    // returned value is filtered with a rolling median filter
    double pidOutput = m_pidController.calculate(m_filter.calculate(m_ultrasonic.getVoltage()));
    m_diffDrive.arcadeDrive(pidOutput, 0);
  }
}
