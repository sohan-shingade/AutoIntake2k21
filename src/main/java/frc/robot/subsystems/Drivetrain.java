// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

   // real hardware
   private TalonFX leftFront; // motor on drivetrain located on the left side in the front
   private TalonFX rightFront; // motor on drivetrain located on the right side in the front
   private TalonFX leftBack; // motor on drivetrain located on the left side in the back
   private TalonFX rightBack; // motor on drivetrain located on the right side in the back
   private SupplyCurrentLimitConfiguration currentLimit;
 

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFront = new TalonFX(46);
    leftBack = new TalonFX(9);
    rightFront = new TalonFX(48);
    rightBack = new TalonFX(6);
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();

    // true: enabled, 40: 40 amp current limit, 50: If current crosses 50 amp
    // threshold trigger current limiting, 3.8: if the current exceeds threshold for
    // 3.8 seconds trigger current limiting
    currentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

    leftFront.configSupplyCurrentLimit(currentLimit, 10);
    rightFront.configSupplyCurrentLimit(currentLimit, 10);
    leftBack.configSupplyCurrentLimit(currentLimit, 10);
    rightBack.configSupplyCurrentLimit(currentLimit, 10);

    leftFront.configOpenloopRamp(0.4);
    rightFront.configOpenloopRamp(0.4);
    leftBack.configOpenloopRamp(0.4);
    rightBack.configOpenloopRamp(0.4);

    leftFront.setInverted(false);
    leftBack.setInverted(false);
    rightFront.setInverted(true);
    rightBack.setInverted(true);

    leftFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
      10);
    rightFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
      10);
    leftBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
      10);
    rightBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
      10);

    leftFront.configVoltageCompSaturation(10);
    rightFront.configVoltageCompSaturation(10);
    leftBack.configVoltageCompSaturation(10);
    rightBack.configVoltageCompSaturation(10);

    leftFront.enableVoltageCompensation(true);
    rightFront.enableVoltageCompensation(true);
    leftBack.enableVoltageCompensation(true);
    rightBack.enableVoltageCompensation(true);
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * Method for regular joystick driving during teleop through implementation of
   * the `syIshDrive, and then applies the speed to the motors
   * 
   * @param throttle  (double) power towards forward and backward movement
   * @param wheel     (double) power towards turning movement
   * @param quickTurn (boolean) status of quickTurnButton
   */
  public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn) {
    throttle = handleDeadband(throttle, Constants.kThrottleDeadband);
    wheel = handleDeadband(wheel, Constants.kWheelDeadband);

    double left = 0, right = 0;

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    Twist2d motion = new Twist2d(throttle, 0, wheel);
    if (Math.abs(motion.dtheta) < 1E-9) {
      left = motion.dx;
      right = motion.dx;
    } else {
      double delta_v = Constants.kTrackWidthInches * motion.dtheta
          / (2 * Constants.kTrackScrubFactor);
      left = motion.dx + delta_v;
      right = motion.dx - delta_v;
    }

    double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));

    setDrivetrainMotorSpeed(left / scaling_factor, right / scaling_factor);
  }

  public void setDrivetrainMotorSpeed(double left, double right) {
		leftFront.set(ControlMode.PercentOutput, left);
		leftBack.set(ControlMode.PercentOutput, left);
		rightBack.set(ControlMode.PercentOutput, right);
		rightFront.set(ControlMode.PercentOutput, right);

	}

  /**
   * Calculates the new input by the joystick after taking into account deadband
   * 
   * @param input         raw input by the joystick
   * @param inputDeadband deadband for the value sent in
   * @return finalInput input by the joystick after calculating deadband
   */
  private double handleDeadband(double input, double inputDeadband) {
    double finalInput = 0;

    if (Math.abs(input) < inputDeadband)
      finalInput = 0;
    else
      finalInput = calculateDeadband(input, inputDeadband);

    return finalInput;
  }
   /**
   * Calculates deadband throught an equation that allows low values to be reached
   * even after the deadband is applied.
   * 
   * @param input         original input before deadband
   * @param inputDeadband deadband being applied to the input
   * @return valAfterDeadband new input value after deadband
   */
  private double calculateDeadband(double input, double inputDeadband) {
    double valAfterDeadband = (input - inputDeadband * Math.abs(input) / input) / (1 - inputDeadband);
    // valAfterDeadband = (1 / (1 - inputDeadband)) * (input + (Math.signum(-input)
    // * inputDeadband));
    return valAfterDeadband;
  }

  public class Constants {
      //Constants
      //zero and ask mech for vals
      public static final double kLimelightFF = 0.042;
      public static final double kLimelightP = 0.04;

      public static final double kTrackScrubFactor = 1.0469745223;
      public static final double kTrackWidthInches = 24.2; // inches
      public static final double kTrackWidthMeters = .5883; // meters
      public static final double kWheelDiameterMeters = .158; // meters      // .1450848;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // meters
              
      public static final double kMaxVelocityMetersPerSecond = 1.5; // m/s
      public static final double kMaxAccelerationMetersPerSecondSq = 2;// m/s^2
      public static final double kDriveGearRatio = (46.0/9) * (44.0/20); // ticks
      public static final int kFalconTicksPerRotation = 2048; // ticks/rotation
              
      public static final double kDriveS = 1.0;
      public static final double kDriveV = 0.8; 
      public static final double kDriveA = 0.07;   

      public static final double kDriveP = 0; 
      public static final double kDriveI = 0;
      public static final double kDriveD = 0.0;
              
      public static final double kSensitivity = 0.90;
      public static final double kWheelDeadband = 0.02;
      public static final double kThrottleDeadband = 0.02;
              
      public static final double kRobotMass = 125.0; // kg
      public static final double kRotationalInertia = 20.0;

      public static final double kAlignP = 0.0051;
      public static final double kAlignI = 0.00051;
      public static final double kAlignD = 0.0018;
      public static final double kAlignff = 0.0033;

      public static final double kAcceptableAlignError = 1.5; 

      public static final double kIntegralRange = 1;
  }
}
