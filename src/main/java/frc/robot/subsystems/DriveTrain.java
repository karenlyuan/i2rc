// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _righttDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private DifferentialDrive _diffDrive;

  private double circumference = 1;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _righttDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    _leftDriveTalon.setInverted(false);
    _righttDriveTalon.setInverted(false);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _righttDriveTalon);

    _leftDriveTalon.configFactoryDefault();
    _leftDriveTalon.setInverted(false);
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    _righttDriveTalon.configFactoryDefault();
    _righttDriveTalon.setInverted(false);
    _righttDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double turn) {
    _diffDrive.arcadeDrive(speed, turn);
  }

  public void resetEncoders() {
    _leftDriveTalon.setSelectedSensorPosition(0,0,10);
    _righttDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getPosition() {
    return ((_leftDriveTalon.getSelectedSensorPosition() + _righttDriveTalon.getSelectedSensorPosition())/2) * (circumference/4096);
    //average distance of both left and right
  }

  public double getVelocity(){
    return ((_leftDriveTalon.getSensorCollection().getPulseWidthVelocity() + _righttDriveTalon.getSensorCollection().getPulseWidthVelocity())/2) * (circumference/4096);
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public void navxReset() {
    navx.reset();
  }

}
