// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _righttDriveTalon;

  private DifferentialDrive _diffDrive;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _righttDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);

    _leftDriveTalon.setInverted(false);
    _righttDriveTalon.setInverted(false);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _righttDriveTalon);


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
}
