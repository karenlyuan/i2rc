// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GyroTurn extends CommandBase {

  DriveTrain d;
  private double speed;
  private double angle;

  /** Creates a new EncoderDrive. */
  public GyroTurn(DriveTrain d, double angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.d = d;
    this.angle = angle;
    this.speed = speed;

    addRequirements(d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    d.navxReset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(angle < 0) {
      d.tankDrive(-speed, speed);
    } else if (angle > 0) {
      d.tankDrive(speed, -speed);
    } else {
      d.tankDrive(0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    d.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(d.getAngle()) >= (Math.abs(angle));
  }

}
