// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetain;

public class SwerveTest1ManualCmd extends Command {
  /** Creates a new SwerveTest1ManualCmd. */
  private final Drivetain drivetain;
  private final CommandXboxController main;

  public SwerveTest1ManualCmd(Drivetain drivetain, CommandXboxController main) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetain = drivetain;
    this.main = main;
    addRequirements(this.drivetain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetain.drive(main.getLeftX(), main.getLeftY(), drivetain.PIDcontrolRot(main.getRightX(), main.getRightY()),
    false);
    double angle = Math.atan2(main.getRightY(), main.getRightX());
    if(angle>3.0*Math.PI/2.0){
      angle = angle - 2.0*Math.PI;
    }
    SmartDashboard.putNumber("controller_turningAngle", Math.toDegrees(angle)-90);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
