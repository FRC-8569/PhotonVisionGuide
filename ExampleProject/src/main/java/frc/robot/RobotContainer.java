// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Drivetrain.Drivetrain;
import frc.utils.FieldPieces;
import frc.utils.FieldPieces.ReefSide;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public XboxController controller = new XboxController(0);
  public double SpeedMode = 1;

  public RobotContainer() {
    if(RobotBase.isReal()){
      drivetrain.setDefaultCommand(drivetrain.drive(
        () -> controller.getLeftY() * SpeedMode,
        () -> controller.getRightX() * SpeedMode * 0.6
      ));
    }else{
      drivetrain.setDefaultCommand(drivetrain.drive(
        () -> controller.getRawAxis(0) * SpeedMode,
        () -> controller.getRawAxis(1) * SpeedMode * 0.6
      ));
    }
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(() -> controller.getRawButton(1))
      .onTrue(drivetrain.drive(FieldPieces.CoralStation, ReefSide.NULL));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
