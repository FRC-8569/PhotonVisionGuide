// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Shooter;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Shooter shooter =  Shooter.getInstance();
  public Auto auto = new Auto(drivetrain, shooter);
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
  }

  public Command getAutonomousCommand() {
    return auto.getAuto(true);
  }
}
