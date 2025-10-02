package frc.robot.Auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Shooter.Shooter;
import frc.utils.FieldPieces;
import frc.utils.FieldPieces.ReefSide;

public class Auto{
    public static Auto auto;
    public Drivetrain drivetrain;
    public Shooter shooter;

    public Auto(Drivetrain drivetrain, Shooter shooter){
        this.drivetrain = drivetrain;
        this.shooter = shooter;
    }

    private Command waitCoral(){
        return new WaitCommand(Seconds.of(1));
    }

    public Command getAuto(boolean isResetPose){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetPose(Constants.InitialPose), drivetrain),
            drivetrain.drive(FieldPieces.ReefGH, ReefSide.NULL),
            shooter.shoot(),
            drivetrain.drive(FieldPieces.CoralStation, ReefSide.NULL),
            waitCoral(),
            drivetrain.drive(FieldPieces.ReefAB, ReefSide.NULL),
            shooter.shoot(),
            drivetrain.drive(FieldPieces.CoralStation,ReefSide.NULL),
            waitCoral(),
            drivetrain.drive(FieldPieces.ReefCD,ReefSide.NULL),
            shooter.shoot()
        ).withTimeout(Seconds.of(15));
    }

}