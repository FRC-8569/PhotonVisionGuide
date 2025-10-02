package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static final int[] Motors = {11,12,13,14};
    public static final double GearRatio = 10.71;
    public static final double WheelCirc = Inches.of(6).times(Math.PI).in(Meters);
    public static final double PositionConvertionFactor = 1/GearRatio*WheelCirc;
    public static final double VelocityConvertionFactor = PositionConvertionFactor/60/60;
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Centimeters.of(65));
    public static final Pose2d InitialPose = new Pose2d(3,7, Rotation2d.kZero);

    public static final ClosedLoopConfig LeftPIDConfig = new ClosedLoopConfig()
        .pidf(0.255, 0, 0.05, 1.0 / 442.5) //the ff is 1/Motor kV which units rpm/V
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    public static final ClosedLoopConfig RightPIDConfig = new ClosedLoopConfig()
        .pidf(0.255, 0, 0.05, 1.0 / 442.5) 
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    //for simulation use
    public static final double MaxVelocity = 5310*VelocityConvertionFactor;
    public static final PathConstraints constraints = PathConstraints.unlimitedConstraints(12);
}
