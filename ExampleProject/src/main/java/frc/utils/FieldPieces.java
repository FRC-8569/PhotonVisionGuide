package frc.utils;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum FieldPieces {
    CoralStation(1,1),
    Processor(3,16),
    BargeBlue(4,14),
    BargeRed(5,15),
    ReefAB(7),
    ReefCD(8),
    ReefEF(9),
    ReefGH(10),
    ReefIJ(11),
    ReefKL(6);

    int tag = -1;
    int[] tags;
    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    FieldPieces(int... tags){
    }

    FieldPieces(int tag){
        this.tag = tag;
    }

    public Pose2d getPose(Pose2d currentPose){
        SmartDashboard.putNumber("PoseTarget", tag);
        if(tag != -1) return field.getTagPose(tag + (DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 0 : 11)).orElseThrow().toPose2d();
        else{
            return switch (this) {
                case CoralStation -> field.getTagPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 2 : 12).orElseThrow().toPose2d();
                case Processor -> field.getTagPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 3 : 16).orElseThrow().toPose2d();
                case BargeRed -> field.getTagPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 5 : 15).orElseThrow().toPose2d();
                case BargeBlue -> field.getTagPose(DriverStation.getAlliance().orElseThrow() == Alliance.Red ? 4 : 14).orElseThrow().toPose2d();
                default -> null;
            };
        }
    }

    public enum ReefSide{
        Left,
        Right,
        NULL;

        ReefSide(){}

        public Transform2d getOffset(){
            return switch(this) {
                case Left -> new Transform2d(Millimeters.of(0), Millimeters.of(328.61899/2), Rotation2d.kZero);
                case Right -> new Transform2d(Millimeters.of(0), Millimeters.of(-328.61899/2), Rotation2d.kZero);
                default ->  new Transform2d();
            };
        }

        @Override
        public String toString(){
            return switch(this) {
                case Left -> "左邊";
                case Right -> "右邊";
                default -> "";
            };
        }
    }
}
