package frc.robot.commands.swervedrive.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurnToSpeakerCommand extends Command {

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final MedianFilter filter = new MedianFilter(100);

    public TurnToSpeakerCommand(SwerveSubsystem swerveSub, VisionSubsystem visionSub, DoubleSupplier vX,
            DoubleSupplier vY) {
        swerve = swerveSub;
        vision = visionSub;

        System.out.println("RUNNING TurnToSpeakerCommand");
        addRequirements(vision);
        alongWith(new AbsoluteFieldDrive(swerve, vX, vY,
                () -> swerve.getHeading().plus(Rotation2d.fromDegrees(vision.getRotationErrorAngle())).getRadians()));
    }

    public void initialize() {
        filter.reset();
    }

    public void execute() {
        // Vision angles are positve CW in degrees, the rotation speed is rad/s CCW
        // positive
        double desiredAngle = swerve.getHeading().getDegrees() - vision.getRotationErrorAngle();
        double desiredAngleRads = desiredAngle * 2.0 * Math.PI / 360.0;
        System.out.println("AIMING - HEADING: " + swerve.getHeading().getDegrees());
        System.out.println("AIMING - ERROR: " + vision.getRotationErrorAngle());
        System.out.println("AIMING - DESIRED: " + desiredAngle);
        
        //swerve.drive(new Translation2d(), rotation, true);

        var chassisSpeeds = swerve.getSwerveController().getTargetSpeeds(0, 0, desiredAngleRads, swerve.getHeading().getRadians(), 10.0);

        swerve.drive(chassisSpeeds);
    }

    public void end(boolean interrupted) {
        System.out.println("YAW - :" + swerve.getSwerveController().lastAngleScalar);
        swerve.getSwerveController().lastAngleScalar = ((swerve.getHeading().getDegrees()
                + vision.getRotationErrorAngle()) % 360) * 2.0 * Math.PI / 360.0;
        System.out.println("YAW AFTER - :" + swerve.getSwerveController().lastAngleScalar);

    }

    public boolean isFinished() {
        // return Math.abs(vision.getRotationErrorAngle()) < 3;
        return false;
    }
}
