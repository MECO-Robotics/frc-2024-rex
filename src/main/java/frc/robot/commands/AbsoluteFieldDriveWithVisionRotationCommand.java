package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AbsoluteFieldDriveWithVisionRotationCommand extends AbsoluteDrive {

    private final VisionSubsystem vision;

    DoubleSupplier headingHorizontalSupplier = new DoubleSupplier() {
        public double getAsDouble() {

            return Math
                    .cos((swerve.getHeading().getDegrees() - vision.getRotationErrorAngle()) * 2.0 * Math.PI / 360.0);
        }
    };

    DoubleSupplier headingVerticalSupplier = new DoubleSupplier() {
        public double getAsDouble() {

            return Math
                    .sin((swerve.getHeading().getDegrees() - vision.getRotationErrorAngle()) * 2.0 * Math.PI / 360.0);
        }
    };

    public AbsoluteFieldDriveWithVisionRotationCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
            VisionSubsystem vision) {

        super();

        this.vision = vision;
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontalSupplier;
        this.headingVertical = headingVerticalSupplier;

        addRequirements(vision);
    }

    public void execute() {

        super.execute();
    }
}
