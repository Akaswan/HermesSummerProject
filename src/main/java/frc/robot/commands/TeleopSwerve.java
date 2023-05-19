package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;

import static frc.robot.utilities.Constants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private SwerveDrive swerveDrive;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(SwerveDrive swerveDrive, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), STICK_DEADBAND);

        /* Drive */
        swerveDrive.drive(
            new Translation2d(translationVal, strafeVal).times(MAX_SPEED), 
            rotationVal * MAX_ANGULAR_VELOCITY, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}