// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Inspiration taken from https://github.com/MAikenMagic1102/2023SheriffPhoenix6/blob/main/src/main/java/frc/robot/commands/PIDdriveToNearestGrid.java

package frc.robot.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionSubsystem;

/**
 * A command that wraps PID controllers to align to a position relative to a tag.
 */
public class PIDAlignReefCommand extends Command {
    /* These constants are used to avoid creating new objects during every execute call. */
    private final static ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds DRIVE = new SwerveRequest.ApplyRobotSpeeds(); 
    private final double[] ZERO_ARRAY = new double[] { 0, 0, 0 };
    
    /* The direction is a -1, 0, or 1. That's left, middle, and right, when facing the tag. */
    private final int direction;
    /* This is whether or not the direction variable should be multiplied by -1 when the robot is past the midway point of the reef */
    private final boolean flipDirection;
    /* This just means the robot will drive to a certain distance behind whatever position you gave it, which happens to 
     * be a constant of the coral's width. You can still score from here, but it's not as accurate.
     */
    private final boolean coralFirst;
    
    /* This is updated on call to the command */
    private Pose2d targetPose;

    /* This is the amount in x, y, and theta, that the robot needs to move to get to the final position. It's robot-relative,
     * meaning forward/backward, left/right, NOT field x and y.
     */
    private Pose2d changePose2d;

    private final PIDController xController = new PIDController(4.3, 0, 0);
    private final PIDController yController = new PIDController(4.3, 0, 0);
    private final PIDController thetaController = new PIDController(4.3, 0, 0);

    /* Coefficient found by simply feeding a constant value instead of the PID controllers, these are the smallest values
     * that allow the robot to still move. These will always be fed along with the PID controller, this is important because
     * when the controller attemps to make micro-adjustments, the wheels are unable to overcome carpet friction, and the robot
     * stops moving entirely, unable to reach the target pose.
     */
    private final double kF_linear = 0.065;
    private final double kF_angular = 0.16;

    /* This timer is used to print/log how much time was spent aligning. Useful to see why autons might be running slow. */
    private final Timer timer;

    /**
     * Creates a new PIDAlignCommand.
     * @param direction - The direction to align, robot-relative when facing the tag (-1, 0, 1).
     * @param flipDirection - Whether to flip the direction when the driver is past the midway point of the reef.
     * @param coralFirst - Whether to align further away considering for a coral.
     */
    public PIDAlignReefCommand(int direction, boolean flipDirection, boolean coralFirst) {
        setName("PIDAlignReefCommand");

        /* This essentially just takes any number and turns it into -1 if it is negative or 1 if it is positive.
         * 0 stays 0. Here just in case a typo is made and the command is called with -2 or 2 or something.
         */
        this.direction = (int) Math.signum(direction);
        this.flipDirection = flipDirection;
        this.coralFirst = coralFirst;
        this.timer = new Timer();

        /* Within this tolerance (these PID controllers use rad and meters) the PID controller will return 0.
         * PID usually returns the speed needed to get to a target, but if you are within 0.005 m of the target position,
         * which is half a centimeter, the controller's atSetpoint() method returns true. HOWEVER, the PID does NOT
         * return 0 speed within the tolerance, it still aims for the setpoint.
         */
        this.xController.setTolerance(0.005);
        this.yController.setTolerance(0.005);
        this.thetaController.setTolerance(Units.degreesToRadians(1.5));
        /* This is always important for rotational PIDs. It tells the controller that 0=2PI. Otherwise, if you feed
         * 1.9PI to the controller and set 0 as the target, it will rotate all 1.9PI.
         * This allows the PID controller to rotate only 0.1PI, much more efficient.
         */
        this.thetaController.enableContinuousInput(0, 2 * Math.PI);

        /* Because we always feed controllers the amount of change needed, we always want the change to be 0.
         * If you need to move 0, you are at the target position. This does mean that values from PID controllers are
         * negative. This is explained below where they are called in execute.
         */
        this.xController.setSetpoint(0);
        this.yController.setSetpoint(0);
        this.thetaController.setSetpoint(0);

        /* This lets you see the position the robot is targeting. It is (0,0),0 by default when the command is not running.
         * Note : this manner of storing poses using arrays will be removed in WPILib 2026 and you should learn to use their new
         * struct system or whatever it is.
         */
        SmartDashboard.putNumberArray("Aligning Target Pose", VisionSubsystem.pose2dToArray(Pose2d.kZero));

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Translation2d robotPose = SwerveSubsystem.getInstance().getState().Pose.getTranslation();
        // targetPose should be Pose2d.kZero if the closest position is too far.
        this.targetPose = ReefAlignedPositions.getClosestReefAlignedPosition(
            robotPose,
            /* You can see here the flip direction working using a ternary operator.
             * If direction should be flipped & the robot is NOT on the driver side of the reef, then multiply
             * direction by -1. Otherwise, multiply it by 1, so don't do anything.
             */
            this.direction * (this.flipDirection && !ReefAlignedPositions.isPositionDriverSideOfReef(robotPose) ? -1 : 1)
        );
        
        SmartDashboard.putNumberArray("Aligning Target Pose", VisionSubsystem.pose2dToArray(this.targetPose));
        
        this.changePose2d = null;
        /* Always reset PIDControllers on every new run of the command, because the robot is starting from a new location
         * and otherwise it will use the movement history of the last align.
         */
        this.xController.reset();
        this.yController.reset();
        this.thetaController.reset();

        this.timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /* If the target is (0,0),0, essentially if it is too far, don't do anything during execute. The end clause of this command
         * will terminate the command if this is the case, but execute is called anyway and it shouldn't run otherwise the robot will
         * aim for the origin.
         */
        if (this.targetPose == Pose2d.kZero) return;

        /* Math */
        this.changePose2d = calculateChangeRobotRelative();

        /* These are negative because the setpoint is zero.
         * To go from 0 to +x, you need a positive speed. If the setpoint is zero, you are doing +x to 0.
         * Essentially, the robot needs to move +x, but the PID controller thinks it needs to move -x.
         * Since x is the same just in different directions, it doesn't affect the controller to just make it negative.
         */
        double xSpeed = -this.xController.calculate(this.changePose2d.getX());
        if (this.xController.atSetpoint()) {
            /* Do this because around the tolerance PID is not zero,
             * and combined with the friction speed this could mess up other directions.
             */
            xSpeed = 0; 
        }

        double ySpeed = -this.yController.calculate(this.changePose2d.getY());
        if (this.yController.atSetpoint()) {
            ySpeed = 0;
        }
        
        double thetaSpeed = -this.thetaController.calculate(this.changePose2d.getRotation().getRadians());
        if (this.thetaController.atSetpoint()) {
            thetaSpeed = 0;
        }

        SmartDashboard.putNumberArray("Aligning Change Needed", VisionSubsystem.pose2dToArray(changePose2d));
        
        ChassisSpeeds speeds = new ChassisSpeeds(
            /* Math.signum here is used to not only add the friction coefficient in the right direction, but also if the
             * controller is within tolerance, because it is set to 0 above, this means the friction will also be set to 0.
             * That way, there is only a speed applied to overcome friction IF the controller is not within the tolerance.
             */
            xSpeed + Math.signum(xSpeed) * this.kF_linear,
            ySpeed + Math.signum(ySpeed) * this.kF_linear,
            thetaSpeed + Math.signum(thetaSpeed) * this.kF_angular
        );

        /* Used for logging/testing. */
        SmartDashboard.putNumberArray("Aligning Speeds", new Double[] {
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            Math.toDegrees(speeds.omegaRadiansPerSecond)
        });
        
        SwerveSubsystem.getInstance().setControl(this.DRIVE.withSpeeds(speeds));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        /* ALWAYS end a command that controls the swerve subsystem with setting its speeds to 0.
         * Otherwise, it will keep heading in the last set direction until a movement input is provided !
         */
        SwerveSubsystem.getInstance().setControl(this.DRIVE.withSpeeds(PIDAlignReefCommand.ZERO_SPEEDS));
        /* Reset these once the command is over so it's clear the robot isn't targeting anything anymore. */
        SmartDashboard.putNumberArray("Aligning Target Pose", this.ZERO_ARRAY);
        SmartDashboard.putNumberArray("Aligning Change Needed", this.ZERO_ARRAY);
        SmartDashboard.putNumberArray("Aligning Speeds", this.ZERO_ARRAY);

        /* The latter doesn't interrupt the Command even though it's an early end, so it is treated as such
         * with the LEDs turning into error color (to tell the driver the robot is too far to align).
         */
        if (interrupted || this.targetPose == Pose2d.kZero) {
            LEDSubsystem.getInstance().setColor(StatusColors.ERROR);
        }
        else {
            LEDSubsystem.getInstance().setColor(StatusColors.OK);
        }

        System.out.println("\n\nTime taken to align : " + this.timer.get() + "\n\n");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.targetPose == Pose2d.kZero
            || (
                this.changePose2d != null
                && this.xController.atSetpoint()
                && this.yController.atSetpoint()
                /* The rotational error is greater than the tolerance because we do want the robot to keep turning until it's
                 * nearly perpendicular to the reef, but this never actually happens perfectly and the command can go
                 * on forever just jittering back and forth.
                 */
                && Math.abs(this.thetaController.getError()) <= Units.degreesToRadians(5)
            );
    }

    /**
     * The distances needed to drive to get to the target pose.
     * Essentially just the values to use with the PIDControllers.
     * @return The change pose.
     */
    private Pose2d calculateChangeRobotRelative() {
        /* This math basically turns the field-relative robot position and target position into a robot-relative
         * target position. I don't feel like explaining it. It's trigonometry. Figure it out. But essentially,
         * it takes the x and y changes required in FIELD-ORIENTATION, and uses the robot's known rotation
         * (sine and cosine) and turns those x and y changes into their sine and cosine components robot-relative
         */
        Pose2d robotPose = SwerveSubsystem.getInstance().getState().Pose;
        double robotCos = robotPose.getRotation().getCos();
        double robotSin = robotPose.getRotation().getSin();

        double xChange = this.targetPose.getX() - robotPose.getX();
        double yChange = this.targetPose.getY() - robotPose.getY();

        return new Pose2d(
            new Translation2d(
                xChange * robotCos + yChange * robotSin,
                -xChange * robotSin + yChange * robotCos
            ).minus( /* This is where coralFirst has an effect, you can see it essentially just subtracts
            a constant from the x direction (foward/backward) */
                this.coralFirst
                    ? new Translation2d(ReefAlignedPositions.CORAL_DISTANCE, 0)
                    : Translation2d.kZero
            ),
            this.targetPose.getRotation().minus(robotPose.getRotation())
        );
    }
}
