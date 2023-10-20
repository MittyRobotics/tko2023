package frc.robot.commands.auto;

import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.util.autonomous.QuinticHermiteSpline;
import frc.robot.util.autonomous.SwervePath;
import frc.robot.util.math.Point;
import frc.robot.util.math.Pose;
import frc.robot.util.math.Vector;

import java.util.HashMap;

import static frc.robot.commands.auto.AutoPathManager.PathName.*;

public class AutoPathManager {
    private PoseEstimator poseEstimator;
    private Swerve swerve;
    private Gyro gyro;

    public final HashMap<PathName, SwervePath> paths;

    private Pose lowStartPose = new Pose();
    private Pose highStartPose = new Pose();

    private Pose lowFirstPiece = new Pose();
    private Pose highFirstPiece = new Pose();

    private Pose lowSecondPiece = new Pose();
    private Pose highSecondPiece = new Pose();

    private Pose lowBalance = new Pose();
    private Pose highBalance = new Pose();

    public AutoPathManager(PoseEstimator poseEstimator, Swerve swerve, Gyro gyro) {
        this.poseEstimator = poseEstimator;
        this.swerve = swerve;
        this.gyro = gyro;

        paths = new HashMap<>();

        initLowStartPaths();
        initLowReturnPaths();
        initHighStartPaths();
        initHighReturnPaths();
    }

    private void initLowStartPaths() {
        paths.put(LOW_TAXI, new SwervePath(

        ));
        paths.put(LOW_TO_FIRST_PIECE, new SwervePath(

        ));
        paths.put(LOW_TO_SECOND_PIECE, new SwervePath(

        ));
        paths.put(LOW_TO_THIRD_PIECE, new SwervePath(

        ));
        paths.put(LOW_BALANCE_FAR_SIDE, new SwervePath(

        ));
    }

    private void initLowReturnPaths() {
        paths.put(LOW_FIRST_PIECE_TO_SCORE, new SwervePath(

        ));
        paths.put(LOW_SECOND_PIECE_TO_SCORE, new SwervePath(

        ));
    }

    private void initHighStartPaths() {
        paths.put(HIGH_TAXI, new SwervePath(

        ));
        paths.put(HIGH_TO_FIRST_PIECE, new SwervePath(

        ));
        paths.put(HIGH_TO_SECOND_PIECE, new SwervePath(

        ));
        paths.put(HIGH_TO_THIRD_PIECE, new SwervePath(

        ));
        paths.put(HIGH_BALANCE_FAR_SIDE, new SwervePath(

        ));
    }

    private void initHighReturnPaths() {
        paths.put(HIGH_FIRST_PIECE_TO_SCORE, new SwervePath(

        ));
        paths.put(HIGH_SECOND_PIECE_TO_SCORE, new SwervePath(

        ));
    }

    public SwervePath getGroundIntakingPath(double dist) {
        Pose robot = new Pose(poseEstimator.getState().getPoint(), gyro.getRadiansAsAngle());

        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                robot,
                new Pose(Point.add(robot.getPoint(), new Point(new Vector(robot.getAngle(), dist))), robot.getAngle())
        );

        return new SwervePath(
                spline,
                robot.getHeading(), robot.getHeading(),
                swerve.getDesiredVel().getMagnitude(), 0, 10, 10, 10, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        );
    }

    public enum PathName {
        LOW_TAXI,
        LOW_TO_FIRST_PIECE,
        LOW_TO_SECOND_PIECE,
        LOW_TO_THIRD_PIECE,
        LOW_BALANCE_FAR_SIDE,

        LOW_FIRST_PIECE_TO_SCORE,
        LOW_SECOND_PIECE_TO_SCORE,

        HIGH_TAXI,
        HIGH_TO_FIRST_PIECE,
        HIGH_TO_SECOND_PIECE,
        HIGH_TO_THIRD_PIECE,
        HIGH_BALANCE_FAR_SIDE,

        HIGH_FIRST_PIECE_TO_SCORE,
        HIGH_SECOND_PIECE_TO_SCORE
    }
}
