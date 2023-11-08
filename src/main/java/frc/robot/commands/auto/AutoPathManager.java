package frc.robot.commands.auto;

import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.util.autonomous.QuinticHermiteSpline;
import frc.robot.util.autonomous.SwervePath;
import frc.robot.util.math.Angle;
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

    private Pose lowStartPose;
    private Pose lowEndPose;
    private Pose highStartPose;
    private Pose highEndPose;

    private Pose lowBeforeFirstPiece;
    private Pose lowAfterFirstPiece;
    private Pose highBeforeFirstPiece;
    private Pose highAfterFirstPiece;

    private Pose lowBeforeSecondPiece;
    private Pose lowAfterSecondPiece;
    private Pose highBeforeSecondPiece;
    private Pose highAfterSecondPiece;

    private Pose lowBalanceEnding;
    private Pose lowBalanceConnection;
    private Pose highBalanceEnding;
    private Pose highBalanceConnection;

    public AutoPathManager(PoseEstimator poseEstimator, Swerve swerve, Gyro gyro) {
        this.poseEstimator = poseEstimator;
        this.swerve = swerve;
        this.gyro = gyro;

        paths = new HashMap<>();

        int lowTag = poseEstimator.FIELD_LEFT_SIDE ? 8 : 1;
        int highTag = poseEstimator.FIELD_LEFT_SIDE ? 6 : 3;

        initLowPoses(lowTag, poseEstimator.FIELD_LEFT_SIDE);
        initHighPoses(highTag, poseEstimator.FIELD_LEFT_SIDE);

        initLowStartPaths(poseEstimator.FIELD_LEFT_SIDE);
        initLowReturnPaths();
        initHighStartPaths(poseEstimator.FIELD_LEFT_SIDE);
        initHighReturnPaths();
    }

    private void initLowPoses(int tag, boolean leftSide) {
        lowStartPose = new Pose(Point.add(poseEstimator.getScoringZone(tag)[1].getPoint(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI, true));
        lowEndPose = new Pose(lowStartPose.getPoint(),
                new Angle(leftSide ? Math.PI : 0, true));

        lowBeforeFirstPiece = new Pose(Point.add(lowStartPose.getPosition(), new Point(leftSide ? 180 : -180, -6.25)),
                lowStartPose.getAngle());
        lowAfterFirstPiece = new Pose(Point.add(lowBeforeFirstPiece.getPoint(), new Point(new Vector(lowBeforeFirstPiece.getAngle(), 10))),
                new Angle(lowBeforeFirstPiece.getAngle().getRadians() + Math.PI, true));

        lowBeforeSecondPiece = new Pose(Point.add(lowBeforeFirstPiece.getPoint(), new Point(0, 45)),
                new Angle(lowStartPose.getAngle().getRadians() + Math.PI / 3 * (leftSide ? 1 : -1), true));
        lowAfterSecondPiece = new Pose(Point.add(lowBeforeSecondPiece.getPoint(), new Point(new Vector(lowBeforeSecondPiece.getAngle(), 10))),
                new Angle(lowBeforeFirstPiece.getAngle().getRadians() + Math.PI, true));
    }

    private void initHighPoses(int tag, boolean leftSide) {
        highStartPose = new Pose(Point.add(poseEstimator.getScoringZone(tag)[1].getPoint(), new Point(leftSide ? 32 : -32, 0)),
                new Angle(leftSide ? 0 : Math.PI, true));
        highEndPose = new Pose(highStartPose.getPoint(),
                new Angle(leftSide ? Math.PI : 0, true));

        highBeforeFirstPiece = new Pose(Point.add(highStartPose.getPosition(), new Point(leftSide ? 180 : -180, 5.75)),
                lowStartPose.getAngle());
        highAfterFirstPiece = new Pose(Point.add(highBeforeFirstPiece.getPoint(), new Point(new Vector(highBeforeFirstPiece.getAngle(), 10))),
                new Angle(highBeforeFirstPiece.getAngle().getRadians() + Math.PI, true));

        highBeforeSecondPiece = new Pose(Point.add(highBeforeFirstPiece.getPoint(), new Point(0, -45)),
                new Angle(highStartPose.getAngle().getRadians() + Math.PI / 3 * (leftSide ? -1 : 1), true));
        highAfterSecondPiece = new Pose(Point.add(lowBeforeSecondPiece.getPoint(), new Point(new Vector(lowBeforeSecondPiece.getAngle(), 10))),
                new Angle(highBeforeFirstPiece.getAngle().getRadians() + Math.PI, true));
    }

    private void initLowStartPaths(boolean leftSide) {
        paths.put(LOW_TAXI, new SwervePath(
                new QuinticHermiteSpline(
                        lowStartPose,
                        new Pose(Point.add(lowStartPose.getPoint(), new Point(leftSide ? 100 : -100, 0)), lowStartPose.getAngle())
                ),
                new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
        paths.put(LOW_TO_FIRST_PIECE, new SwervePath(
                new QuinticHermiteSpline(
                        lowStartPose,
                        lowBeforeFirstPiece
                ),
                new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(lowBeforeFirstPiece.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
        paths.put(LOW_TO_SECOND_PIECE, new SwervePath(
                new QuinticHermiteSpline(
                        lowStartPose,
                        lowBeforeSecondPiece
                ),
                new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(lowBeforeSecondPiece.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
//        paths.put(LOW_TO_THIRD_PIECE, new SwervePath(
//
//        ));
//        paths.put(LOW_BALANCE_FAR_SIDE, new SwervePath(
//
//        ));
    }

    private void initLowReturnPaths() {
        paths.put(LOW_FIRST_PIECE_TO_SCORE, new SwervePath(
                new QuinticHermiteSpline(
                        lowAfterFirstPiece,
                        lowEndPose),
                new Angle(lowBeforeFirstPiece.getHeading().getRadians() - Math.PI, true), new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
        paths.put(LOW_SECOND_PIECE_TO_SCORE, new SwervePath(
                new QuinticHermiteSpline(
                        lowAfterSecondPiece,
                        lowEndPose),
                new Angle(lowBeforeSecondPiece.getHeading().getRadians() - Math.PI, true), new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
    }

    private void initHighStartPaths(boolean leftSide) {
        paths.put(HIGH_TAXI, new SwervePath(
                new QuinticHermiteSpline(
                        highStartPose,
                        new Pose(Point.add(highStartPose.getPoint(), new Point(leftSide ? 100 : -100, 0)), highStartPose.getAngle())
                ),
                new Angle(lowStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(highStartPose.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
        paths.put(HIGH_TO_FIRST_PIECE, new SwervePath(
                new QuinticHermiteSpline(
                        highStartPose,
                        highBeforeFirstPiece
                ),
                new Angle(highStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(highStartPose.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
        paths.put(HIGH_TO_SECOND_PIECE, new SwervePath(
                new QuinticHermiteSpline(
                        highStartPose,
                        highBeforeSecondPiece
                ),
                new Angle(highStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(highBeforeSecondPiece.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
//        paths.put(HIGH_TO_THIRD_PIECE, new SwervePath(
//
//        ));
//        paths.put(HIGH_BALANCE_FAR_SIDE, new SwervePath(
//
//        ));
    }

    private void initHighReturnPaths() {
        paths.put(HIGH_FIRST_PIECE_TO_SCORE, new SwervePath(
                new QuinticHermiteSpline(
                        highAfterFirstPiece,
                        highEndPose),
                new Angle(highStartPose.getHeading().getRadians() - Math.PI, true),
                new Angle(highStartPose.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
        paths.put(HIGH_SECOND_PIECE_TO_SCORE, new SwervePath(
                new QuinticHermiteSpline(
                        highAfterSecondPiece,
                        highEndPose),
                new Angle(highBeforeSecondPiece.getHeading().getRadians() - Math.PI, true), new Angle(highAfterSecondPiece.getHeading().getRadians() - Math.PI, true),
                0, 0, 100, 100, 100, 0,
                0.1, 0.5, 1, 1,
                3, 0, 0.001
        ));
    }

    public SwervePath getGroundIntakingPath(double dist) {
        Pose start = new Pose(poseEstimator.getState().getPoint(), new Angle(gyro.getHeadingRadians() + Math.PI, true));

        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                start,
                new Pose(
                        Point.add(start.getPoint(), new Point(new Vector(start.getAngle(), dist))),
                        start.getAngle()
                )
        );

        return new SwervePath(
                spline,
                start.getHeading(), start.getHeading(),
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
