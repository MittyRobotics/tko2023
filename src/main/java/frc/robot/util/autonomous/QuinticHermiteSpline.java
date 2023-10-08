package frc.robot.util.autonomous;

import frc.robot.util.math.*;

public class QuinticHermiteSpline extends Parametric {
    Point start, end;
    Vector vel0, accel0, vel1, accel1;
    double length;

    public QuinticHermiteSpline(Point start, Vector vel0, Vector accel0, Point end, Vector vel1, Vector accel1) {
        this.start = start;
        this.end = end;
        this.vel0 = vel0;
        this.accel0 = accel0;
        this.vel1 = vel1;
        this.accel1 = accel1;

        this.length = getLength(1.0, 17);
    }

    public double getLength() {
        return length;
    }

    public Vector getEndVel() {
        return vel1;
    }

    public Point getEnd() {
        return end;
    }

    public QuinticHermiteSpline(Point start, Angle theta0, Point end, Angle theta1) {
        this(
                start,
                new Vector(
                        Math.cos(theta0.getRadians()) * Math.sqrt((end.getX() - start.getX()) * (end.getX() - start.getX()) + (end.getY() - start.getY()) * (end.getY() - start.getY())),
                        Math.sin(theta0.getRadians()) * Math.sqrt((end.getX() - start.getX()) * (end.getX() - start.getX()) + (end.getY() - start.getY()) * (end.getY() - start.getY()))
                ),
                new Vector(0, 0),
                end,
                new Vector(
                        Math.cos(theta1.getRadians()) * Math.sqrt((end.getX() - start.getX()) * (end.getX() - start.getX()) + (end.getY() - start.getY()) * (end.getY() - start.getY())),
                        Math.sin(theta1.getRadians()) * Math.sqrt((end.getX() - start.getX()) * (end.getX() - start.getX()) + (end.getY() - start.getY()) * (end.getY() - start.getY()))
                ),
                new Vector(0, 0)
        );
    }

    public QuinticHermiteSpline(Pose start, Pose end) {
        this(start.getPosition(), start.getHeading(), end.getPosition(), end.getHeading());
    }

    public QuinticHermiteSpline(Point p1, Point p2) {
        this(new Pose(p1, new Vector(p1, p2).getAngle()), new Pose(p2, new Vector(p1, p2).getAngle()));
    }

    static class BasisFunctions {
        static class Functions {
            public static double h0(double t) {
                return -6 * t * t * t * t * t + 15 * t * t * t * t - 10 * t * t * t + 1;
            }

            public static double h1(double t) {
                return -3 * t * t * t * t * t + 8 * t * t * t * t - 6 * t * t * t + 1 * t;
            }

            public static double h2(double t) {
                return -(t * t * t * t * t) / 2. + 3 * t * t * t * t / 2. - 3 * t * t * t / 2. + t * t / 2.;
            }

            public static double h3(double t) {
                return t * t * t * t * t / 2. - t * t * t * t + t * t * t / 2.;
            }

            public static double h4(double t) {
                return -3 * t * t * t * t * t + 7 * t * t * t * t - 4 * t * t * t;
            }

            public static double h5(double t) {
                return 6 * t * t * t * t * t - 15 * t * t * t * t + 10 * t * t * t;
            }
        }

        static class FirstDerivative {
            public static double h0(double t) {
                return -30 * t * t * t * t + 60 * t * t * t - 30 * t * t;
            }

            public static double h1(double t) {
                return -15 * t * t * t * t + 32 * t * t * t - 18 * t * t + 1;
            }

            public static double h2(double t) {
                return -(5 * t * t * t * t) / 2 + 6 * t * t * t - 9 * t * t / 2 + t;
            }

            public static double h3(double t) {
                return 5 * t * t * t * t / 2 - 4 * t * t * t + 3 * t * t / 2;
            }

            public static double h4(double t) {
                return -15 * t * t * t * t + 28 * t * t * t - 12 * t * t;
            }

            public static double h5(double t) {
                return 30 * t * t * t * t - 60 * t * t * t + 30 * t * t;
            }
        }

        static class SecondDerivative {
            public static double h0(double t) {
                return -120 * t * t * t + 180 * t * t - 60 * t;
            }

            public static double h1(double t) {
                return -60 * t * t * t + 96 * t * t - 36 * t;
            }

            public static double h2(double t) {
                return -10 * t * t * t + 18 * t * t - 9 * t + 1;
            }

            public static double h3(double t) {
                return t * (10 * t * t - 12 * t + 3);
            }

            public static double h4(double t) {
                return -60 * t * t * t + 84 * t * t - 24 * t;
            }

            public static double h5(double t) {
                return 120 * t * t * t - 180 * t * t + 60 * t;
            }
        }
    }

    @Override
    public double x(double t) {
        return BasisFunctions.Functions.h0(t) * start.getX() + BasisFunctions.Functions.h1(t) * vel0.getX() + BasisFunctions.Functions.h2(t) * accel0.getX() +
                BasisFunctions.Functions.h3(t) * accel1.getX() + BasisFunctions.Functions.h4(t) * vel1.getX() + BasisFunctions.Functions.h5(t) * end.getX();
    }

    @Override
    public double y(double t) {
        return BasisFunctions.Functions.h0(t) * start.getY() + BasisFunctions.Functions.h1(t) * vel0.getY() + BasisFunctions.Functions.h2(t) * accel0.getY() +
                BasisFunctions.Functions.h3(t) * accel1.getY() + BasisFunctions.Functions.h4(t) * vel1.getY() + BasisFunctions.Functions.h5(t) * end.getY();
    }

    @Override
    public double xPrime(double t) {
        return BasisFunctions.FirstDerivative.h0(t) * start.getX() + BasisFunctions.FirstDerivative.h1(t) * vel0.getX() + BasisFunctions.FirstDerivative.h2(t) * accel0.getX() +
                BasisFunctions.FirstDerivative.h3(t) * accel1.getX() + BasisFunctions.FirstDerivative.h4(t) * vel1.getX() + BasisFunctions.FirstDerivative.h5(t) * end.getX();
    }

    @Override
    public double yPrime(double t) {
        return BasisFunctions.FirstDerivative.h0(t) * start.getY() + BasisFunctions.FirstDerivative.h1(t) * vel0.getY() + BasisFunctions.FirstDerivative.h2(t) * accel0.getY() +
                BasisFunctions.FirstDerivative.h3(t) * accel1.getY() + BasisFunctions.FirstDerivative.h4(t) * vel1.getY() + BasisFunctions.FirstDerivative.h5(t) * end.getY();
    }

    @Override
    public double xDoublePrime(double t) {
        return BasisFunctions.SecondDerivative.h0(t) * start.getX() + BasisFunctions.SecondDerivative.h1(t) * vel0.getX() + BasisFunctions.SecondDerivative.h2(t) * accel0.getX() +
                BasisFunctions.SecondDerivative.h3(t) * accel1.getX() + BasisFunctions.SecondDerivative.h4(t) * vel1.getX() + BasisFunctions.SecondDerivative.h5(t) * end.getX();
    }

    @Override
    public double yDoublePrime(double t) {
        return BasisFunctions.SecondDerivative.h0(t) * start.getY() + BasisFunctions.SecondDerivative.h1(t) * vel0.getY() + BasisFunctions.SecondDerivative.h2(t) * accel0.getY() +
                BasisFunctions.SecondDerivative.h3(t) * accel1.getY() + BasisFunctions.SecondDerivative.h4(t) * vel1.getY() + BasisFunctions.SecondDerivative.h5(t) * end.getY();
    }

    public Vector getVelocityVector(double t) {
        return new Vector(xPrime(t), yPrime(t));
    }

    private double f(double t, Point robot) {
        return Math.sqrt(
                (x(t) - robot.getX()) * (x(t) - robot.getX()) +
                        (y(t) - robot.getY()) * (y(t) - robot.getY())
        );
    }

    private double numer1(double t, Point robot) {
        return (x(t) - robot.getX()) * xPrime(t) + (y(t) - robot.getY()) * yPrime(t);
    }

    private double firstDeriv(double t, Point robot) {
        return numer1(t, robot) / f(t, robot);
    }

    private double numer2(double t, Point robot) {
        return f(t, robot) * (
                (x(t) - robot.getX()) * xDoublePrime(t) + xPrime(t) * xPrime(t) +
                        (y(t) - robot.getY()) * yDoublePrime(t) + yPrime(t) * yPrime(t)) - (
                numer1(t, robot) * firstDeriv(t, robot)
        );
    }

    private double secondDeriv(double t, Point robot) {
        return numer2(t, robot) / (f(t, robot) * f(t, robot));
    }

    private double curvatureNumerator(double t) {
        return xPrime(t) * yDoublePrime(t) - xDoublePrime(t) * yPrime(t);
    }

    private double curvatureDenominator(double t) {
        return Math.sqrt(Math.pow(xPrime(t) * xPrime(t) + yPrime(t) * yPrime(t), 3));
    }

    public double curvature(double t) {
        return curvatureNumerator(t) / curvatureDenominator(t);
    }

    private double tAndPoseDist(double t, Pose robot) {
        return new Vector(robot.getPosition(), get(t)).getMagnitude();
    }

    private double firstDerivDistSquared(double t, Pose robot) {
        return 2 * (x(t) - robot.getPosition().getX()) * xPrime(t) + 2 * (y(t) - robot.getPosition().getY()) * yPrime(t);
    }

    private double secondDerivDistSquared(double t, Pose robot) {
        return 2 * (x(t) - robot.getPosition().getX()) * xDoublePrime(t) + xPrime(t) * xPrime(t) +
                2 * (y(t) - robot.getPosition().getY()) * yDoublePrime(t) + yPrime(t) * yPrime(t);
    }

    public double getClosestPoint(Pose robot, int pointsToSample, int newtonSteps) {
        Vector cur_min = new Vector(Double.POSITIVE_INFINITY, 0);

        //the steps to start Newton's method from
        for (double i = 0; i <= 1 + 1e-6; i += 1. / pointsToSample) {
            double cur_t = i;
            //get first and secondary derivatives of the distance function at that point
            double firstDeriv = firstDerivDistSquared(cur_t, robot);
            double secondDeriv = secondDerivDistSquared(cur_t, robot);

            //amount to adjust according to Newton's method
            //https://en.wikipedia.org/wiki/Newton%27s_method
            //using first and second derivatives because we want min of distance function (zero of its derivative)
            double dt = firstDeriv / secondDeriv;

            int counter = 0;

            //run for certain number of iterations
            while (counter < newtonSteps) {

                //adjust based on Newton's method, get new derivatives
                cur_t -= dt;
                firstDeriv = firstDerivDistSquared(cur_t, robot);
                secondDeriv = secondDerivDistSquared(cur_t, robot);
                dt = firstDeriv / secondDeriv;
                counter++;
            }

            //if distance is less than previous min, update distance and t
            double cur_d = tAndPoseDist(cur_t, robot);

            if (cur_d < cur_min.getX() && cur_t >= 0 && cur_t <= 1) {
                cur_min = new Vector(cur_d, cur_t);
            }
        }

        //return t of minimum distance, clamped from 0 to 1
        return Math.min(1, Math.max(0, cur_min.getY()));

    }
}
