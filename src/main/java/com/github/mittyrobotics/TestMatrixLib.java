package com.github.mittyrobotics;

import org.ejml.simple.SimpleMatrix;

public class TestMatrixLib {
    public static void main(String[] args) {
//        SimpleMatrix matrix = new SimpleMatrix(new double[][] {{0, 1}, {2, 3}});
//        matrix.mult(new SimpleMatrix(new double[][] {{3}, {5}})).plus(new SimpleMatrix(new double[][]{{2}, {7}})).print();

        SimpleMatrix matrix = new SimpleMatrix(new double[]{20, 1, 2, 3, 4});
        matrix.print();
    }
}
