package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;

public class Polynomial {

    double[] coeff;
    double[] deriv;
    double[] realRoots;
    int deg;

    // TODO: optimize the root solver's efficiency later

    /**
     * Constructs a Polynomial of the form: a_0x^n + a_1x^(n-1) + ... + a_n*x^(n-n)
     * @param a, the coefficients, written with higher degree first
     */
    public Polynomial(double... a) {
        deg = a.length;
        coeff = new double[deg];
        System.arraycopy(a, 0, coeff, 0, deg);
        deg--;
        realRoots = new double[deg];
    }

    // Evaluate derivative at x
    public static double evalPoly(double[] coeffs, double x) {
        double result = 0;
        for (double c : coeffs) {
            result = result * x + c; // Horner's method
        }
        return result;
    }

    // Evaluate derivative at x
    public static double[] derivative(double[] coeffs) {
        if (coeffs.length <= 1) return new double[]{0};
        double[] deriv = new double[coeffs.length - 1];
        int n = coeffs.length - 1;
        for (int i = 0; i < deriv.length; i++) {
            deriv[i] = coeffs[i] * (n - i);
        }
        return deriv;
    }

    // Bisection method for root in [a, b]
    public static double bisection(double[] coeffs, double a, double b, double tol) {
        double fa = evalPoly(coeffs, a);
        double fb = evalPoly(coeffs, b);
        if (fa * fb > 0) throw new IllegalArgumentException("No sign change in interval");

        while ((b - a) / 2 > tol) {
            double mid = (a + b) / 2;
            double fm = evalPoly(coeffs, mid);
            if (fm == 0) return mid;
            if (fa * fm < 0) {
                b = mid;
                fb = fm;
            } else {
                a = mid;
                fa = fm;
            }
        }
        return (a + b) / 2;
    }

    // Find all real roots
    public static List<Double> findRealRoots(double[] coeffs, double tol) {
        List<Double> roots = new ArrayList<>();

        // Remove leading zeros
        int firstNonZero = 0;
        while (firstNonZero < coeffs.length && Math.abs(coeffs[firstNonZero]) < tol) {
            firstNonZero++;
        }
        if (firstNonZero == coeffs.length) return roots; // All zero polynomial
        coeffs = Arrays.copyOfRange(coeffs, firstNonZero, coeffs.length);

        if (coeffs.length == 2) { // Linear
            roots.add(-coeffs[1] / coeffs[0]);
            return roots;
        }

        // Get derivative roots (critical points)
        double[] deriv = derivative(coeffs);
        List<Double> criticalPoints = findRealRoots(deriv, tol);

        // Add boundaries for scanning
        List<Double> testPoints = new ArrayList<>();
        testPoints.add(-1e3); // Large negative bound
        testPoints.addAll(criticalPoints);
        testPoints.add(1e3);  // Large positive bound

        // Check each interval for sign change
        for (int i = 0; i < testPoints.size() - 1; i++) {
            double a = testPoints.get(i);
            double b = testPoints.get(i + 1);
            double fa = evalPoly(coeffs, a);
            double fb = evalPoly(coeffs, b);
            if (fa == 0) roots.add(a);
            if (fa * fb < 0) {
                double root = bisection(coeffs, a, b, tol);
                roots.add(root);
            }
        }
        return roots;
    }


}
