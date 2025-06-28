function [F, G] = solve_diophantine(A, C, degF)
    syms x
    f = sym('f', [1 degF+1]);  
    F_poly = poly2sym(f, x);   

    A_poly = poly2sym(A, x);
    C_poly = poly2sym(C, x);

    % Multiply A and F
    AF_poly = expand(A_poly * F_poly);

    % G = C - A*F
    G_poly = expand(C_poly - AF_poly);

    % Degree of G
    degG = double(feval(symengine, 'degree', G_poly, x));
    g = sym('g', [1 degG+1]);
    G_test = poly2sym(g, x);

    % Create equation: AF + G = C
    eqn = expand(AF_poly + G_test - C_poly);
    eqn_coeffs = fliplr(coeffs(eqn, x));

    % Solve the system of equations
    unknowns = [f, g];
    sol = solve(eqn_coeffs == 0, unknowns, 'Real', true);

    % Extract solutions
    F = double(subs(f, sol));
    G = double(subs(g, sol));
end