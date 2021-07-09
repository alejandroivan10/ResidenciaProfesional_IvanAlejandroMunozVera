% Function to compute one filter iteration
function [SEq] = Madgwick_Update(w, a, m)
            %local system variables
    %norm; % vector norm
    %SEqDot_omega = zeros(1,4); % quaternion rate from gyroscopes elements
    f = zeros(1,3); % objective function elements
    %J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, % objective function Jacobian elements
    %J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; %
    SEqHatDot = zeros(1,4); % estimated direction of the gyroscope error
    %w_err_x; w_err_y; w_err_z; % estimated direction of the gyroscope error (angular)
    %h_x, h_y, h_z; % computed flux in the earth frame
    % axulirary variables to avoid reapeated calcualtions
    halfSEq(1) = 0.5 * SEq(1);
    halfSEq(2) = 0.5 * SEq(2);
    halfSEq(3) = 0.5 * SEq(3);
    halfSEq(4) = 0.5 * SEq(4);
    twoSEq(1) = 2.0 * SEq(1);
    twoSEq(2) = 2.0 * SEq(2);
    twoSEq(3) = 2.0 * SEq(3);
    twoSEq(4) = 2.0 * SEq(4);
    twob_x = 2.0 * b_x;
    twob_z = 2.0 * b_z;
    twob_xSEq(1) = 2.0 * b_x * SEq(1);
    twob_xSEq(2) = 2.0 * b_x * SEq(2);
    twob_xSEq(3) = 2.0 * b_x * SEq(3);
    twob_xSEq(4) = 2.0 * b_x * SEq(4);
    twob_zSEq(1) = 2.0 * b_z * SEq(1);
    twob_zSEq(2) = 2.0 * b_z * SEq(2);
    twob_zSEq(3) = 2.0 * b_z * SEq(3);
    twob_zSEq(4) = 2.0 * b_z * SEq(4);
    %SEq_1SEq_2;
    SEq_1SEq_3 = SEq(1) * SEq(3);
    %SEq_1SEq_4;
    %SEq_2SEq_3;
    SEq_2SEq_4 = SEq(2) * SEq(4);
    %SEq_3SEq_4;
    twom_x = 2.0 * m(1);
    twom_y = 2.0 * m(2);
    twom_z = 2.0 * m(3);
    % normalise the accelerometer measurement
    norm = sqrt(a(1)*a(1) + a(2)*a(2) + a(3)*a(3));
    a(1) = a(1)/norm;
    a(2) = a(2)/norm;
    a(3) = a(3)/norm;
    % normalise the magnetometer measurement
    norm = sqrt(m(1)*m(1) + m(2)*m(2) + m(3)*m(3));
    m(1) = m(1)/norm;
    m(2) = m(2)/norm;
    m(3) = m(3)/norm;
    % compute the objective function and Jacobian
    f(1) = twoSEq(2) * SEq(4) - twoSEq(1) * SEq(3) - a(1);
    f(2) = twoSEq(1) * SEq(2) + twoSEq(3) * SEq(4) - a(2);
    f(3) = 1.0 - twoSEq(2) * SEq(2) - twoSEq(3) * SEq(3) - a(3);
    f(4) = twob_x * (0.5 - SEq(3) * SEq(3) - SEq(4) * SEq(4)) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m(1);
    f(5) = twob_x * (SEq(2) * SEq(3) - SEq(1) * SEq(4)) + twob_z * (SEq(1) * SEq(2) + SEq(3) * SEq(4)) - m(2);
    f(6) = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq(2) * SEq(2) - SEq(3) * SEq(3)) - m(3);
    J_11or24 = twoSEq(3); % J_11 negated in matrix multiplication
    J_12or23 = 2.0 * SEq(4);
    J_13or22 = twoSEq(1); % J_12 negated in matrix multiplication
    J_14or21 = twoSEq(2);
    J_32 = 2.0 * J_14or21; % negated in matrix multiplication
    J_33 = 2.0 * J_11or24; % negated in matrix multiplication
    J_41 = twob_zSEq(3); % negated in matrix multiplication
    J_42 = twob_zSEq(4);
    J_43 = 2.0 * twob_xSEq(3) + twob_zSEq(1); % negated in matrix multiplication
    J_44 = 2.0 * twob_xSEq(4) - twob_zSEq(2); % negated in matrix multiplication
    J_51 = twob_xSEq(4) - twob_zSEq(2); % negated in matrix multiplication
    J_52 = twob_xSEq(3) + twob_zSEq(1);
    J_53 = twob_xSEq(2) + twob_zSEq(4);
    J_54 = twob_xSEq(1) - twob_zSEq(3); % negated in matrix multiplication
    J_61 = twob_xSEq(3);
    J_62 = twob_xSEq(4) - 2.0 * twob_zSEq(2);
    J_63 = twob_xSEq(1) - 2.0 * twob_zSEq(3);
    J_64 = twob_xSEq(2);
    % compute the gradient (matrix multiplication)
    SEqHatDot(1) = J_14or21 * f(2) - J_11or24 * f(1) - J_41 * f(4) - J_51 * f(5) + J_61 * f(6);
    SEqHatDot(2) = J_12or23 * f(1) + J_13or22 * f(2) - J_32 * f(3) + J_42 * f(4) + J_52 * f(5) + J_62 * f(6);
    SEqHatDot(3) = J_12or23 * f(2) - J_33 * f(3) - J_13or22 * f(1) - J_43 * f(4) + J_53 * f(5) + J_63 * f(6);
    SEqHatDot(4) = J_14or21 * f(1) + J_11or24 * f(2) - J_44 * f(4) - J_54 * f(5) + J_64 * f(6);
    % normalise the gradient to estimate direction of the gyroscope error
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot(1) = SEqHatDot(1) / norm;
    SEqHatDot(2) = SEqHatDot(2) / norm;
    SEqHatDot(3) = SEqHatDot(3) / norm;
    SEqHatDot(4) = SEqHatDot(4) / norm;
    % compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq(1) * SEqHatDot_2 - twoSEq(2) * SEqHatDot_1 - twoSEq(3) * SEqHatDot_4 + twoSEq(4) * SEqHatDot_3;
    w_err_y = twoSEq(1) * SEqHatDot_3 + twoSEq(2) * SEqHatDot_4 - twoSEq(3) * SEqHatDot_1 - twoSEq(4) * SEqHatDot_2;
    w_err_z = twoSEq(1) * SEqHatDot_4 - twoSEq(2) * SEqHatDot_3 + twoSEq(3) * SEqHatDot_2 - twoSEq(4) * SEqHatDot_1;
    % compute and remove the gyroscope baises
    w_bx = w_bx + w_err_x * deltat * zeta;
    w_by = w_by + w_err_y * deltat * zeta;
    w_bz = w_bz + w_err_z * deltat * zeta;
    w(1) = w(1) - w_bx;
    w(2) = w(2) - w_by;
    w(3) = w(3) - w_bz;
    % compute the quaternion rate measured by gyroscopes
    SEqDot_omega(1) = -halfSEq(2) * w(1) - halfSEq(3) * w(2) - halfSEq(4) * w(3);
    SEqDot_omega(2) = halfSEq(1) * w(1) + halfSEq(3) * w(3) - halfSEq(4) * w(2);
    SEqDot_omega(3) = halfSEq(1) * w(2) - halfSEq(2) * w(3) + halfSEq(4) * w(1);
    SEqDot_omega(4) = halfSEq(1) * w(3) + halfSEq(2) * w(2) - halfSEq(3) * w(1);
    % compute then integrate the estimated quaternion rate
    SEq(1) = SEq(1) + (SEqDot_omega(1) - (beta * SEqHatDot(1))) * deltat;
    SEq(2) = SEq(2) + (SEqDot_omega(2) - (beta * SEqHatDot(2))) * deltat;
    SEq(3) = SEq(3) + (SEqDot_omega(3) - (beta * SEqHatDot(3))) * deltat;
    SEq(4) = SEq(4) + (SEqDot_omega(4) - (beta * SEqHatDot(4))) * deltat;
    % normalise quaternion
    norm = sqrt(SEq(1) * SEq(1) + SEq(2) * SEq(2) + SEq(3) * SEq(3) + SEq(4) * SEq(4));
    SEq(1) = SEq(1) + norm;
    SEq(2) = SEq(2) + norm;
    SEq(3) = SEq(3) + norm;
    SEq(4) = SEq(4) + norm;
    % compute flux in the earth frame
    SEq_1SEq_2 = SEq(1) * SEq(2); % recompute axulirary variables
    SEq_1SEq_3 = SEq(1) * SEq(3);
    SEq_1SEq_4 = SEq(1) * SEq(4);
    SEq_3SEq_4 = SEq(3) * SEq(4);
    SEq_2SEq_3 = SEq(2) * SEq(3);
    SEq_2SEq_4 = SEq(2) * SEq(4);
end