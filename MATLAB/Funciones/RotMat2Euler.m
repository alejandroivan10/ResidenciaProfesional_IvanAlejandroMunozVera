function euler = RotMat2Euler(R)
    % from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
    % Heading Reference System" by Wang et al, IEEE.
    
    % R(Phi).*R(Theta).*R(Psi)
%     phi   =-atan2d( R(2,3), R(3,3) );
%     theta =  asind(-R(3,1) );    
%     psi   = atan2d( R(2,1), R(1,1) );
    
    % Rz(Psi).*Ry(Theta).*Rx(Phi)
    phi   = atan2d( R(3,2), R(3,3) );
    theta =  asind(-R(3,1) );
    psi   = atan2d( R(2,1), R(1,1) );

    euler = [phi theta psi]; 
end

