function omegad = omega(theta, angc)
    tmp = theta-angc;
    if tmp > pi
        omegad = tmp - 2*pi;
    elseif tmp < -pi
        omegad = tmp + 2*pi;
    else
        omegad = tmp;
    end
end

