% makeVgGammaCourse.m: Create groundspeed, flight path angle (gamma) and 
%   course from a ground-relative vector in NED coordinates
%
%   [Vg gamma course] = makeVgGammaCourse(vg_ned)
%      Inputs: 
%        vg_ned:  3-element vector representing ground-relative velocity
%                 in NED coordinates
%      Outputs:
%        Vg:      Airspeed (scalar). (units are same as input vector)
%        gamma:   Vertical flight path angle, radians (+Up)
%        course:  Course (horiz. flight path angle), radians (+East-of-North)
%
function [Vg, gamma, course] = makeVgGammaCourse(vg_ned)

    % Replace the following with appropriate code (use atan2!)
    vn = vg_ned(1,1);
    ve = vg_ned(2,1);
    vd = vg_ned(3,1);
    Vg = sqrt(vn^2 + ve^2 + vd^2);
    if Vg ~= 0
        course = atan2(ve,vn); % radians
        gamma = -asin(vd/Vg); % radians
    else
        course = 0;
        gamma = 0;

end
