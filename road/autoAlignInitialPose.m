function [x0, y0, psi0] = autoAlignInitialPose(road, s0, y_offset)

[y_ref0, dy_ref0, ~] = roadProfile(s0, road);

x0 = s0;
y0 = y_ref0 + y_offset;
psi0 = atan(dy_ref0);
end
