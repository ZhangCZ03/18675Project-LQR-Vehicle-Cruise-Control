function ang = wrapToPiLocal(ang)

ang = mod(ang + pi, 2*pi) - pi;
end
