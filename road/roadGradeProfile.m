function theta = roadGradeProfile(x, dist)

switch dist.gradeType
    case 0
        theta = 0;

    case 1
        theta = dist.gradeValue;

    case 2
        if x >= dist.gradeStart && x <= dist.gradeEnd
            theta = dist.gradeValue;
        else
            theta = 0;
        end

    case 3
        theta = dist.gradeAmp * sin(2*pi*x/dist.gradeWave);

    otherwise
        warning('UnknownGrade%g', dist.gradeType);
        theta = 0;
end
end
