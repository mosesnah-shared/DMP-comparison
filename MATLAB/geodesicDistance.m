function distance = geodesicDistance(R1, R2)
    % Compute the geodesic distance between two rotation matrices.
    %
    % Args:
    %   R1: A 3x3 rotation matrix.
    %   R2: A 3x3 rotation matrix.
    %
    % Returns:
    %   distance: The geodesic distance between the two rotation matrices.

    % Validate input matrices
    if ~isequal(size(R1), [3, 3]) || ~isequal(size(R2), [3, 3])
        error('Input matrices must be 3x3.');
    end
    
    % Compute the relative rotation matrix
    R = R1' * R2;
    
    % Ensure the matrix is valid
    if det(R) < 0
        error('Invalid rotation matrix. Determinant must be positive.');
    end

    % Calculate the angle of rotation
    % angle = acos((trace(R) - 1) / 2)
    angle = acos(max(min((trace(R) - 1) / 2, 1), -1)); % Clamp between [-1, 1]

    % Geodesic distance is the angle itself
    distance = angle;
end