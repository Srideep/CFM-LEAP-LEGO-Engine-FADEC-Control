function [profile, T_total] = throttle_profiles(name)
%THROTTLE_PROFILES  Pre-built throttle scenarios for FADEC simulation.
%
%   [profile, T_total] = throttle_profiles('departure')
%
%   Each profile is a matrix: [t_start, t_end, tla_deg]
%   T_total is the simulation duration.

switch lower(name)
    case 'departure'
        profile = [
            0    15   -30    % Start sequence
           15    40   -30    % Taxi at idle
           40    42    30    % Advance to 40% (symmetry check)
           42    44    90    % Set TOGA
           44    55    90    % Takeoff roll + climb
           55    57    60    % Reduce to climb thrust
           57    70    60    % Hold climb
           70    75    60    % Continue
        ];
        T_total = 75;

    case 'arrival'
        profile = [
            0    15   -30    % Start sequence
           15    20    50    % Cruise
           20    35    50    % Hold cruise
           35    37   -30    % Top of descent — idle
           37    55   -30    % Idle descent
           55    57    20    % Approach thrust
           57    68    20    % Final approach
           68    70   -30    % Flare — idle
           70    75   -30    % Touchdown
        ];
        T_total = 75;

    case 'full_flight'
        profile = [
            0    15   -30    % Start sequence
           15    40   -30    % Taxi out
           40    42    30    % Symmetry check
           42    44    90    % TOGA
           44    55    90    % Takeoff + initial climb
           55    57    60    % Reduce to climb
           57    70    60    % Climb
           70    72    50    % Cruise
           72    90    50    % Hold cruise
           90    92   -30    % Top of descent
           92   105   -30    % Idle descent
          105   107    20    % Approach
          107   115    20    % Final approach
          115   117   -30    % Flare
          117   120   -30    % Touchdown
        ];
        T_total = 120;

    case 'go_around'
        profile = [
            0    15   -30    % Start sequence
           15    20    50    % Cruise
           20    30   -30    % Descend at idle
           30    40    20    % Approach
           40    42   120    % GO AROUND — slam to TOGA
           42    55   120    % Full thrust climb-out
           55    57    60    % Reduce to climb
           57    65    60    % Hold climb
           65    67   -30    % Second approach
           67    75   -30    % Land
        ];
        T_total = 75;

    case 'stress_test'
        profile = [
            0    15   -30    % Start sequence
           15    17   120    % Idle to max TOGA (test accel limiter)
           17    30   120    % Hold TOGA (test EGT limiting)
           30    32   -30    % Snap to idle (test decel limiter)
           32    35   -30    % Hold idle
           35    37   120    % Slam to TOGA again
           37    45   120    % Sustained TOGA
           45    47   -30    % Snap idle
           47    50   120    % Rapid cycle
           50    52   -30    % Rapid cycle
           52    55   120    % Rapid cycle
           55    60   -30    % Cool down
        ];
        T_total = 60;

    otherwise
        error('Unknown profile: %s\nAvailable: departure, arrival, full_flight, go_around, stress_test', name);
end
end