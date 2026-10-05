# Single-Window Unified Swarm Visualizer for Windows (runs inside WSL2 Ubuntu)
# ONE Gazebo window: simulated drones plus the swarm's merged map, frontiers, exclusion zones and HUD.
Write-Host "========================================================================" -ForegroundColor Green
Write-Host "Launching Unified Single-Window Swarm Simulation..." -ForegroundColor Green
Write-Host "   Physics Engine: Gazebo Harmonic (real rotor dynamics, depth cameras, IMU)" -ForegroundColor Cyan
Write-Host "   Display: ONE Gazebo window with merged map, frontiers, exclusion zones and HUD" -ForegroundColor Yellow
Write-Host "========================================================================" -ForegroundColor Green

$wslPath = wsl -d Ubuntu wslpath -a "$PSScriptRoot"
$launchArgs = $args -join ' '
wsl -d Ubuntu bash -c "cd '$wslPath' && chmod +x ./run_gazebo_swarm.sh ./run_single_window.sh && ./run_single_window.sh $launchArgs"
