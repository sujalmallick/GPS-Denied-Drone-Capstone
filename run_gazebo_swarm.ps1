# Master 3-Drone Gazebo Swarm Simulation Launcher for Windows (runs inside WSL2 Ubuntu)
# Usage: .\run_gazebo_swarm.ps1               ONE Gazebo window with the swarm overlay
#        .\run_gazebo_swarm.ps1 rviz:=true    also open RViz2
Write-Host "========================================================================" -ForegroundColor Green
Write-Host "Starting Multi-Drone Gazebo Simulation Stack via WSL2..." -ForegroundColor Green
Write-Host "   Engine: Gazebo Harmonic + ROS 2 Jazzy + Swarm Autonomy Pipeline" -ForegroundColor Cyan
Write-Host "========================================================================" -ForegroundColor Green

$wslPath = wsl -d Ubuntu wslpath -a "$PSScriptRoot"
Write-Host "Workspace WSL Path: $wslPath" -ForegroundColor Yellow
$launchArgs = $args -join ' '
wsl -d Ubuntu bash -c "cd '$wslPath' && chmod +x ./run_gazebo_swarm.sh ./run_single_window.sh && ./run_gazebo_swarm.sh $launchArgs"
