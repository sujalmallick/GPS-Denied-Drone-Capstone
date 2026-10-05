#!/bin/sh
# TurboVNC session for the swarm simulation desktop: Fluxbox plus one terminal.
# Apps started from the terminal need `vglrun` to render on the GPU (e.g. `vglrun rviz2`).
unset SESSION_MANAGER DBUS_SESSION_BUS_ADDRESS
xsetroot -solid "#1f2430"
xterm -geometry 110x24+20+20 -title "swarm shell" -e bash -l &
exec fluxbox
