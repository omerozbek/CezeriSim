#!/bin/sh
UE_TRUE_SCRIPT_NAME=$(echo \"$0\" | xargs readlink -f)
UE_PROJECT_ROOT=$(dirname "$UE_TRUE_SCRIPT_NAME")
chmod +x "$UE_PROJECT_ROOT/CezeriSim/Binaries/Linux/CezeriSim"
"$UE_PROJECT_ROOT/CezeriSim/Binaries/Linux/CezeriSim" CezeriSim "$@" 
