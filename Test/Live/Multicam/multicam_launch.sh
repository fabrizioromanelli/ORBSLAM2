#!/bin/bash
thisDir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
$thisDir/multicam_live ~/workspace/ORBSLAM2/Vocabulary/orb_mur.fbow ~/workspace/ORBSLAM2/Config/RealSense-D435i-MULTI.yaml OFF ON OFF