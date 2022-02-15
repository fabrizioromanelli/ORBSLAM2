#!/bin/bash
thisDir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
$thisDir/multicam_live /usr/local/share/ORB_SLAM2/Vocabulary/orb_mur.fbow /usr/local/share/ORB_SLAM2/Config/RealSense-D435i-IRD.yaml OFF ON ON