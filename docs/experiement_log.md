# Experiment Log

Current findings from the LD-250 work. See [testing](../amr_core/test/README.md) for test
results and commands. These records are not authorization for new robot access,
configuration changes, or movement.

## Configuration

- Translation dead zone was changed from 10% to 14% and verified after one
  reboot; rotation was 5%. Teleop timeout was changed from 2.0 s to 1.0 s and
  independently read back, but its reboot persistence and physical watchdog
  behavior remain unverified.
- The local driver reverse clamp is -0.2 m/s; the Nav2 smoother uses -0.1 m/s.
  The driver retains `ratio_mapping: legacy`, `calibration.confirmed: false`,
  and `unsafe_drive: true`. Legacy commands are not calibrated SI velocities.
- Firmware ratio semantics, effective limits, odometry accuracy, ownership,
  stopping behavior and protective functions still need independent validation.

## Docking and Motion

- Two separately authorized firmware-backed undock actions timed out without
  observed departure and with stop unconfirmed. The operator then unplugged the
  charger and rebooted; later preflights reported Undocked. The cause remains
  unresolved; see [known issues](../known_issues.md).
- Supervised forward, reverse and curved commands produced movement. A -0.2 m/s
  reverse request yielded approximately -0.048 m/s reported steady velocity.
  Longer forward and curved attempts hit the odometry displacement guard and
  had failed immediate readbacks. Stop requests are not acknowledged.
- These observations do not establish calibrated velocity, distance tracking,
  trajectory following, watchdog stopping or protective safety. Independent
  physical distance measurements are needed to validate odometry and motion.

Private audits are retained under the workspace's `build/amr_core/calibration/`,
`build/amr_core/undocking/`, and `build/amr_core/drive-pulse/` directories.
Raw captures and full configuration exports can contain credentials and must
remain private and out of source control.