# Laser interface

The hardware interface publishes up to two laser scans from the Omron controller through libaria:

- `/scan` from the front safety laser in `laser_frame`
- `/scan_low` from the low front laser in `laser_frame_low`

The main laser remains the primary scan used for mapping and navigation. The low laser is downward-looking and is disabled or enabled with configuration depending on the robot and use case.

## Parameter layout

Laser configuration lives under `amr_core.ros__parameters.laser` in the robot-specific parameter files:

```yaml
laser:
	main_laser:
		topic: "scan"
		frame_id: "laser_frame"
		request: "Laser_1Current"
		request_period_ms: 200
		angle_min: -2.09439510239
		angle_max: 2.09439510239
		angle_increment: 0.00872664626
		range_min: 0.02
		range_max: 15.0
	low_laser:
		enabled: true
		topic: "scan_low"
		frame_id: "laser_frame_low"
		request: "Laser_2Current"
		request_period_ms: 200
		angle_min: -1.09955742876
		angle_max: 1.09955742876
		angle_increment: 0.00872664626
		range_min: 0.02
		range_max: 4.0
```

`Laser_1Current` and `Laser_2Current` are the libaria current range-device streams used for the main and low scanners.

## Datasheet-derived defaults

### LD90

- Main safety laser: 240 degree field of view, 15 m range
- Low front laser: 126 degree field of view, 4 m range

### LD250

- Main safety laser: 240 degree field of view, 40 m general sensing range
- Low front laser: 126 degree field of view, 4 m range

## Angle increment behavior

The configured `angle_min` and `angle_max` define the scan span. At runtime, the interface computes the published increment from the number of points in the packet:

$$
	ext{angle\_increment} = \frac{\text{angle\_max} - \text{angle\_min}}{N - 1}
$$

where $N$ is the number of returned points in that packet. This keeps the `LaserScan` geometry aligned with the actual point count instead of assuming a fixed angular resolution.

## Notes

- The low laser is useful for close-range floor-level sensing.
- The low laser is not automatically wired into Nav2 obstacle sources.
- If the robot returns sparse data, the scan may contain `NaN` bins where no point landed in that angular slot.
