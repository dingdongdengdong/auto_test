# path_maker package (ROS1)

The `path_maker` package records GPS-based paths to plain text files for later use in planning and control. It provides two recorders:

- `src/path_maker.py`: saves raw WGS84 latitude/longitude/altitude.
- `src/path_maker_utmk.py`: converts WGS84 to UTM Zone 52 coordinates (meters) and saves x/y/z.

Both scripts subscribe to `/gps` (`morai_msgs/GPSMessage`) and write to a file under the package.

## Outputs

Each recorded line has 4 tab-separated columns:

- x or latitude
- y or longitude
- z (altitude or fixed 0 in UTM mode)
- mode (reserved integer, currently 0)

Examples:

- UTM: `402465.9826\t4132959.4911\t0\t0`
- WGS84: `37.389123\t127.123456\t83.2\t0`

## Nodes and topics

- Subscribes: `/gps` (`morai_msgs/GPSMessage`)
- No publishers; the scripts only write files.

## Usage

### Launch

Edit `launch/path_maker.launch` to select the desired output file. The args are `<folder> <filename_base>` and the output path is:

`<pkg_path>/<folder>/<filename_base>.txt`

For example:

```xml
<node pkg="path_maker" type="path_maker_utmk.py" name="maker" args="path tmp" output="screen" />
```

This will produce `$(rospack find path_maker)/path/tmp.txt`.

### rosrun

- Raw WGS84:

```bash
rosrun path_maker path_maker.py path first
```

- UTM Zone 52 (meters):

```bash
rosrun path_maker path_maker_utmk.py path first
```

Make sure the destination folder exists under the package (e.g., `path_maker/path`).

## Coordinate systems

- `path_maker.py` records WGS84 lat/lon degrees directly; spacing filter is ~1e-5 degrees.
- `path_maker_utmk.py` converts lon/lat to UTM Zone 52 (WGS84 ellipsoid) using `pyproj`. Spacing filter is 0.3 meters.

Notes:
- UTM Zone 52 covers much of Korea; adjust if your region differs.
- The code uses `Proj(proj='utm', zone=52, elips='WGS84')`. Some pyproj versions prefer `ellps`.

## File structure

- `src/path_maker.py`: recorder in WGS84 degrees.
- `src/path_maker_utmk.py`: recorder in UTM meters.
- `src/utmk_to_odom.py`: example helper showing TF broadcast and file rewrite (paths are hard-coded; adjust before use).
- `path/trans.py`: example coordinate conversion between CRS (paths are hard-coded; adjust).
- `path/*.txt`: example path files.

## Data rate & filters

- Main loop runs at 30 Hz, but points are downsampled by a spacing filter to reduce file size.
- UTM: 0.3 m spacing. WGS84: 1e-5 deg (~1 m scale near mid-latitudes).

## Dependencies

- ROS1 (tested Kinetic/Noetic), `rospy`, `rospkg`
- Messages: `morai_msgs/GPSMessage`
- `pyproj` for coordinate conversion

## Troubleshooting

- If no file is written, verify `/gps` is publishing `morai_msgs/GPSMessage`.
- Ensure the target folder exists and the node has write permissions.
- If pyproj raises a parameter error for ellipsoid, change `elips` to `ellps`.

## License

See `package.xml`.
