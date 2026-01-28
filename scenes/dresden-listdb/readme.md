# Dresden dataset excerpt

## Full dataset

The full dataset is described in `dataset_readme.txt`. Here, there is an excerpt focussing on two vehicles.

## Excerpt

### Map

- Map name: `map_hauptbahnhof_nord_dresden.osm`
- Converted to `xodr`: `map_hauptbahnhof_nord_dresden_osm2xodr.xodr` (used `osm2xodr` https://github.com/JHMeusener/osm2xodr/tree/master for conversion)

### Video clip

A videoclip of the excerpt of the dataset is at `20220511_100012_Sid_StP_3W_d_1_1_org_sample.mp4`.

### Full data

The full data of the excerpt is contained in `20220511_100012_Sid_StP_3W_d_1_1_ann.csv`. The coordinate system used for the x-y-positions of vehicles is the  [Universal Transverse Mercator (UTM) coordinate system](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system), specifically **UTM zone 33**.

### Excerpt - 2 cars

In `two_cars.csv`, an excerpt focussing on only two cars from `20220511_100012_Sid_StP_3W_d_1_1_ann.csv` is found.

### Processing

To convert from the UTM33 coordinates found in the dataset excerpt (e.g., `two_cars.csv`) to a TMERC projection equivalent to that used by OpenDrive, use the notebook `csv_processing.ipynb`.