import rirun.kinetics.stats as stats
import carla

# data (Tuple): Reference time (seconds, from raw trajectory), reference location (carla.Location, from raw trajectory), reference_speed (carla.Vector3D km/h, from raw trajectory), simulation time (seconds), simulation location (carla.Location)

# same time, offset in space, no speed
offset = 15
data = [(i, carla.Location(offset, 0, 0), carla.Vector3D(0, 0, 0), i, carla.Location(0, 0, 0)) for i in range(10)]

stat = stats.Average_Distance_Interpolated()
for d in data:
    stat.add(d)

assert(stat.evaluate() - offset == 0)


# offset in time, no offset in space, some speed
speed_m_s = 2
delta = 2
data = [(i, carla.Location(i, 0, 0), carla.Vector3D(speed_m_s * 3.6, 0, 0), i + delta, carla.Location(i, 0, 0)) for i in range(10)]

stat = stats.Average_Distance_Interpolated()
for d in data:
    stat.add(d)

assert(stat.evaluate() - speed_m_s * delta == 0)
