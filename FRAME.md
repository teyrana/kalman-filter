## Natural Axes of IMU

The natural axes of the 3-Space Sensor are as follows:
   - Place board flat
        - with the buttons upward
        - the plug faceing towards you
   - axes are left-handed

1. `X` - positive x points the right hand side of the sensor
2. `Y` - positive y points out up.
3. `Z` - positive z points away from you.

Bear in mind the difference between natural axes and the axes that are used in protocol data. While they are by default
the same, they can be remapped so that, for example, data axis Y could contain data from natural axis X. This allows
users to work with data in a reference frame they are familiar with. See the “Axis Assignment” section for a diagram of
the natural axes.

## World Axes (UUV Design Convention)

By convention the world axes are NED (North-East-Down).   Because depth is measured downwards.  This is a right-handed coordinate frame.


## Body Axes (UUV Convention)

![Body Frame]( doc/UUV-Body-Farme.jpg )

By convention the world axes are Forward-Right-Down (XYZ). This is a right-handed coordinate frame.

### Coordinate Axes:
    
Locate the Body frame, wrt the World Frame

1. X - Projects forward, through the nose / bow of the vehicle
2. Y - Projects leftward, starboard side of the vehicle
3. Z - Projects downwards, with the pull of gravity

### Euler Angles: World => Body

Locate the Body frame, wrt the World Frame

` R^{RW} ( ϕ, θ, ψ) = R_z(ψ) R_y(θ) R_x(ϕ) =  ϕ θ ψ ψ θ ϕ = = z y x

1. ϕ - (RHR about x-axis)
2. θ - (RHR about y-axis)
3. ψ - (LHR about z-axis)

### Orientation Angles (?):

1. _Roll_ - (RHR about x-axis)
2. _Pitch_ - (RHR about y-axis)
3. _Yaw_ - (LHR about z-axis)

#### Transient Motion:
1. Heave - Up/Down motion (==Δz) 
2. Sway - Side-to-side motion (==Δy)
3. Surge - Forward/Back Motion (==Δx)
