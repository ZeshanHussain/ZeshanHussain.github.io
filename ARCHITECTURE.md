# Architecture Overview

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Main Loop (60 Hz)                    │
│                      (main.cpp)                          │
└────────────┬────────────────────────────┬────────────────┘
             │                            │
             ▼                            ▼
    ┌────────────────┐          ┌─────────────────┐
    │ Input Handler  │          │   Renderer      │
    │                │          │  (OpenGL +      │
    │ - Keyboard     │          │   ImGui)        │
    │ - Controls     │          │                 │
    └────────┬───────┘          └────────┬────────┘
             │                           │
             ▼                           │
    ┌────────────────┐                  │
    │    Aircraft    │◄─────────────────┘
    │                │         (renders)
    │ - State        │
    │ - Properties   │
    │ - Aero Coeff   │
    └────────┬───────┘
             │
             ▼
    ┌──────────────────┐
    │ Flight Dynamics  │
    │                  │
    │ - Forces         │
    │ - Moments        │
    │ - RK4 Integration│
    └────────┬─────────┘
             │
             ▼
    ┌──────────────────┐
    │   Atmosphere     │
    │                  │
    │ - ISA Model      │
    │ - ρ, P, T, a     │
    └──────────────────┘
```

## Data Flow

```
User Input ──► InputHandler ──► Aircraft State
                                      │
                                      ▼
                            FlightDynamics.update()
                                      │
                    ┌─────────────────┴─────────────────┐
                    │                                   │
                    ▼                                   ▼
            calculateForces()                  calculateMoments()
                    │                                   │
                    └─────────────────┬─────────────────┘
                                      │
                                      ▼
                            RK4 Integration (4 steps)
                                      │
                                      ▼
                          Update Aircraft State
                                      │
                    ┌─────────────────┴─────────────────┐
                    │                                   │
                    ▼                                   ▼
            Instruments.render()              Renderer.render3D()
```

## Component Responsibilities

### 1. Aircraft (`aircraft.hpp/cpp`)
**Purpose**: Store and manage aircraft state and properties

**Data**:
- Position (x, y, z in NED frame)
- Velocity (u, v, w in body frame)
- Angular velocity (p, q, r in body frame)
- Euler angles (roll, pitch, yaw)
- Control inputs (elevator, aileron, rudder, throttle)
- Physical properties (mass, wing area, inertia tensor)

**Methods**:
- `getAirspeed()` - Calculate total velocity magnitude
- `getAltitude()` - Return altitude above ground
- `getAngleOfAttack()` - Calculate α = atan2(w, u)
- `getCL()`, `getCD()`, `getCm()`, etc. - Aerodynamic coefficients

### 2. FlightDynamics (`flight_dynamics.hpp/cpp`)
**Purpose**: Integrate equations of motion

**Core Algorithm**:
```
For each timestep dt:
    1. Calculate current forces (lift, drag, thrust, gravity)
    2. Calculate current moments (roll, pitch, yaw)
    3. Apply RK4 integration:
        k1 = f(state)
        k2 = f(state + k1*dt/2)
        k3 = f(state + k2*dt/2)
        k4 = f(state + k3*dt)
        new_state = state + (k1 + 2*k2 + 2*k3 + k4)*dt/6
    4. Update aircraft state
    5. Check ground collision
```

**Key Equations**:
- **Force**: F = ma → v_dot = F/m - ω × v
- **Moment**: M = Iω_dot → ω_dot = I⁻¹(M - ω × Iω)
- **Position**: p_dot = R(body→NED) · v
- **Euler rates**: [φ̇, θ̇, ψ̇]ᵀ = E(φ,θ) · [p, q, r]ᵀ

### 3. Atmosphere (`atmosphere.hpp/cpp`)
**Purpose**: Provide atmospheric properties at any altitude

**ISA Model**:
- **Troposphere (0-11 km)**:
  - T = T₀ - L·h
  - P = P₀·(T/T₀)^(g/(L·R))
  - ρ = P/(R·T)

- **Stratosphere (11-20 km)**:
  - T = const (216.65 K)
  - P = P₁₁·exp(-g·(h-h₁₁)/(R·T))
  - ρ = P/(R·T)

- **Speed of sound**: a = √(γ·R·T)

### 4. Instruments (`instruments.hpp/cpp`)
**Purpose**: Render cockpit instruments using ImGui

**Instruments**:
- **Airspeed Indicator**: V → knots, 0-200 scale
- **Altimeter**: h → feet, 0-10000 scale
- **Attitude Indicator**: roll & pitch → artificial horizon
- **Heading Indicator**: ψ → compass rose
- **VSI**: h_dot → feet/min, ±2000 scale
- **Turn Coordinator**: bank angle & turn rate

**Rendering**:
- Uses ImGui DrawList API for 2D vector graphics
- Circular gauges with needles
- Color coding (sky blue, ground brown, yellow aircraft)

### 5. Renderer (`renderer.hpp/cpp`)
**Purpose**: Manage OpenGL window and 3D visualization

**Responsibilities**:
- Initialize GLFW window
- Setup ImGui context
- Render 3D horizon view
- Draw ground reference
- Display compass tape
- Frame management (vsync, double buffering)

### 6. InputHandler (`input_handler.hpp/cpp`)
**Purpose**: Process user input and map to controls

**Control Mapping**:
```
Keyboard        →    Aircraft Control
──────────────────────────────────────
W/S, ↑/↓        →    Elevator (pitch)
A/D, ←/→        →    Aileron (roll)
Q/E             →    Rudder (yaw)
Z/X, PgUp/PgDn  →    Throttle
Space           →    Center controls
P               →    Pause/Resume
R               →    Reset
ESC             →    Exit
```

**Features**:
- Rate-based control (not position-based)
- Automatic centering when no input
- Deadzone to prevent drift

## Coordinate Systems

### NED (North-East-Down) - Inertial Frame
```
     North (x)
       ↑
       │
       │
       └────► East (y)
      ╱
     ╱
    ↓ Down (z)
```

### Body Frame - Aircraft-Fixed
```
     Forward (x)
       ↑
       │ (nose)
       │
       └────► Right (y)
      ╱
     ╱
    ↓ Down (z)
```

### Transformations
- **Body → NED**: Use rotation matrix R(φ,θ,ψ)
- **Wind → Body**: Rotate by α (angle of attack) and β (sideslip)

## Aerodynamic Model

### Forces (Body Frame)
```
X = -D·cos(α) + L·sin(α) + T - mg·sin(θ)
Y = Y_side + mg·sin(φ)·cos(θ)
Z = -D·sin(α) - L·cos(α) + mg·cos(φ)·cos(θ)
```

Where:
- L = ½ρV²S·CL (Lift)
- D = ½ρV²S·CD (Drag)
- Y = ½ρV²S·CY (Side force)
- T = Thrust

### Moments (Body Frame)
```
L_roll  = ½ρV²S·b·Cl  (Rolling moment)
M_pitch = ½ρV²S·c·Cm  (Pitching moment)
N_yaw   = ½ρV²S·b·Cn  (Yawing moment)
```

### Coefficient Breakdown
```
CL = CL₀ + CLα·α + CLδe·δe
CD = CD₀ + K·CL²
Cm = Cm₀ + Cmα·α + Cmδe·δe + Cmq·q̂
Cl = Clβ·β + Clδa·δa + Clδr·δr + Clp·p̂
Cn = Cnβ·β + Cnδa·δa + Cnδr·δr + Cnr·r̂
```

## Integration Method: RK4

**4th Order Runge-Kutta**:
```
k₁ = f(t, y)
k₂ = f(t + dt/2, y + k₁·dt/2)
k₃ = f(t + dt/2, y + k₂·dt/2)
k₄ = f(t + dt, y + k₃·dt)

y_new = y + (k₁ + 2k₂ + 2k₃ + k₄)·dt/6
```

**Advantages**:
- 4th order accuracy: error ~ O(dt⁵)
- Stable for typical timesteps (dt = 1/60 sec)
- Industry standard for flight simulation

## Performance Characteristics

- **Simulation Rate**: 60 Hz (16.67 ms timestep)
- **Render Rate**: Vsync-limited (typically 60 FPS)
- **Integration**: RK4 with fixed timestep
- **Accumulator**: Handles variable frame time

```
accumulator += deltaTime
while (accumulator >= dt):
    update_physics(dt)
    accumulator -= dt
render()
```

## Extension Points

### Adding New Aircraft
1. Create new class inheriting from `Aircraft`
2. Override aerodynamic coefficient methods
3. Set physical properties in constructor

### Adding Weather
1. Extend `Atmosphere` with wind model
2. Add wind velocity to velocity calculations
3. Add turbulence as random perturbations

### Adding Navigation
1. Create `Navigation` class
2. Store waypoints
3. Calculate bearing/distance in `update()`
4. Display in new instrument

### Adding Autopilot
1. Create `Autopilot` class
2. Implement PID controllers for altitude, heading, airspeed
3. Override control inputs from `InputHandler`

## Dependencies

```
flight_simulator
├── OpenGL 3.3+      (graphics)
├── GLFW3            (windowing & input)
└── ImGui            (GUI rendering)
    ├── imgui.cpp
    ├── imgui_draw.cpp
    ├── imgui_widgets.cpp
    ├── imgui_tables.cpp
    ├── imgui_impl_glfw.cpp
    └── imgui_impl_opengl3.cpp
```

## Build System

**CMake Flow**:
```
CMakeLists.txt
    ├── Find OpenGL
    ├── Find GLFW
    ├── Include ImGui sources
    ├── Compile src/*.cpp
    ├── Link libraries
    └── Output: flight_simulator
```

## File Organization

```
Total Lines of Code: ~2500

include/         ~600 lines
  - vector3.hpp        ~80
  - quaternion.hpp    ~100
  - atmosphere.hpp     ~30
  - aircraft.hpp       ~70
  - flight_dynamics.hpp ~50
  - instruments.hpp     ~50
  - renderer.hpp        ~50
  - input_handler.hpp   ~50

src/            ~1900 lines
  - atmosphere.cpp      ~50
  - aircraft.cpp       ~150
  - flight_dynamics.cpp ~200
  - instruments.cpp     ~700
  - renderer.cpp       ~300
  - input_handler.cpp  ~100
  - main.cpp           ~150
```

---

This architecture balances:
- **Modularity**: Clear separation of concerns
- **Performance**: Efficient 60 Hz simulation
- **Extensibility**: Easy to add features
- **Realism**: Physics-based modeling

