Look at the center number: $500.0,\text{m}$.

As you walk away from the center in any direction, the numbers slope downwards ($280\text{m} \to 135\text{m} \to 102\text{m}$).
That equation in generate_gaussian_ridge: $$z(x, y) = 100 + 400 \cdot \exp\left( -\left[\frac{(x - c_x)^2}{2\sigma_x^2} + \frac{(y - c_y)^2}{2\sigma_y^2}\right] \right)$$ is just the mathematical formula for a 3D bell curve! It drops a 400-meter-tall mountain peak right on top of a 100-meter plain.

2. What Did generate_rolling_hills Do? (Octaves / Wave Stacking)
In nature, mountains aren't simple smooth bell curves. They have large ridges, smaller foothills, and tiny bumps.

In generate_rolling_hills, we used Octave Synthesis (the foundation of procedural terrain generation in computer graphics and game engines like Unreal Engine):

Octave 1 (The Giant Wave): A massive wave with wavelength $400,\text{meters}$ and amplitude $80,\text{meters}$ (the main mountain ridges).
Octave 2 (The Foothills): We halved the wavelength to $200,\text{meters}$ and halved the height to $40,\text{meters}$, adding it on top.
Octave 3 (The Bumps & Gullies): Wavelength $100,\text{meters}$, height $20,\text{meters}$, adding fine texture.

When you add those three sine waves together at random angles, they form natural-looking valleys, saddles, and peaks!

3. Let's Actually See It! (Building a 3D & 2D Visualizer)
Let's create a script in scripts/visualize_subphase1_1.py that will render:

A 3D surface plot of the Gaussian Peak and Rolling Hills.
A 2D contour topographic map (like a real pilot's navigation chart).
A microscopic zoom-in on Bilinear Interpolation, showing the 4 corner posts and the smooth curved sheet stretching between them with our test drone sitting at $(x=25, y=75)$ at height $130,\text{m}$!
