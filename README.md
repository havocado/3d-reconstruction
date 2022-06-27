# 3d-reconstruction
# Intrinsics & Extrinsics
## Samples -> NDC coord
![A grayscale depth image displaying a scene produced by habitat simulator](img/readme_1.png)

Samples from habitat are obtained as images: (row j, col i, depth)

For any pixel in image, we can get coordinates

$$NDC \ Coords: 
\begin{bmatrix} 
x_n \\ 
y_n \\ 
z_n 
\end{bmatrix}
\ , \ \ x_n, y_n \in [-1, 1] \ , \ \ z_n \in [0, 1]$$

Where x: right, y: up, z: towards screen and camera located at (0, 0, 0).

Code:
```matlab
% Sampling pixel (i, j) from image im
% 1 <= i <= w, 1 <= j <= h, 0 <= im(j, i) <= 1.0
x_n(count) = (i - w/2) / w;
y_n(count) = ((-1)*j + h/2)/h;
z_n(count) = (-1)*(im(j, i));
```

## Downsampling & Stepsize
For fast computation the images are downsampled:
```
% Downsampling using stepSize parameter
count = 0;
x_n = [];
y_n = [];
z_n = [];
for i=1:stepSize:w
	for j=1:stepSize:h
		if (im(j, i) < 0.15 || im(j, i) > 0.85), continue; end
		count = count + 1;
		x_n(count) = (i - w/2)/w;
		y_n(count) = ((-1)*j + h/2)/h;
		z_n(count) = (-1)*(im(j, i));
	end
end
NDC = [x_n; y_n; z_n];
```

## Intrinsics
#### Input from Habitat: width, height, near, far, hfov
Near: 0.01, Far: 1000.0
- Near, Far: float
	- [CameraSensor Class doc](https://aihabitat.org/docs/habitat-sim/habitat_sim.sensor.CameraSensor.html)
	- Hard coded in `src/esp/sensor/VisualSensor.h` : [here](https://github.com/facebookresearch/habitat-sim/blob/ec81c8cfe50b35f98edc4a390ee9b46a961b7144/src/esp/sensor/VisualSensor.h#:~:text=*/-,float%20near%20%3D%200.01f%3B,*/,-Mn%3A%3AColor4%20clearColor)
		- near: 0.01, far: 1000.0

hfov: 90 (degrees)
- hfov: degree
	- from `settings.py`

width, height: 640, 480
	- from `settings.py`

#### Near plane size (screen x, y-length)
screen x-length : $sx =  2 \cdot near \cdot tan(\large\frac{hfov}{2})$

screen y-length: $sy = sx \cdot \large\frac{resolution_x}{resolution_y}$ where resolution: width, height.

![Derivation of formula above, written on paper](img/readme_2.png)

```MATLAB
width = 640; height = 480;
near = 0.01; far = 1000.0; hfov = 90*(2*PI/180);
s_x = 2 * near * tan(hfov/2);
s_y = s_x * width/height;
```

#### Near plane top/bottom/right/left
top: sy/2, bottom: -sy/2, right: sx/2, left: -sx/2

Code:
```matlab
near = 0.01; far = 1000.0;
[h, w] = size(im);
hfov = 90*(2*pi/360);
s_x = 2 * near * tan(hfov/2);
s_y = s_x * h/w;
r = s_x/2;
t = s_y/2;
```


## Extrinsics
Code to parse habitat output log to csv: [[parse output to csv]]
(nothing special with this code, just reading habitat output and parsing into csv file)
Outputs 3 values for location and 4 values for rotation quaternion.

## Rotation & Quaternion
#### Modifying camera location
Our code: 
- x: right, y: up, z: towards screen

Habitat output:
- x: towards screen, y: up, z: right

#### Modifying quaternion

![2 plots drawn on paper demonstration conversion from habitat quaternion to our quaternion](img/readme_3.png)

Hence
```matlab
modifiedCameraPos = [extrinsics(i,3); extrinsics(i,2); extrinsics(i,1)]';
modifiedQuaternion = [extrinsics(i,4); (-1)*extrinsics(i,7); extrinsics(i,6); (-1)*extrinsics(i,5)]';
```

