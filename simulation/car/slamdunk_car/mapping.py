"""Conservative bounded projection of SLAM's 2D grid into SANDO columns."""
import math


def project_grid(data,width,height,resolution,origin,bounds=(-12.,12.,-9.,9.),step=.2,hits=()):
    if width<1 or height<1 or width*height!=len(data) or width*height>2_000_000:
        raise ValueError('invalid/bounded map dimensions')
    if not all(math.isfinite(v) for v in (*origin,*bounds,resolution,step)) or not 0<resolution<=step:
        raise ValueError('invalid map resolution or origin')
    xmin,xmax,ymin,ymax=bounds;nx=round((xmax-xmin)/step);ny=round((ymax-ymin)/step)
    if not 0<nx*ny<=50000 or nx<1 or ny<1:raise ValueError('invalid mission bounds')
    if any(not isinstance(v,int) or v< -1 or v>100 for v in data):raise ValueError('invalid occupancy value')
    occupied=[];unknown=[];cells=[];hitset=set(hits)
    for iy in range(ny):
        y=ymin+iy*step
        for ix in range(nx):
            x=xmin+ix*step
            x0=math.floor((x-origin[0])/resolution+1e-7);x1=math.ceil((x+step-origin[0])/resolution-1e-7)
            y0=math.floor((y-origin[1])/resolution+1e-7);y1=math.ceil((y+step-origin[1])/resolution-1e-7)
            values=[data[sy*width+sx] if 0<=sx<width and 0<=sy<height else -1 for sy in range(y0,y1) for sx in range(x0,x1)]
            value=100 if (ix,iy) in hitset or any(v>=65 for v in values) else (0 if values and all(v==0 for v in values) else -1)
            cells.append(value);point=(x+step/2,y+step/2)
            if value==100:occupied.append(point)
            elif value!=0:unknown.append(point)
    return occupied,unknown,cells,nx,ny


def columns(points,zmin=0.,zmax=1.,step=.2):
    levels=round((zmax-zmin)/step)+1
    if not 1<=levels<=100 or len(points)*levels>1_000_000:raise ValueError('cloud limit exceeded')
    return [(x,y,zmin+i*step) for x,y in points for i in range(levels)]


def quaternion_yaw(q):
    x,y,z,w=q;norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or abs(norm-1)>.05:raise ValueError('invalid quaternion')
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def transform_point(point,translation,rotation):
    x,y,z=point;qx,qy,qz,qw=rotation
    # Quaternion rotation without depending on numpy.
    tx=2*(qy*z-qz*y);ty=2*(qz*x-qx*z);tz=2*(qx*y-qy*x)
    return (x+qw*tx+qy*tz-qz*ty+translation[0],y+qw*ty+qz*tx-qx*tz+translation[1],z+qw*tz+qx*ty-qy*tx+translation[2])
