function [x0p,y0p,x1p,y1p,x2p,y2p,x3p,y3p,x4p,y4p] = draw_2D_ship(x,y,t)

rp = 2.85;
rf = 4;
rcf = 2.8;
rcb = 1;
rb = 1;
radius_f = 4;
radius_b = 1.8;
r1f = 3;
r1b = 1.7;
radius_1f = 1.1;
radius_1b = 1.5;
r2f = 2.6;
r2b = 2.3;
radius_2f = 0.7;
radius_2b = 1.1;
r3 = -0.3;
r3b = -2.2;
radius_3b = 1.05;
r4 = 0.3;
r4x = -0.4;
r4b = -0.8;
radius_4b = 0.35;

x1f = x + rf * cos(t);
y1f = y + rf * sin(t);

xp = x1f + rp * cos(t);
yp = y1f + rp * sin(t);

xccl = x1f + rcf * sin(t);
yccl = y1f - rcf * cos(t);

xccr = x1f - rcf * sin(t);
yccr = y1f + rcf * cos(t);

xl = x + rb * sin(t);
yl = y - rb * cos(t);

xr = x - rb * sin(t);
yr = y + rb * cos(t);

xccc = x + rcb * cos(t);
yccc = y + rcb * sin(t);

circr = @(x,y,radius,rad_ang)  [x+radius*cos(rad_ang);  y+radius*sin(rad_ang)];

N = 10;                                                        
al = linspace(1.65+t, 0.79+t, N); 
ar = linspace(t-0.79, t-1.65, N);
ab = linspace(t+3.67, t+2.60, N);
cl = circr(xccl,yccl,radius_f,al); 
cr = circr(xccr,yccr,radius_f,ar); 
cb = circr(xccc,yccc,radius_b,ab); 
xcl = cl(1,:);
ycl = cl(2,:);
xcr = cr(1,:);
ycr = cr(2,:);
xcb = cb(1,:);
ycb = cb(2,:);

x0p = [xp xcr xcb xcl xp];
y0p = [yp ycr ycb ycl yp];


x1f = x + r1f * cos(t);
y1f = y + r1f * sin(t);

x1b = x + r1b * cos(t);
y1b = y + r1b * sin(t);

a1f = linspace(t+1.1, t-1.1, N);
a1b = linspace(t+3.67, t+2.60, N);
c1f = circr(x1f,y1f,radius_1f,a1f);
c1b = circr(x1b,y1b,radius_1b,a1b);
x1cf = c1f(1,:);
y1cf = c1f(2,:);
x1cb = c1b(1,:);
y1cb = c1b(2,:);

x1p = [x1cf x1cb x1cf(1)];
y1p = [y1cf y1cb y1cf(1)];


x2f = x + r2f * cos(t);
y2f = y + r2f * sin(t);

x2b = x + r2b * cos(t);
y2b = y + r2b * sin(t);

a2f = linspace(t+1.2, t-1.2, N);
a2b = linspace(t+3.67, t+2.60, N);
c2f = circr(x2f,y2f,radius_2f,a2f);
c2b = circr(x2b,y2b,radius_2b,a2b);
x2cf = c2f(1,:);
y2cf = c2f(2,:);
x2cb = c2b(1,:);
y2cb = c2b(2,:);

x2p = [x2cf x2cb x2cf(1)];
y2p = [y2cf y2cb y2cf(1)];


x3 = xp + r3 * cos(t);
y3 = yp + r3 * sin(t);

x3b = xp + r3b * cos(t);
y3b = yp + r3b * sin(t);

a3b = linspace(t+0.75, t-0.75, N);
c3b = circr(x3b,y3b,radius_3b,a3b);
x3cb = c3b(1,:);
y3cb = c3b(2,:);

x3p = [x3 x3cb x3];
y3p = [y3 y3cb y3];


x4l = x + r4x * cos(t) + r4 * sin(t);
y4l = y + r4x * sin(t) - r4 * cos(t);

x4r = x + r4x * cos(t) - r4 * sin(t);
y4r = y + r4x * sin(t) + r4 * cos(t);

x4b = x + r4b * cos(t);
y4b = y + r4b * sin(t);

a4b = linspace(t+4.07, t+2.20, N);
c4b = circr(x4b,y4b,radius_4b,a4b);
x4cb = c4b(1,:);
y4cb = c4b(2,:);

x4p = [x4l x4cb x4r x4l];
y4p = [y4l y4cb y4r y4l];

end
