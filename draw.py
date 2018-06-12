from display import *
from matrix import *
from math import *
from gmath import *

def scanline_convert(polygons, i, screen, zbuffer, color ):
    flip = False
    BOT = 0
    TOP = 2
    MID = 1

    points = [ (polygons[i][0], polygons[i][1], polygons[i][2]),
               (polygons[i+1][0], polygons[i+1][1], polygons[i+1][2]),
               (polygons[i+2][0], polygons[i+2][1], polygons[i+2][2]) ]

    # color = [0,0,0]
    # color[RED] = (23*(i/3)) %256
    # color[GREEN] = (109*(i/3)) %256
    # color[BLUE] = (227*(i/3)) %256

    points.sort(key = lambda x: x[1])
    x0 = points[BOT][0]
    z0 = points[BOT][2]
    x1 = points[BOT][0]
    z1 = points[BOT][2]
    y = int(points[BOT][1])

    distance0 = int(points[TOP][1]) - y * 1.0
    distance1 = int(points[MID][1]) - y * 1.0
    distance2 = int(points[TOP][1]) - int(points[MID][1]) * 1.0

    dx0 = (points[TOP][0] - points[BOT][0]) / distance0 if distance0 != 0 else 0
    dz0 = (points[TOP][2] - points[BOT][2]) / distance0 if distance0 != 0 else 0
    dx1 = (points[MID][0] - points[BOT][0]) / distance1 if distance1 != 0 else 0
    dz1 = (points[MID][2] - points[BOT][2]) / distance1 if distance1 != 0 else 0

    while y <= int(points[TOP][1]):

        draw_line(int(x0), y, z0, int(x1), y, z1, screen, zbuffer, color)
        x0+= dx0
        z0+= dz0
        x1+= dx1
        z1+= dz1
        y+= 1

        if ( not flip and y >= int(points[MID][1])):
            flip = True

            dx1 = (points[TOP][0] - points[MID][0]) / distance2 if distance2 != 0 else 0
            dz1 = (points[TOP][2] - points[MID][2]) / distance2 if distance2 != 0 else 0
            x1 = points[MID][0]
            z1 = points[MID][2]
            
def scanline_convertS(polygons, i, screen, zbuffer, c1,c2,c3 ):
    flip = False
    BOT = 0
    TOP = 2
    MID = 1

    points = [ (polygons[i][0], polygons[i][1], polygons[i][2]),
               (polygons[i+1][0], polygons[i+1][1], polygons[i+1][2]),
               (polygons[i+2][0], polygons[i+2][1], polygons[i+2][2]) ]

    # color = [0,0,0]
    # color[RED] = (23*(i/3)) %256
    # color[GREEN] = (109*(i/3)) %256
    # color[BLUE] = (227*(i/3)) %256

    points.sort(key = lambda x: x[1])
    x0 = points[BOT][0]
    z0 = points[BOT][2]
    x1 = points[BOT][0]
    z1 = points[BOT][2]
    y = int(points[BOT][1])
    c=[c1,c2,c3]
    #c.sort(key = lambda x: x[1])
    c0r=c[BOT][0]
    c0g=c[BOT][1]
    c0b=c[BOT][2]
    c1r=c[MID][0]
    c1g=c[MID][1]
    c1b=c[MID][2]
    c2r=c[TOP][0]
    c2g=c[TOP][1]
    c2b=c[TOP][2]
    distance0 = int(points[TOP][1]) - y * 1.0
    distance1 = int(points[MID][1]) - y * 1.0
    distance2 = int(points[TOP][1]) - int(points[MID][1]) * 1.0

    dx0 = (points[TOP][0] - points[BOT][0]) / distance0 if distance0 != 0 else 0
    dz0 = (points[TOP][2] - points[BOT][2]) / distance0 if distance0 != 0 else 0
    dx1 = (points[MID][0] - points[BOT][0]) / distance1 if distance1 != 0 else 0
    dz1 = (points[MID][2] - points[BOT][2]) / distance1 if distance1 != 0 else 0

    dc0r=(c2r-c0r)/distance0 if distance0 != 0 else 0
    dc0g=(c2g-c0g)/distance0 if distance0 != 0 else 0
    dc0b=(c2b-c0b)/distance0 if distance0 != 0 else 0
    dc1r=(c1r-c0r)/ distance1 if distance1 != 0 else 0
    dc1g=(c1g-c0g)/ distance1 if distance1 != 0 else 0
    dc1b=(c1b-c0b)/ distance1 if distance1 != 0 else 0
    while y <= int(points[TOP][1]):
        c0=[c0r,c0g,c0b]
        c1=[c1r,c1g,c1b]
        draw_lineShade(int(x0), y, z0, int(x1), y, z1, screen, zbuffer, c0,c1)
        #draw_line(int(x0), y, z0, int(x1), y, z1, screen, zbuffer, color)
        x0+= dx0
        z0+= dz0
        x1+= dx1
        z1+= dz1
        c0r+=dc0r
        c0g+=dc0g
        c0b+=dc0b
        c1r+=dc1r
        c1g+=dc1g
        c1b+=dc1b
        y+= 1

        if ( not flip and y >= int(points[MID][1])):
            flip = True

            dx1 = (points[TOP][0] - points[MID][0]) / distance2 if distance2 != 0 else 0
            dz1 = (points[TOP][2] - points[MID][2]) / distance2 if distance2 != 0 else 0
            x1 = points[MID][0]
            z1 = points[MID][2]
            dc1r=(c2r-c1r)/distance2 if distance2 != 0 else 0
            dc1g=(c2g-c1g)/distance2 if distance2 != 0 else 0
            dc1b=(c2b-c1b)/distance2 if distance2 != 0 else 0
            
# def scanline_convert(polygons, i, screen, zbuffer, color ):
#     flip = False
#     BOT = 0
#     TOP = 2
#     MID = 1

#     points = [ (polygons[i][0], polygons[i][1], polygons[i][2]),
#                (polygons[i+1][0], polygons[i+1][1], polygons[i+1][2]),
#                (polygons[i+2][0], polygons[i+2][1], polygons[i+2][2]) ]

#     # color = [0,0,0]
#     # color[RED] = (23*(i/3)) %256
#     # color[GREEN] = (109*(i/3)) %256
#     # color[BLUE] = (227*(i/3)) %256

#     points.sort(key = lambda x: x[1])
#     x0 = points[BOT][0]
#     z0 = points[BOT][2]
#     x1 = points[BOT][0]
#     z1 = points[BOT][2]
#     y = int(points[BOT][1])

#     distance0 = int(points[TOP][1]) - y * 1.0
#     distance1 = int(points[MID][1]) - y * 1.0
#     distance2 = int(points[TOP][1]) - int(points[MID][1]) * 1.0

#     dx0 = (points[TOP][0] - points[BOT][0]) / distance0 if distance0 != 0 else 0
#     dz0 = (points[TOP][2] - points[BOT][2]) / distance0 if distance0 != 0 else 0
#     dx1 = (points[MID][0] - points[BOT][0]) / distance1 if distance1 != 0 else 0
#     dz1 = (points[MID][2] - points[BOT][2]) / distance1 if distance1 != 0 else 0

#     while y <= int(points[TOP][1]):

#         draw_line(int(x0), y, z0, int(x1), y, z1, screen, zbuffer, color)
#         x0+= dx0
#         z0+= dz0
#         x1+= dx1
#         z1+= dz1
#         y+= 1

#         if ( not flip and y >= int(points[MID][1])):
#             flip = True

#             dx1 = (points[TOP][0] - points[MID][0]) / distance2 if distance2 != 0 else 0
#             dz1 = (points[TOP][2] - points[MID][2]) / distance2 if distance2 != 0 else 0
#             x1 = points[MID][0]
#             z1 = points[MID][2]

def add_polygon( polygons, x0, y0, z0, x1, y1, z1, x2, y2, z2 ):
    add_point(polygons, x0, y0, z0);
    add_point(polygons, x1, y1, z1);
    add_point(polygons, x2, y2, z2);

def processHashTable(vertices):
    toRet={}
    for key in vertices:
        surfNorms=vertices[key]
        sumVect=[0,0,0]
        for i in surfNorms:
            normalize(i)
        for j in surfNorms:
            sumVect=sumVectors(sumVect,j)
        normalize(sumVect)
        toRet[key]=sumVect
    return toRet

def draw_polygons( matrix, screen, zbuffer, view, ambient, light, areflect, dreflect, sreflect):
    if len(matrix) < 2:
        print 'Need at least 3 points to draw'
        return

    point = 0
    while point < len(matrix) - 2:

        normal = calculate_normal(matrix, point)[:]
        if dot_product(normal, view) > 0:

            color = get_lighting(normal, view, ambient, light, areflect, dreflect, sreflect )
            scanline_convert(matrix, point, screen, zbuffer, color)
        point+= 3
        
def checkFound(v,li):
    v0x=v[0]
    v0y=v[1]
    v0z=v[2]
    found=False
    for a in li:
        if float(round(v0x,3))==a[0] and float(round(v0y,3))==a[1] and float(round(v0z,3))==a[2]:
            found=True
    return found
    
def draw_polygonsS( matrix, screen, zbuffer, view, ambient, light, areflect, dreflect, sreflect):
    if len(matrix) < 2:
        print 'Need at least 3 points to draw'
        return

    point = 0
    vert={}
    while point < len(matrix) - 2:
        normal = calculate_normal(matrix, point)[:]
        v0=matrix[point]
        v1=matrix[point+1]
        v2=matrix[point+2]
        for i in range(len(v0)):
            v0[i]=float(round(v0[i],3))
        for j in range(len(v1)):
            v1[j]=float(round(v1[j],3))
        for k in range(len(v2)):
            v2[k]=float(round(v2[k],3))
        v0t=v0[0],v0[1],v0[2]
        v1t=v1[0],v1[1],v1[2]
        v2t=v2[0],v2[1],v2[2]
        surfNor0=calculate_normal(matrix,point)[:]
        #surfNor1=calculate_normal(matrix,point+1)[:]
        #surfNor2=calculate_normal(matrix,point+2)[:]
        if checkFound(v0,vert):
            #surfNor0=calculate_normal(matrix,point)
            vert[v0t].append(surfNor0)
        else:
            vert[v0t]= [surfNor0]
        if checkFound(v1,vert):
            #surfNor1=calculate_normal(matrix,point+1)
            vert[v1t ].append(surfNor0)
        else:
            vert[v1t]=[surfNor0]
        if checkFound(v2,vert):
            #surfNor2=calculate_normal(matrix,point+2)
            vert[v2t].append(surfNor0)
        else:
            vert[v2t]=[surfNor0]
        point +=3
    point=0
    processed=processHashTable(vert)
    while point < len(matrix) - 2:
            normal = calculate_normal(matrix, point)[:]
            # v0=int(matrix[point])
            # v1=int(matrix[point+1])
            # v2=int(matrix[point+2])
            v00=matrix[point]
            v11=matrix[point+1]
            v22=matrix[point+2]
            minY=min(v00[1],v11[1],v22[1])
            maxY=max(v00[1],v11[1],v22[1])
            if v00[1]==minY:
                v0=matrix[point]
                if v11[1]==maxY:
                    v2=matrix[point+1]
                    v1=matrix[point+2]
                else:
                    v2=matrix[point+2]
                    v1=matrix[point+1]
            elif v11[1]==minY:
                v0=matrix[point+1]
                if v00[1]==maxY:
                    v2=matrix[point]
                    v1=matrix[point+2]
                else:
                    v2=matrix[point+2]
                    v1=matrix[point]
            else:
                v0=matrix[point+2]
                if v11[1]==maxY:
                    v2=matrix[point+1]
                    v1=matrix[point]
                else:
                    v2=matrix[point]
                    v1=matrix[point+1]
            for i in range(len(v0)):
                v0[i]=float(round(v0[i],3))
            for j in range(len(v1)):
                v1[j]=float(round(v1[j],3))
            for k in range(len(v2)):
                v2[k]=float(round(v2[k],3))
            v0t=v0[0],v0[1],v0[2]
            v1t=v1[0],v1[1],v1[2]
            v2t=v2[0],v2[1],v2[2]
            #v0=int(matrix[point])
            #v1=int(matrix[point+1])
            #v2=int(matrix[point+2])
            if dot_product(normal, view) > 0:
                v0sumNormal=processed[v0t]
                v1sumNormal=processed[v1t]
                v2sumNormal=processed[v2t]
                i0= get_lighting(v0sumNormal, view, ambient, light, areflect, dreflect, sreflect )
                i1= get_lighting(v1sumNormal, view, ambient, light, areflect, dreflect, sreflect )
                i2= get_lighting(v2sumNormal, view, ambient, light, areflect, dreflect, sreflect )
                #color = get_lighting(normal, view, ambient, light, areflect, dreflect, sreflect )
                #scanline_convert(matrix, point, screen, zbuffer, color)
                scanline_convertS(matrix, point, screen, zbuffer, i0,i1,i2)
            point +=3


def add_box( polygons, x, y, z, width, height, depth ):
    x1 = x + width
    y1 = y - height
    z1 = z - depth

    #front
    add_polygon(polygons, x, y, z, x1, y1, z, x1, y, z);
    add_polygon(polygons, x, y, z, x, y1, z, x1, y1, z);

    #back
    add_polygon(polygons, x1, y, z1, x, y1, z1, x, y, z1);
    add_polygon(polygons, x1, y, z1, x1, y1, z1, x, y1, z1);

    #right side
    add_polygon(polygons, x1, y, z, x1, y1, z1, x1, y, z1);
    add_polygon(polygons, x1, y, z, x1, y1, z, x1, y1, z1);
    #left side
    add_polygon(polygons, x, y, z1, x, y1, z, x, y, z);
    add_polygon(polygons, x, y, z1, x, y1, z1, x, y1, z);

    #top
    add_polygon(polygons, x, y, z1, x1, y, z, x1, y, z1);
    add_polygon(polygons, x, y, z1, x, y, z, x1, y, z);
    #bottom
    add_polygon(polygons, x, y1, z, x1, y1, z1, x1, y1, z);
    add_polygon(polygons, x, y1, z, x, y1, z1, x1, y1, z1);

def add_sphere( edges, cx, cy, cz, r, step ):
    points = generate_sphere(cx, cy, cz, r, step)
    lat_start = 0
    lat_stop = step
    longt_start = 0
    longt_stop = step

    step+= 1
    for lat in range(lat_start, lat_stop):
        for longt in range(longt_start, longt_stop):

            p0 = lat * step + longt
            p1 = p0+1
            p2 = (p1+step) % (step * (step-1))
            p3 = (p0+step) % (step * (step-1))

            if longt != step - 2:
                add_polygon( edges, points[p0][0],
                             points[p0][1],
                             points[p0][2],
                             points[p1][0],
                             points[p1][1],
                             points[p1][2],
                             points[p2][0],
                             points[p2][1],
                             points[p2][2])
            if longt != 0:
                add_polygon( edges, points[p0][0],
                             points[p0][1],
                             points[p0][2],
                             points[p2][0],
                             points[p2][1],
                             points[p2][2],
                             points[p3][0],
                             points[p3][1],
                             points[p3][2])

def generate_sphere( cx, cy, cz, r, step ):
    points = []

    rot_start = 0
    rot_stop = step
    circ_start = 0
    circ_stop = step

    for rotation in range(rot_start, rot_stop):
        rot = rotation/float(step)
        for circle in range(circ_start, circ_stop+1):
            circ = circle/float(step)

            x = r * math.cos(math.pi * circ) + cx
            y = r * math.sin(math.pi * circ) * math.cos(2*math.pi * rot) + cy
            z = r * math.sin(math.pi * circ) * math.sin(2*math.pi * rot) + cz

            points.append([x, y, z])
            #print 'rotation: %d\tcircle%d'%(rotation, circle)
    return points

def add_torus( edges, cx, cy, cz, r0, r1, step ):
    points = generate_torus(cx, cy, cz, r0, r1, step)
    lat_start = 0
    lat_stop = step
    longt_start = 0
    longt_stop = step

    for lat in range(lat_start, lat_stop):
        for longt in range(longt_start, longt_stop):

            p0 = lat * step + longt;
            if (longt == (step - 1)):
                p1 = p0 - longt;
            else:
                p1 = p0 + 1;
            p2 = (p1 + step) % (step * step);
            p3 = (p0 + step) % (step * step);

            add_polygon(edges,
                        points[p0][0],
                        points[p0][1],
                        points[p0][2],
                        points[p3][0],
                        points[p3][1],
                        points[p3][2],
                        points[p2][0],
                        points[p2][1],
                        points[p2][2] )
            add_polygon(edges,
                        points[p0][0],
                        points[p0][1],
                        points[p0][2],
                        points[p2][0],
                        points[p2][1],
                        points[p2][2],
                        points[p1][0],
                        points[p1][1],
                        points[p1][2] )

def generate_torus( cx, cy, cz, r0, r1, step ):
    points = []
    rot_start = 0
    rot_stop = step
    circ_start = 0
    circ_stop = step

    for rotation in range(rot_start, rot_stop):
        rot = rotation/float(step)
        for circle in range(circ_start, circ_stop):
            circ = circle/float(step)

            x = math.cos(2*math.pi * rot) * (r0 * math.cos(2*math.pi * circ) + r1) + cx;
            y = r0 * math.sin(2*math.pi * circ) + cy;
            z = -1*math.sin(2*math.pi * rot) * (r0 * math.cos(2*math.pi * circ) + r1) + cz;

            points.append([x, y, z])
    return points

def add_circle( points, cx, cy, cz, r, step ):
    x0 = r + cx
    y0 = cy
    i = 1
    while i <= step:
        t = float(i)/step
        x1 = r * math.cos(2*math.pi * t) + cx;
        y1 = r * math.sin(2*math.pi * t) + cy;

        add_edge(points, x0, y0, cz, x1, y1, cz)
        x0 = x1
        y0 = y1
        i+= 1

def add_curve( points, x0, y0, x1, y1, x2, y2, x3, y3, step, curve_type ):

    xcoefs = generate_curve_coefs(x0, x1, x2, x3, curve_type)[0]
    ycoefs = generate_curve_coefs(y0, y1, y2, y3, curve_type)[0]

    i = 1
    while i <= step:
        t = float(i)/step
        x = xcoefs[0] * t*t*t + xcoefs[1] * t*t + xcoefs[2] * t + xcoefs[3]
        y = ycoefs[0] * t*t*t + ycoefs[1] * t*t + ycoefs[2] * t + ycoefs[3]

        add_edge(points, x0, y0, 0, x, y, 0)
        x0 = x
        y0 = y
        i+= 1


def draw_lines( matrix, screen, zbuffer, color ):
    if len(matrix) < 2:
        print 'Need at least 2 points to draw'
        return

    point = 0
    while point < len(matrix) - 1:
        draw_line( int(matrix[point][0]),
                   int(matrix[point][1]),
                   matrix[point][2],
                   int(matrix[point+1][0]),
                   int(matrix[point+1][1]),
                   matrix[point+1][2],
                   screen, zbuffer, color)
        point+= 2

def add_edge( matrix, x0, y0, z0, x1, y1, z1 ):
    add_point(matrix, x0, y0, z0)
    add_point(matrix, x1, y1, z1)

def add_point( matrix, x, y, z=0 ):
    matrix.append( [x, y, z, 1] )

def draw_lineShade( x0, y0, z0, x1, y1, z1, screen, zbuffer, colorLeft, colorRight ):
    x=x0
    y=y0
    smallX=min(x0,x1)
    bigX=max(x0,x1)
    dist=bigX-smallX
    colorR=colorLeft[0]
    colorG=colorLeft[1]
    colorB=colorLeft[2]
    colorRightr=colorRight[0]
    colorRightg=colorRight[1]
    colorRightb=colorRight[2]
    colorLeftr=colorLeft[0]
    colorLeftg=colorLeft[1]
    colorLeftb=colorLeft[2]
    dcr=(colorRightr-colorLeftr)/dist if dist!=0 else 0
    dcg=(colorRightg-colorLeftg)/dist if dist!=0 else 0
    dcb=(colorRightb-colorLeftb)/dist if dist!=0 else 0
    for i in range(smallX,bigX+1):
        color=[colorR,colorG,colorB]
        plot(screen,zbuffer,color,x,y0,z0)
        colorR+=dcr
        colorG+=dcg
        colorB+=dcb
        x+=1
    
def draw_line( x0, y0, z0, x1, y1, z1, screen, zbuffer, color ):

    #swap points if going right -> left
    if x0 > x1:
        xt = x0
        yt = y0
        zt = z0
        x0 = x1
        y0 = y1
        z0 = z1
        x1 = xt
        y1 = yt
        z1 = zt

    x = x0
    y = y0
    z = z0
    A = 2 * (y1 - y0)
    B = -2 * (x1 - x0)
    wide = False
    tall = False

    if ( abs(x1-x0) >= abs(y1 - y0) ): #octants 1/8
        wide = True
        loop_start = x
        loop_end = x1
        dx_east = dx_northeast = 1
        dy_east = 0
        d_east = A
        distance = x1 - x
        if ( A > 0 ): #octant 1
            d = A + B/2
            dy_northeast = 1
            d_northeast = A + B
        else: #octant 8
            d = A - B/2
            dy_northeast = -1
            d_northeast = A - B

    else: #octants 2/7
        tall = True
        dx_east = 0
        dx_northeast = 1
        distance = abs(y1 - y)
        if ( A > 0 ): #octant 2
            d = A/2 + B
            dy_east = dy_northeast = 1
            d_northeast = A + B
            d_east = B
            loop_start = y
            loop_end = y1
        else: #octant 7
            d = A/2 - B
            dy_east = dy_northeast = -1
            d_northeast = A - B
            d_east = -1 * B
            loop_start = y1
            loop_end = y

    dz = (z1 - z0) / distance if distance != 0 else 0

    while ( loop_start < loop_end ):
        plot( screen, zbuffer, color, x, y, z )
        if ( (wide and ((A > 0 and d > 0) or (A < 0 and d < 0))) or
             (tall and ((A > 0 and d < 0) or (A < 0 and d > 0 )))):

            x+= dx_northeast
            y+= dy_northeast
            d+= d_northeast
        else:
            x+= dx_east
            y+= dy_east
            d+= d_east
        z+= dz
        loop_start+= 1
    plot( screen, zbuffer, color, x, y, z )

def addMeshPoly(poly,verList,i1,i2,i3):
    ver1=verList[i1-1]
    ver2=verList[i2-1]
    ver3=verList[i3-1]
    add_polygon(poly,float(ver1[0]),float(ver1[1]),float(ver1[2]),float(ver2[0]),float(ver2[1]),float(ver2[2]),float(ver3[0]),float(ver3[1]),float(ver3[2]))
    #print "len "+str(len(verList))
    #if i3>=len(verList)-1:
    #print "v3 "+str(i3)
        
def drawMesh(fileName, polyList, stack):
    vertices=[]
    faces=[]
    f=open(fileName)
    lines=f.readlines()
    c=0
    while c<len(lines):
        line=lines[c].strip()
        #print line
        arrLine=line.split(" ")
        if arrLine[0]=="v":
            vertices.append([arrLine[1],arrLine[2],arrLine[3]])
        elif arrLine[0]=="f":
            if len(arrLine)==5:
                v1=arrLine[1]
                v2=arrLine[2]
                v3=arrLine[3]
                v4=arrLine[4]
                faces.append([v1,v2,v3])
                faces.append([v1,v3,v4])
            elif len(arrLine)==6:
                v1=arrLine[1]
                v2=arrLine[2]
                v3=arrLine[3]
                v4=arrLine[4]
                v5=arrLine[5]
                faces.append([v1,v2,v3])
                faces.append([v1,v3,v4])
                faces.append([v1,v4,v5])
            elif len(arrLine)==7:
                v1=arrLine[1]
                v2=arrLine[2]
                v3=arrLine[3]
                v4=arrLine[4]
                v5=arrLine[5]
                v6=arrLine[6]
                faces.append([v1,v2,v3])
                faces.append([v1,v3,v4])
                faces.append([v1,v4,v5])
                faces.append([v1,v5,v6])
            else:
            #print arrLine[1].split("//")[0]
                if "//" in arrLine[1]:
                    v1=arrLine[1].split("//")[0]
                else:
                    v1=arrLine[1]
                if "//" in arrLine[2]:
                    v2=arrLine[2].split("//")[0]
                else:
                    v2=arrLine[2]
                if "//" in arrLine[3]:
                    v3=arrLine[3].split("//")[0]
                else:
                    v3=arrLine[3]
                faces.append([v1,v2,v3])
        c+=1
    for face in faces:
        ver1=face[0]
        ver2=face[1]
        ver3=face[2]
        addMeshPoly(polyList,vertices,int(ver1),int(ver2),int(ver3))
        #matrix_mult( stack[-1], polyList )
    #draw_polygons(polyList,screen, zbuffer, view, ambient, light, areflect, dreflect, sreflect)
        
# def drawMesh(fileName):
#     vectors=[]
#     faces=[]
#     f=open(fileName)
#     lines=f.readlines()
#     c=0
#     while c<len(lines):
#         line=lines[c].strip()
#         print line
#         arrLine=line.split(" ")
#         if arrLine[0]=="v":
#             vectors.append([arrLine[1],arrLine[2],arrLine[3]])
#         elif arrLine[0]=="f":
#             #print arrLine[1].split("//")[0]
#             v1=arrLine[1].split("//")[0]
#             v2=arrLine[2].split("//")[0]
#             v3=arrLine[3].split("//")[0]
#             faces.append([v1,v2,v3])
#         c+=1

# view = [0,
#         0,
#         1];
# ambient = [50,
#            50,
#            50]
# light = [[0.5,
#           0.75,
#           1],#[0,191,255]]
#          [0,
#           255,
#           255]]
# areflect = [0.1,
#             0.1,
#             0.1]
# dreflect = [0.5,
#             0.5,
#             0.5]
# sreflect = [0.5,
#             0.5,
#             0.5]

# color = [0, 0, 0]
# tmp = new_matrix()
# ident( tmp )
# polygons=[]
# stack = [ [x[:] for x in tmp] ]
# screen = new_screen()
# zbuffer = new_zbuffer()
# tmp = []

# drawMesh("airboat.obj",tmp, screen, zbuffer, view, ambient, light, areflect, dreflect, sreflect)
