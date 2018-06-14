# Graphics Final Project: Lamp
###### Jenny Gao
###### Period 5

### Features
+ Mesh: allows programmer to specify a polygon mesh defined in an external file
```
mesh :filename.obj
```
+ Shading

For flat shading, programmer should NOT add a line in the script regarding shading

For gouraud shading, programmer should add this line
```
shading gouraud
```
<br>
Note: For Gouraud Shading, I implemented a hash table and interpolation. Currently, it's close to a smooth surface (e.g. you can test a sphere and see); I couldn't get it to be perfectly smooth.
