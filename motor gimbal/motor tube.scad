$fn=100;
difference() {
union() {
    cylinder(r=30/2, h =100);
    //bottom hole
    #translate ([0, -12, 11])rotate([90,0,0]) cylinder (r1= 7, r2 =4, h = 5);
    #translate ([0, -12, 50])rotate([90,0,0]) cylinder (r1= 7, r2 =5/2, h = 4);
    #translate ([0, 16, 50])rotate([90,0,0]) cylinder (r1= 5/2, r2 =7, h = 4);
}
cylinder(r=26/2, h =100);
    #translate([0,25,50])rotate([90,0,0]) cylinder (r= 2/2, h = 50);
#translate ([0, -10, 11])rotate([90,0,0]) cylinder (r= 1.5/2, h = 10);
}    