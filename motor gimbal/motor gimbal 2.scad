$fn=75;
innerDiam = 42;
outterDiam=47;
outterDiam1=52;
difference () {
    union () {cylinder(r=outterDiam1/2, h=10); 
        translate([-outterDiam/2+9,0,0]) cube([outterDiam/2+3, outterDiam/2 +12, 40]);
    }
    cylinder(r=innerDiam/2, h=100, center = true); 
    translate([0,0,10])cylinder(r=outterDiam/2, h=100); 
    translate([-outterDiam/2,0,10])cube([outterDiam/2, outterDiam/2 , 44]);
    #translate([-((innerDiam/2)-6),outterDiam/2,10])cube([20,12,24]);
    
    //holes
    #translate ([-50, 0, 5]) rotate([90,0,90])cylinder(r=3/2, h= 100);
   
    //servo holes
    # translate([-11,28,8.5])rotate([0,-90,0])cylinder(r=1.5/2, h = 4);
    #translate([-11,28,35.5])rotate([0,-90,0])cylinder(r=1.5/2, h = 4);
    #difference() {
        
        cube([outterDiam1,outterDiam1,20], center=true);
    cube([outterDiam,outterDiam1,20], center=true);
    }
    #translate([0,18,0]) cylinder (r= 6, h =80, center = true);
    
    #translate([0,-18,0]) cylinder (r= 6, h =80, center = true);
    
    //outer gim
   /* #difference() {
        cylinder(r=82/2, h =10);
        cylinder(r=outterDiam1/2 , h=10);
        }*/
}