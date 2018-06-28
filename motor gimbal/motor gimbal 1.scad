$fn =35;

innerDiam = 33;
outterDiam = 40;
union() {difference() {
    union() {
        cylinder( r = outterDiam/2, h = 10);
        translate([-outterDiam/2,0,-34]) cube([(outterDiam/2)+8, 30, 84]);
        
        //bout
        translate ([20,0 , 5])rotate([90,90,0])bout();
        translate ([-20, 0, 5])rotate([-90,90,0])bout();
    }
    translate ([0,0, -45]) cylinder( r = 46/2, h = 45);
    cylinder( r = innerDiam/2, h = 140);
    //bout
    translate ([innerDiam/2, 0, 5])rotate([90,90,0])bout();
    translate ([-innerDiam/2, 0, 5])rotate([-90,90,0])bout();
    //holes
    #translate ([-50, 0, 5]) rotate([90,0,90])cylinder(r=3/2, h= 100);
    #translate ([-0, -50, 5])rotate([0,90,90])cylinder(r=3/2, h= 100);
    
     #translate ([-0, 20, 5])rotate([0,90,90])cylinder(r=8/2, h= 20);
    
    //servo
   #translate([-(innerDiam/2 -4),(innerDiam/2)-2,21])cube([22,16,24]);
    #translate([-16,0,10])cube([25,18,40]);
    translate ([0,0, -25]) difference() {
    cylinder ( r = 36, h = 100);
       cylinder ( r = 32, h = 100);
    }
    //servo holes
    # translate([10,24,19.5])rotate([0,-90,0])cylinder(r=1.5/2, h = 10);
    translate([10,24,46.5])rotate([0,-90,0])cylinder(r=1.5/2, h = 10);
    //outer gim
    #difference() {
        cylinder(r=46/2, h =15);
        cylinder(r=outterDiam/2 , h=15);
        }
    
    #translate([-20,20,-40])cube([30,4,50]);
        #rotate([0,15,0])translate([-25,20,-30])cube([30,4,40]);
        #rotate([0,-15,0])translate([-5,20,-27])cube([30,4,40]);
        #translate([-22,11,-35])cube([6,30,50]);
        #translate([-17,16,-35])cube([6,30,50]);
        #translate([-18,22.5,-35])cube([30,5,30]);
     
    
}

//bout
        //translate ([20,0 , 5])rotate([90,90,0])bout();
        //translate ([-20, 0, 5])rotate([-90,90,0])bout();


    translate ([0,(outterDiam/2)+1 , 5])rotate([0,90,0])bout();
    translate ([0, -((outterDiam/2)+1), 5])rotate([-180,90,0])bout();
    


}

module bout() {
    difference() {
rotate([90,0,0]) cylinder (r1= 4/2, r2 =7, h = 3);
        #rotate([90,0,0])translate([-9,-6,0])cube([4,14, 5]);
        #rotate([90,0,0])translate([5,-6,0]) cube([4,12, 5]);
        rotate([90,0,0]) cylinder (r= 3/2,  h = 10);
        
    }
}
//bout();