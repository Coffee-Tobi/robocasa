// RoboBarista IR40-50 shared grip concept, millimetres.
// CONCEPT ONLY. Not load-rated, not dimensioned to vendor CAD.
// part: "handle", "finger", "assembly"; mount: "portafilter" or "pitcher".
part="assembly";
mount="portafilter";
grip_width=24; // across closing direction (Y)
grip_height=24;
grip_length=48;
stop_width=3;
stop_height=3;
clearance=0.5;
finger_thickness=8;
finger_length=34;
$fn=48;

module handle() {
  color("dimgray")
  union() {
    // Positive stops in the tool axis direction X.
    translate([-grip_length/2,0,0]) cube([stop_width,grip_width+2*stop_height,grip_height+2*stop_height],center=true);
    translate([grip_length/2,0,0]) cube([stop_width,grip_width+2*stop_height,grip_height+2*stop_height],center=true);
    cube([grip_length,grip_width,grip_height],center=true);
    // Conceptual mounting stem, NOT a real Decent thread or pitcher fastener.
    translate([-grip_length/2-12,0,0]) rotate([0,90,0]) cylinder(d=12,h=12);
  }
}
module finger(side=1) {
  color("teal")
  difference() {
    translate([0,side*(grip_width/2+clearance+finger_thickness/2),0])
      cube([finger_length,finger_thickness,grip_height+10],center=true);
    // shallow matching longitudinal anti-slip channel; provisional
    translate([0,side*(grip_width/2+clearance+0.6),0])
      cube([finger_length-6,2,grip_height-2],center=true);
  }
}
if(part=="handle") handle();
else if(part=="finger") finger(1);
else {handle();finger(1);finger(-1);}
